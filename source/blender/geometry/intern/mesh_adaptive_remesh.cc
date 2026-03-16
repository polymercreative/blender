/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup geo
 *
 * Adaptive mesh remeshing based on dyntopo algorithm from sculpt mode.
 * Performs iterative edge splits and collapses to achieve target edge lengths.
 */

#include "GEO_mesh_adaptive_remesh.hh"

#include "DNA_modifier_types.h"

#include "BKE_customdata.hh"
#include "BKE_mesh.hh"

#include "BLI_heap_simple.h"
#include "BLI_math_vector.h"
#include "BLI_mempool.h"

#include "bmesh.hh"
#include "bmesh_tools.hh"

namespace blender::geometry {

/* Temporary edge queue structure for remeshing. */
struct EdgeQueueContext {
  HeapSimple *heap;
  BLI_mempool *pool;  /* Memory pool for vertex pairs */
  int cd_target_offset;
  int cd_mask_offset;
  float mask_threshold; /* Only remesh vertices with mask > threshold */
};

/* Check if a vertex should be remeshed based on its mask value. */
static bool vert_is_masked_for_remesh(BMVert *v, const EdgeQueueContext *eq_ctx)
{
  if (eq_ctx->cd_mask_offset == -1) {
    return true; /* No mask = remesh everything */
  }
  const float mask = BM_ELEM_CD_GET_FLOAT(v, eq_ctx->cd_mask_offset);
  return mask >= eq_ctx->mask_threshold;
}

/* Check if an edge should be processed (both verts must be masked for remesh). */
static bool edge_in_remesh_region(BMVert *v1, BMVert *v2, const EdgeQueueContext *eq_ctx)
{
  return vert_is_masked_for_remesh(v1, eq_ctx) && vert_is_masked_for_remesh(v2, eq_ctx);
}

/* Get target edge length for an edge (average of both vertices). */
static float edge_target_length(BMVert *v1, BMVert *v2, const EdgeQueueContext *eq_ctx)
{
  if (eq_ctx->cd_target_offset == -1) {
    return 0.1f; /* Default if no target length attribute - small to encourage subdivision */
  }
  const float len1 = BM_ELEM_CD_GET_FLOAT(v1, eq_ctx->cd_target_offset);
  const float len2 = BM_ELEM_CD_GET_FLOAT(v2, eq_ctx->cd_target_offset);
  return (len1 + len2) * 0.5f;
}

/* Check if edge is already queued (using BM_ELEM_TAG). */
static bool edge_queue_test(BMEdge *e)
{
  return BM_elem_flag_test(e, BM_ELEM_TAG);
}

/* Mark edge as queued. */
static void edge_queue_enable(BMEdge *e)
{
  BM_elem_flag_enable(e, BM_ELEM_TAG);
}

/* Mark edge as not queued. */
static void edge_queue_disable(BMEdge *e)
{
  BM_elem_flag_disable(e, BM_ELEM_TAG);
}

/* Insert edge into queue as vertex pair. */
static void edge_queue_insert(EdgeQueueContext *eq_ctx, BMEdge *e, float priority)
{
  /* Allocate vertex pair from pool */
  BMVert **pair = static_cast<BMVert **>(BLI_mempool_alloc(eq_ctx->pool));
  pair[0] = e->v1;
  pair[1] = e->v2;

  edge_queue_enable(e);
  BLI_heapsimple_insert(eq_ctx->heap, priority, pair);
}

/* Insert edge into long edge queue if needed. */
static void long_edge_queue_edge_add(EdgeQueueContext *eq_ctx, BMEdge *e, float split_threshold)
{
  if (edge_queue_test(e) || !edge_in_remesh_region(e->v1, e->v2, eq_ctx)) {
    return;
  }

  const float target_len = edge_target_length(e->v1, e->v2, eq_ctx);
  const float max_len = target_len * split_threshold;
  const float edge_len = BM_edge_calc_length(e);

  if (edge_len > max_len) {
    /* Negative priority = longer edges processed first. */
    edge_queue_insert(eq_ctx, e, -edge_len);
  }
}

/* Insert edge into collapse queue if needed. */
static void short_edge_queue_edge_add(EdgeQueueContext *eq_ctx,
                                      BMEdge *e,
                                      float collapse_threshold)
{
  if (edge_queue_test(e) || !edge_in_remesh_region(e->v1, e->v2, eq_ctx)) {
    return;
  }

  const float target_len = edge_target_length(e->v1, e->v2, eq_ctx);
  const float min_len = target_len * collapse_threshold;
  const float edge_len = BM_edge_calc_length(e);

  if (edge_len < min_len) {
    /* Positive priority = shorter edges processed first. */
    edge_queue_insert(eq_ctx, e, edge_len);
  }
}

/* Build initial queue of edges that are too long. */
static void build_long_edge_queue(EdgeQueueContext *eq_ctx, BMesh *bm, float split_threshold)
{
  BMIter iter;
  BMEdge *e;

  /* Clear all edge tags first. */
  BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
    edge_queue_disable(e);
  }

  /* Add long edges to queue. */
  BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
    long_edge_queue_edge_add(eq_ctx, e, split_threshold);
  }
}

/* Build initial queue of edges that are too short. */
static void build_short_edge_queue(EdgeQueueContext *eq_ctx, BMesh *bm, float collapse_threshold)
{
  BMIter iter;
  BMEdge *e;

  /* Clear all edge tags first. */
  BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
    edge_queue_disable(e);
  }

  /* Add short edges to queue. */
  BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
    short_edge_queue_edge_add(eq_ctx, e, collapse_threshold);
  }
}

/* Check if vertex is on a boundary. */
static bool vert_is_boundary(BMVert *v)
{
  BMIter iter;
  BMEdge *e;
  BM_ITER_ELEM (e, &iter, v, BM_EDGES_OF_VERT) {
    if (BM_edge_is_boundary(e)) {
      return true;
    }
  }
  return false;
}

/* Split an edge properly by subdividing adjacent triangular faces.
 * Based on dyntopo's pbvh_bmesh_split_edge. */
static bool split_edge(BMesh *bm,
                       BMVert *v1,
                       BMVert *v2,
                       EdgeQueueContext *eq_ctx,
                       float split_threshold)
{
  /* Find the edge (it might have been deleted). */
  BMEdge *e = BM_edge_exists(v1, v2);
  if (!e) {
    return false;
  }

  /* Gather all faces using this edge before we modify topology. */
  Vector<BMFace *> edge_faces;
  BMIter iter;
  BMFace *f;
  BM_ITER_ELEM (f, &iter, e, BM_FACES_OF_EDGE) {
    /* Only handle triangular faces (mesh should be triangulated). */
    if (f->len == 3) {
      edge_faces.append(f);
    }
  }

  if (edge_faces.is_empty()) {
    return false;  /* Edge has no triangular faces */
  }

  /* Create new vertex at edge midpoint. */
  float midpoint[3];
  mid_v3_v3v3(midpoint, v1->co, v2->co);

  BMVert *v_new = BM_vert_create(bm, midpoint, nullptr, BM_CREATE_NOP);
  if (!v_new) {
    return false;
  }

  /* Interpolate custom data for new vertex. */
  if (eq_ctx->cd_mask_offset != -1) {
    const float mask1 = BM_ELEM_CD_GET_FLOAT(v1, eq_ctx->cd_mask_offset);
    const float mask2 = BM_ELEM_CD_GET_FLOAT(v2, eq_ctx->cd_mask_offset);
    BM_ELEM_CD_SET_FLOAT(v_new, eq_ctx->cd_mask_offset, (mask1 + mask2) * 0.5f);
  }
  if (eq_ctx->cd_target_offset != -1) {
    const float len1 = BM_ELEM_CD_GET_FLOAT(v1, eq_ctx->cd_target_offset);
    const float len2 = BM_ELEM_CD_GET_FLOAT(v2, eq_ctx->cd_target_offset);
    BM_ELEM_CD_SET_FLOAT(v_new, eq_ctx->cd_target_offset, (len1 + len2) * 0.5f);
  }

  /* For each triangular face, split it into two new triangular faces.
   * This follows dyntopo's exact approach:
   *         + v_opp
   *        /|\
   *       / | \
   *      /  |  \
   *   e4/   |   \ e3
   *    /    |e5  \
   *   /     |     \
   *  /  e1  |  e2  \
   * +-------+-------+
   * v1      v_new     v2
   *
   * - f_new_first:  (v1, v_new, v_opp) with edges (e1, e5, e4)
   * - f_new_second: (v_new, v2, v_opp) with edges (e2, e3, e5)
   * Note: e5 is shared between both faces
   */
  for (BMFace *f_old : edge_faces) {
    /* Find the loop for this edge in the face. */
    BMLoop *l_edge = nullptr;
    BMLoop *l_iter = BM_FACE_FIRST_LOOP(f_old);
    do {
      if (l_iter->e == e) {
        l_edge = l_iter;
        break;
      }
    } while ((l_iter = l_iter->next) != BM_FACE_FIRST_LOOP(f_old));

    if (!l_edge) {
      continue;
    }

    /* Get vertices in correct winding order:
     * l_edge->v is first vertex of edge
     * l_edge->next->v is second vertex of edge
     * l_edge->prev->v is the opposite vertex */
    BMVert *v1_face = l_edge->v;
    BMVert *v2_face = l_edge->next->v;
    BMVert *v_opp = l_edge->prev->v;

    /* Create edges for first triangle (v1, v_new, v_opp).
     * BM_CREATE_NO_DOUBLE ensures we reuse existing edges. */
    BMEdge *e1 = BM_edge_create(bm, v1_face, v_new, nullptr, BM_CREATE_NO_DOUBLE);
    BMEdge *e5 = BM_edge_create(bm, v_new, v_opp, nullptr, BM_CREATE_NO_DOUBLE);
    BMEdge *e4 = BM_edge_create(bm, v_opp, v1_face, nullptr, BM_CREATE_NO_DOUBLE);

    /* Create first face using explicit edge array. */
    BMVert *verts_first[3] = {v1_face, v_new, v_opp};
    BMEdge *edges_first[3] = {e1, e5, e4};
    BMFace *f_new_first = BM_face_create(
        bm, verts_first, edges_first, 3, f_old, BM_CREATE_NOP);

    /* Create edges for second triangle (v_new, v2, v_opp).
     * e5 is reused from first triangle. */
    BMEdge *e2 = BM_edge_create(bm, v_new, v2_face, nullptr, BM_CREATE_NO_DOUBLE);
    BMEdge *e3 = BM_edge_create(bm, v2_face, v_opp, nullptr, BM_CREATE_NO_DOUBLE);

    /* Create second face using explicit edge array. */
    BMVert *verts_second[3] = {v_new, v2_face, v_opp};
    BMEdge *edges_second[3] = {e2, e3, e5};
    BMFace *f_new_second = BM_face_create(
        bm, verts_second, edges_second, 3, f_old, BM_CREATE_NOP);

    /* If face creation failed, skip this split to avoid corrupting mesh. */
    if (!f_new_first || !f_new_second) {
      continue;
    }

    /* Delete the old face. */
    BM_face_kill(bm, f_old);

    /* Add all new edges to the queue for potential further subdivision. */
    long_edge_queue_edge_add(eq_ctx, e1, split_threshold);
    long_edge_queue_edge_add(eq_ctx, e4, split_threshold);
    long_edge_queue_edge_add(eq_ctx, e5, split_threshold);
    long_edge_queue_edge_add(eq_ctx, e2, split_threshold);
    long_edge_queue_edge_add(eq_ctx, e3, split_threshold);
  }

  /* Kill the original edge now that all faces have been subdivided. */
  BM_edge_kill(bm, e);

  return true;
}

/* Collapse an edge. */
static bool collapse_edge(BMesh *bm,
                          BMVert *v1,
                          BMVert *v2,
                          EdgeQueueContext *eq_ctx,
                          bool preserve_boundaries,
                          float collapse_threshold)
{
  /* Find the edge (it might have been deleted). */
  BMEdge *e = BM_edge_exists(v1, v2);
  if (!e || !e->l) {
    return false;
  }

  /* Check boundary preservation. */
  if (preserve_boundaries) {
    const bool v1_boundary = vert_is_boundary(v1);
    const bool v2_boundary = vert_is_boundary(v2);

    /* Don't collapse boundary edges. */
    if (v1_boundary && v2_boundary) {
      return false;
    }
  }

  /* Decide which vertex to keep. */
  BMVert *v_del = v1;
  BMVert *v_conn = v2;

  /* Prefer keeping boundary vertices. */
  if (preserve_boundaries && vert_is_boundary(v2)) {
    v_del = v1;
    v_conn = v2;
  }
  else if (preserve_boundaries && vert_is_boundary(v1)) {
    v_del = v2;
    v_conn = v1;
  }

  /* Collapse the edge. */
  BMVert *v_result = BM_edge_collapse(bm, e, v_del, true, true);
  if (!v_result) {
    return false;
  }

  /* Add adjacent edges to queue - topology has changed. */
  BMIter iter;
  BMEdge *e_adj;
  BM_ITER_ELEM (e_adj, &iter, v_result, BM_EDGES_OF_VERT) {
    short_edge_queue_edge_add(eq_ctx, e_adj, collapse_threshold);
  }

  return true;
}

/* Subdivide long edges using dynamic queue. */
static int subdivide_long_edges(BMesh *bm, EdgeQueueContext *eq_ctx, float split_threshold)
{
  build_long_edge_queue(eq_ctx, bm, split_threshold);

  int split_count = 0;
  while (!BLI_heapsimple_is_empty(eq_ctx->heap)) {
    BMVert **pair = static_cast<BMVert **>(BLI_heapsimple_pop_min(eq_ctx->heap));
    BMVert *v1 = pair[0];
    BMVert *v2 = pair[1];

    /* Free the pair back to pool */
    BLI_mempool_free(eq_ctx->pool, pair);

    /* Check that the edge still exists */
    BMEdge *e = BM_edge_exists(v1, v2);
    if (!e) {
      continue;
    }

    edge_queue_disable(e);

    /* Re-check edge length and region. */
    if (!edge_in_remesh_region(v1, v2, eq_ctx)) {
      continue;
    }

    const float target_len = edge_target_length(v1, v2, eq_ctx);
    const float max_len = target_len * split_threshold;
    const float edge_len = BM_edge_calc_length(e);

    if (edge_len > max_len) {
      if (split_edge(bm, v1, v2, eq_ctx, split_threshold)) {
        split_count++;
      }
    }
  }

  return split_count;
}

/* Collapse short edges using dynamic queue. */
static int collapse_short_edges(BMesh *bm,
                                EdgeQueueContext *eq_ctx,
                                float collapse_threshold,
                                bool preserve_boundaries)
{
  build_short_edge_queue(eq_ctx, bm, collapse_threshold);

  int collapse_count = 0;
  while (!BLI_heapsimple_is_empty(eq_ctx->heap)) {
    BMVert **pair = static_cast<BMVert **>(BLI_heapsimple_pop_min(eq_ctx->heap));
    BMVert *v1 = pair[0];
    BMVert *v2 = pair[1];

    /* Free the pair back to pool */
    BLI_mempool_free(eq_ctx->pool, pair);

    /* Check that the edge still exists */
    BMEdge *e = BM_edge_exists(v1, v2);
    if (!e || !e->l) {
      continue;
    }

    edge_queue_disable(e);

    /* Re-check edge length and region. */
    if (!edge_in_remesh_region(v1, v2, eq_ctx)) {
      continue;
    }

    const float target_len = edge_target_length(v1, v2, eq_ctx);
    const float min_len = target_len * collapse_threshold;
    const float edge_len = BM_edge_calc_length(e);

    if (edge_len < min_len) {
      if (collapse_edge(bm, v1, v2, eq_ctx, preserve_boundaries, collapse_threshold)) {
        collapse_count++;
      }
    }
  }

  return collapse_count;
}

/* Calculate vertex curvature based on deviation from average neighbor plane.
 * Returns curvature value (higher = more curved). */
static float calculate_vertex_curvature(BMVert *v)
{
  /* Need at least 3 neighbors to calculate curvature. */
  if (BM_vert_edge_count(v) < 3) {
    return 1.0f; /* High curvature for boundary/pole vertices */
  }

  /* Calculate average normal of adjacent faces. */
  float avg_normal[3] = {0.0f, 0.0f, 0.0f};
  int face_count = 0;

  BMIter iter;
  BMFace *f;
  BM_ITER_ELEM (f, &iter, v, BM_FACES_OF_VERT) {
    add_v3_v3(avg_normal, f->no);
    face_count++;
  }

  if (face_count == 0) {
    return 1.0f;
  }

  normalize_v3(avg_normal);

  /* Calculate deviation of each face normal from average.
   * Higher deviation = higher curvature. */
  float curvature = 0.0f;
  BM_ITER_ELEM (f, &iter, v, BM_FACES_OF_VERT) {
    const float deviation = 1.0f - dot_v3v3(f->no, avg_normal);
    curvature += deviation;
  }

  return curvature / float(face_count);
}

/* Dissolve low-curvature vertices to simplify flat regions.
 * This reduces polycount while preserving detail in curved areas. */
static int dissolve_low_curvature_verts(BMesh *bm, float curvature_threshold)
{
  int dissolved_count = 0;

  /* Build list of vertices to check (can't modify during iteration). */
  Vector<BMVert *> verts_to_check;
  BMIter iter;
  BMVert *v;
  BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
    verts_to_check.append(v);
  }

  for (BMVert *v : verts_to_check) {
    /* Skip if vertex was already deleted. */
    if (!BM_elem_flag_test(v, BM_ELEM_TAG)) {
      continue;
    }

    /* Skip boundary vertices. */
    if (BM_vert_is_boundary(v)) {
      continue;
    }

    /* Calculate curvature. */
    const float curvature = calculate_vertex_curvature(v);

    /* If curvature is below threshold, try to dissolve. */
    if (curvature < curvature_threshold) {
      /* Only dissolve if vertex has exactly 4 edges (quad-like topology). */
      const int edge_count = BM_vert_edge_count(v);
      if (edge_count == 4) {
        /* Dissolve vertex and surrounding faces. */
        if (BM_vert_dissolve(bm, v)) {
          dissolved_count++;
        }
      }
    }
  }

  return dissolved_count;
}

/* Transfer mesh attributes to BMesh custom data layers. */
static void transfer_attributes_to_bmesh(BMesh *bm,
                                         const Mesh &mesh,
                                         const VArray<float> &remesh_mask,
                                         const VArray<float> &target_edge_length,
                                         int *r_cd_mask_offset,
                                         int *r_cd_target_offset)
{
  /* Ensure custom data layers exist. */
  BM_data_layer_ensure_named(bm, &bm->vdata, CD_PROP_FLOAT, ".remesh_mask");
  BM_data_layer_ensure_named(bm, &bm->vdata, CD_PROP_FLOAT, ".target_edge_length");

  /* Get offsets for accessing the data. */
  *r_cd_mask_offset = CustomData_get_offset_named(&bm->vdata, CD_PROP_FLOAT, ".remesh_mask");
  *r_cd_target_offset = CustomData_get_offset_named(
      &bm->vdata, CD_PROP_FLOAT, ".target_edge_length");

  /* Transfer values from VArrays to BMesh. */
  BMIter iter;
  BMVert *v;
  int i = 0;
  BM_ITER_MESH_INDEX (v, &iter, bm, BM_VERTS_OF_MESH, i) {
    if (i < mesh.verts_num) {
      BM_ELEM_CD_SET_FLOAT(v, *r_cd_mask_offset, remesh_mask[i]);
      BM_ELEM_CD_SET_FLOAT(v, *r_cd_target_offset, target_edge_length[i]);
    }
  }
}

Mesh *adaptive_remesh_mesh(const Mesh &mesh,
                           const VArray<float> &remesh_mask,
                           const VArray<float> &target_edge_length,
                           int iterations,
                           bool preserve_boundaries)
{
  /* Convert mesh to BMesh for topology operations. */
  const BMAllocTemplate allocsize = BMALLOC_TEMPLATE_FROM_ME(&mesh);

  BMeshCreateParams bmesh_create_params{};
  bmesh_create_params.use_toolflags = false;
  BMesh *bm = BM_mesh_create(&allocsize, &bmesh_create_params);

  BMeshFromMeshParams bmesh_from_mesh_params{};
  bmesh_from_mesh_params.calc_face_normal = true;
  bmesh_from_mesh_params.calc_vert_normal = true;
  BM_mesh_bm_from_me(bm, &mesh, &bmesh_from_mesh_params);

  /* Triangulate mesh (required for remeshing algorithm). */
  BM_mesh_triangulate(
      bm, MOD_TRIANGULATE_QUAD_BEAUTY, MOD_TRIANGULATE_NGON_EARCLIP, 4, false, nullptr, nullptr, nullptr);

  /* Transfer attributes to BMesh. */
  int cd_mask_offset = -1;
  int cd_target_offset = -1;
  transfer_attributes_to_bmesh(
      bm, mesh, remesh_mask, target_edge_length, &cd_mask_offset, &cd_target_offset);

  /* Setup edge queue context with memory pool for vertex pairs. */
  EdgeQueueContext eq_ctx{};
  eq_ctx.heap = BLI_heapsimple_new();
  eq_ctx.pool = BLI_mempool_create(sizeof(BMVert *) * 2, 0, 128, BLI_MEMPOOL_NOP);
  eq_ctx.cd_mask_offset = cd_mask_offset;
  eq_ctx.cd_target_offset = cd_target_offset;
  eq_ctx.mask_threshold = 0.5f; /* Only remesh vertices with mask >= 0.5 */

  /* Perform iterative remeshing.
   * Each iteration: collapse short edges first, then split long edges.
   * This order matches dyntopo's approach and is more efficient.
   * Thresholds match dyntopo behavior. */
  const float split_threshold = 1.33f;     /* Split edges longer than 1.33x target */
  const float collapse_threshold = 0.75f;  /* Collapse edges shorter than 0.75x target */

  for (int i = 0; i < iterations; i++) {
    /* Collapse short edges first (dyntopo does this before subdivision). */
    BLI_heapsimple_clear(eq_ctx.heap, nullptr);
    collapse_short_edges(bm, &eq_ctx, collapse_threshold, preserve_boundaries);

    /* Then split long edges. */
    BLI_heapsimple_clear(eq_ctx.heap, nullptr);
    subdivide_long_edges(bm, &eq_ctx, split_threshold);
  }

  /* Cleanup edge queue. */
  BLI_heapsimple_free(eq_ctx.heap, nullptr);
  BLI_mempool_destroy(eq_ctx.pool);

  /* Remove temporary custom data layers. */
  BM_data_layer_free_named(bm, &bm->vdata, ".remesh_mask");
  BM_data_layer_free_named(bm, &bm->vdata, ".target_edge_length");

  /* Update mesh topology tables before conversion to ensure consistency. */
  BM_mesh_elem_table_ensure(bm, BM_VERT | BM_EDGE | BM_FACE);
  BM_mesh_normals_update(bm);

  /* Convert BMesh back to Mesh. */
  Mesh *result = BKE_mesh_from_bmesh_for_eval_nomain(bm, nullptr, &mesh);
  BM_mesh_free(bm);

  return result;
}

}  // namespace blender::geometry
