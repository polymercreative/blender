/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup geo
 *
 * Dissolve low-curvature vertices to simplify tessellated displacement meshes.
 */

#include "GEO_mesh_dissolve_flat_vertices.hh"

#include "BKE_mesh.hh"

#include "BLI_math_base.h"
#include "BLI_math_vector.h"

#include "bmesh.hh"
#include "bmesh_tools.hh"

namespace blender::geometry {

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

/* Calculate the minimum angle in a face (in radians).
 * Small angles indicate thin/sliver triangles. */
static float calculate_face_min_angle(BMFace *f)
{
  if (f->len < 3) {
    return M_PI; /* Degenerate face */
  }

  float min_angle = M_PI;

  /* For each vertex in the face, calculate the angle at that corner. */
  BMLoop *l_first = BM_FACE_FIRST_LOOP(f);
  BMLoop *l_iter = l_first;
  do {
    /* Get the three vertices: previous, current, next */
    BMVert *v_prev = l_iter->prev->v;
    BMVert *v_curr = l_iter->v;
    BMVert *v_next = l_iter->next->v;

    /* Calculate vectors from current vertex to neighbors. */
    float vec1[3], vec2[3];
    sub_v3_v3v3(vec1, v_prev->co, v_curr->co);
    sub_v3_v3v3(vec2, v_next->co, v_curr->co);

    normalize_v3(vec1);
    normalize_v3(vec2);

    /* Calculate angle using dot product. */
    const float dot = dot_v3v3(vec1, vec2);
    const float angle = acosf(clamp_f(dot, -1.0f, 1.0f));

    if (angle < min_angle) {
      min_angle = angle;
    }
  } while ((l_iter = l_iter->next) != l_first);

  return min_angle;
}

/* Check if vertex is part of any thin/sliver faces.
 * Returns true if ANY adjacent face has an angle below the threshold. */
static bool has_thin_faces(BMVert *v, float min_angle_threshold)
{
  BMIter iter;
  BMFace *f;
  BM_ITER_ELEM (f, &iter, v, BM_FACES_OF_VERT) {
    const float min_angle = calculate_face_min_angle(f);
    if (min_angle < min_angle_threshold) {
      return true;
    }
  }
  return false;
}

/* Dissolve low-curvature vertices and vertices in thin faces to simplify flat regions. */
static int dissolve_low_curvature_verts(BMesh *bm,
                                        float curvature_threshold,
                                        float min_angle_threshold)
{
  int dissolved_count = 0;

  /* Ensure face normals are calculated. */
  BM_mesh_normals_update(bm);

  /* Tag all vertices initially. */
  BMIter iter;
  BMVert *v;
  BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
    BM_elem_flag_enable(v, BM_ELEM_TAG);
  }

  /* Build list of vertices to check with their initial edge counts.
   * We store the edge count at evaluation time to prevent cascading dissolution -
   * if we check edge count later, dissolving one vertex can change neighbor edge counts
   * and cause unwanted cascading that creates ngons. */
  struct VertexToCheck {
    BMVert *v;
    int initial_edge_count;
    float curvature;
    bool has_thin_face;
  };

  Vector<VertexToCheck> verts_to_check;
  BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
    /* Skip boundary vertices. */
    if (BM_vert_is_boundary(v)) {
      continue;
    }

    const int edge_count = BM_vert_edge_count(v);
    /* Only consider vertices with 3 or 4 edges. */
    if (edge_count == 3 || edge_count == 4) {
      const float curvature = calculate_vertex_curvature(v);
      const bool thin_face = has_thin_faces(v, min_angle_threshold);
      verts_to_check.append({v, edge_count, curvature, thin_face});
    }
  }

  for (const VertexToCheck &vtc : verts_to_check) {
    BMVert *v = vtc.v;

    /* Skip if vertex was already deleted. */
    if (!BM_elem_flag_test(v, BM_ELEM_TAG)) {
      continue;
    }

    /* Dissolve if EITHER:
     * 1. Low curvature (flat area), OR
     * 2. Part of thin/sliver face (cleanup decimation artifacts)
     * AND edge count hasn't changed (prevents cascading dissolution). */
    const bool should_dissolve = (vtc.curvature < curvature_threshold || vtc.has_thin_face) &&
                                 BM_vert_edge_count(v) == vtc.initial_edge_count;

    if (should_dissolve) {
      /* Dissolve vertex and surrounding faces. */
      if (BM_vert_dissolve(bm, v)) {
        dissolved_count++;
      }
    }
  }

  return dissolved_count;
}

Mesh *dissolve_flat_vertices_mesh(const Mesh &mesh,
                                  float curvature_threshold,
                                  float min_angle_radians)
{
  /* Convert to BMesh for topology operations. */
  BMeshCreateParams create_params{};
  BMesh *bm = BM_mesh_create(&bm_mesh_allocsize_default, &create_params);

  BMeshFromMeshParams from_mesh_params{};
  from_mesh_params.calc_face_normal = true;
  from_mesh_params.calc_vert_normal = true;
  BM_mesh_bm_from_me(bm, &mesh, &from_mesh_params);

  /* Dissolve low-curvature vertices and vertices in thin faces. */
  dissolve_low_curvature_verts(bm, curvature_threshold, min_angle_radians);

  /* Update mesh topology tables before conversion to ensure consistency. */
  BM_mesh_elem_table_ensure(bm, BM_VERT | BM_EDGE | BM_FACE);
  BM_mesh_normals_update(bm);

  /* Convert back to Mesh. */
  Mesh *result = BKE_mesh_from_bmesh_for_eval_nomain(bm, nullptr, &mesh);
  BM_mesh_free(bm);

  return result;
}

}  // namespace blender::geometry
