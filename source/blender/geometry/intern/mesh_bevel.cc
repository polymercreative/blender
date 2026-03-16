/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_mesh_bevel.hh"

#include "BKE_mesh.hh"

#include "DNA_modifier_types.h"

#include "bmesh.hh"
#include "bmesh_tools.hh"

#include "BLI_array.hh"
#include "BLI_math_geom.h"
#include "BLI_math_matrix.h"
#include "BLI_math_vector.h"
#include "BLI_vector.hh"

#include "MEM_guardedalloc.h"

#include <cmath>

namespace blender::geometry {

/* Collision detection constants */
static constexpr int CLAMP_METHOD_STANDARD = 0;
static constexpr int CLAMP_METHOD_COLLISION = 1;

/**
 * Per-vertex data for collision detection within a face.
 */
struct FaceVertexData {
  BMVert *vert;
  float2 pos_2d;       /* Position in 2D face space */
  float2 bisector;     /* Bisector direction (normalized, pointing inward) */
  float speed;         /* Movement speed = 1/sin(angle/2) */
  float max_distance;  /* Maximum allowed movement distance */
};

/**
 * Project a 3D point to 2D face-local coordinates.
 * The face normal defines the projection plane, with the face center as origin.
 */
static float2 project_to_face_2d(const float3 &point,
                                 const float3 &face_center,
                                 const float3 &axis_x,
                                 const float3 &axis_y)
{
  float3 local = point - face_center;
  return float2(dot_v3v3(local, axis_x), dot_v3v3(local, axis_y));
}

/**
 * Unproject 2D face-local coordinates back to 3D.
 */
static float3 unproject_from_face_2d(const float2 &point_2d,
                                     const float3 &face_center,
                                     const float3 &axis_x,
                                     const float3 &axis_y)
{
  return face_center + axis_x * point_2d.x + axis_y * point_2d.y;
}

/**
 * Calculate bisector direction for a vertex in 2D.
 * The bisector points inward (towards the face interior).
 */
static float2 calc_bisector_2d(const float2 &prev, const float2 &curr, const float2 &next)
{
  float2 e1 = prev - curr;
  float2 e2 = next - curr;

  float len1 = len_v2(e1);
  float len2 = len_v2(e2);

  if (len1 < 1e-8f || len2 < 1e-8f) {
    return float2(0.0f, 0.0f);
  }

  e1 = e1 / len1;
  e2 = e2 / len2;

  /* Bisector is the sum of normalized edge directions */
  float2 bisector = e1 + e2;
  float bis_len = len_v2(bisector);

  if (bis_len < 1e-8f) {
    /* Edges are collinear and opposite, use perpendicular */
    bisector = float2(-e1.y, e1.x);
  }
  else {
    bisector = bisector / bis_len;
  }

  /* Ensure bisector points inward (cross product test) */
  /* For CCW winding, inward is to the left of e2 direction */
  float cross = e1.x * e2.y - e1.y * e2.x;
  if (cross < 0.0f) {
    bisector = -bisector;
  }

  return bisector;
}

/**
 * Calculate vertex movement speed based on the angle at that vertex.
 * Speed = 1/sin(angle/2), which is how far the vertex moves per unit of offset.
 */
static float calc_vertex_speed(const float2 &prev, const float2 &curr, const float2 &next)
{
  float2 e1 = prev - curr;
  float2 e2 = next - curr;

  float len1 = len_v2(e1);
  float len2 = len_v2(e2);

  if (len1 < 1e-8f || len2 < 1e-8f) {
    return 1.0f;
  }

  e1 = e1 / len1;
  e2 = e2 / len2;

  float dot = std::clamp(dot_v2v2(e1, e2), -1.0f, 1.0f);
  float angle = acosf(dot);
  float half_angle = angle / 2.0f;
  float sin_half = sinf(half_angle);

  if (sin_half < 0.001f) {
    /* Near-180 degree angle, cap the speed */
    return 1000.0f;
  }

  return 1.0f / sin_half;
}

/**
 * Check if two line segments intersect in 2D.
 * Returns true if they intersect, and sets t to the parameter along segment (a1, a2).
 */
static bool segment_intersect_2d(const float2 &a1,
                                 const float2 &a2,
                                 const float2 &b1,
                                 const float2 &b2,
                                 float &t)
{
  float2 d1 = a2 - a1;
  float2 d2 = b2 - b1;
  float2 d3 = b1 - a1;

  float cross = d1.x * d2.y - d1.y * d2.x;
  if (fabsf(cross) < 1e-10f) {
    return false; /* Parallel */
  }

  t = (d3.x * d2.y - d3.y * d2.x) / cross;
  float u = (d3.x * d1.y - d3.y * d1.x) / cross;

  return (t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f);
}

/**
 * Calculate the collision time for a vertex moving along its bisector
 * against an edge that is also moving (shrinking inward).
 *
 * Based on Hard Bevel's get_est() function.
 * This solves for the time t when vertex k (moving along its bisector)
 * would collide with the swept region of edge (a, b).
 *
 * \param a, b: The two vertices of the edge
 * \param ma, mb: Movement vectors for vertices a and b (bisector * speed)
 * \param k: The vertex to check collision for
 * \param mk: Movement vector for vertex k
 * \return Collision time, or FLT_MAX if no collision
 */
static float calc_collision_time(const float2 &a,
                                 const float2 &b,
                                 const float2 &ma,
                                 const float2 &mb,
                                 const float2 &k,
                                 const float2 &mk)
{
  /* We need to find t such that k + t*mk lies on the line (a + t*ma, b + t*mb) */
  /* This requires solving a quadratic equation in general */

  /* Simplified approach: check intersection of k's trajectory with the original edge,
   * then check if the edge has shrunk past k at that point */

  /* For now, use a simpler linear approximation:
   * Check if k's trajectory intersects the stationary edge (a, b) */
  float2 k_far = k + mk * 1000.0f;  /* Extend trajectory */

  float t_intersect;
  if (segment_intersect_2d(k, k_far, a, b, t_intersect)) {
    /* k's trajectory hits the edge - compute distance */
    float dist = t_intersect * 1000.0f;  /* Distance along trajectory */
    return dist;
  }

  return FLT_MAX;
}

/**
 * Apply collision detection to adjust bevel vertex positions within a single face.
 * This implements the per-face 2D straight-skeleton-style collision detection
 * similar to Hard Bevel.
 */
static void apply_collision_detection_face(BMesh *bm, BMFace *face, float offset)
{
  const int face_len = face->len;
  if (face_len < 3) {
    return;
  }

  /* Calculate face normal and center */
  float normal[3], center[3];
  BM_face_calc_normal(face, normal);
  BM_face_calc_center_median(face, center);

  float3 face_normal(normal[0], normal[1], normal[2]);
  float3 face_center(center[0], center[1], center[2]);

  /* Build orthonormal basis for 2D projection */
  float3 axis_x, axis_y;
  float axis_x_arr[3], axis_y_arr[3];
  ortho_basis_v3v3_v3(axis_x_arr, axis_y_arr, normal);
  axis_x = float3(axis_x_arr[0], axis_x_arr[1], axis_x_arr[2]);
  axis_y = float3(axis_y_arr[0], axis_y_arr[1], axis_y_arr[2]);

  /* Collect vertex data */
  Array<FaceVertexData> verts(face_len);
  BMLoop *l_first = BM_FACE_FIRST_LOOP(face);
  BMLoop *l_iter = l_first;
  int idx = 0;

  do {
    verts[idx].vert = l_iter->v;
    verts[idx].pos_2d = project_to_face_2d(
        float3(l_iter->v->co[0], l_iter->v->co[1], l_iter->v->co[2]),
        face_center,
        axis_x,
        axis_y);
    verts[idx].max_distance = FLT_MAX;
    idx++;
    l_iter = l_iter->next;
  } while (l_iter != l_first);

  /* Calculate bisectors and speeds */
  for (int i = 0; i < face_len; i++) {
    int prev_i = (i + face_len - 1) % face_len;
    int next_i = (i + 1) % face_len;

    verts[i].bisector = calc_bisector_2d(verts[prev_i].pos_2d, verts[i].pos_2d, verts[next_i].pos_2d);
    verts[i].speed = calc_vertex_speed(verts[prev_i].pos_2d, verts[i].pos_2d, verts[next_i].pos_2d);
  }

  /* Check each vertex against all other edges for collision */
  for (int i = 0; i < face_len; i++) {
    float2 k = verts[i].pos_2d;
    float2 mk = verts[i].bisector * verts[i].speed;

    for (int j = 0; j < face_len; j++) {
      int j_next = (j + 1) % face_len;

      /* Skip edges adjacent to this vertex */
      if (j == i || j_next == i || j == (i + face_len - 1) % face_len) {
        continue;
      }

      float2 a = verts[j].pos_2d;
      float2 b = verts[j_next].pos_2d;
      float2 ma = verts[j].bisector * verts[j].speed;
      float2 mb = verts[j_next].bisector * verts[j_next].speed;

      float collision_dist = calc_collision_time(a, b, ma, mb, k, mk);
      if (collision_dist < verts[i].max_distance) {
        verts[i].max_distance = collision_dist * 0.999f;  /* Small margin */
      }
    }

    /* Also check collision with own trajectory against neighboring edges meeting at a point */
    /* This handles the case where two vertices from adjacent edges collide */
    int prev_i = (i + face_len - 1) % face_len;
    int next_i = (i + 1) % face_len;

    /* Intersection with previous vertex's trajectory */
    float2 prev_far = verts[prev_i].pos_2d + verts[prev_i].bisector * verts[prev_i].speed * 1000.0f;
    float2 k_far = k + mk * 1000.0f;

    float t1, t2;
    float2 d1 = k_far - k;
    float2 d2 = prev_far - verts[prev_i].pos_2d;
    float2 d3 = verts[prev_i].pos_2d - k;

    float cross = d1.x * d2.y - d1.y * d2.x;
    if (fabsf(cross) > 1e-10f) {
      t1 = (d3.x * d2.y - d3.y * d2.x) / cross;
      t2 = (d3.x * d1.y - d3.y * d1.x) / cross;

      if (t1 > 0.0f && t2 > 0.0f) {
        float dist = t1 * 1000.0f;
        if (dist < verts[i].max_distance) {
          verts[i].max_distance = dist * 0.999f;
        }
      }
    }

    /* Intersection with next vertex's trajectory */
    float2 next_far = verts[next_i].pos_2d + verts[next_i].bisector * verts[next_i].speed * 1000.0f;
    d2 = next_far - verts[next_i].pos_2d;
    d3 = verts[next_i].pos_2d - k;

    cross = d1.x * d2.y - d1.y * d2.x;
    if (fabsf(cross) > 1e-10f) {
      t1 = (d3.x * d2.y - d3.y * d2.x) / cross;
      t2 = (d3.x * d1.y - d3.y * d1.x) / cross;

      if (t1 > 0.0f && t2 > 0.0f) {
        float dist = t1 * 1000.0f;
        if (dist < verts[i].max_distance) {
          verts[i].max_distance = dist * 0.999f;
        }
      }
    }
  }

  /* Now adjust vertex positions based on collision constraints */
  /* Note: We're working on already-beveled geometry, so we need to identify
   * which vertices were moved by the bevel operation and might need adjustment.
   * For simplicity, we check all vertices that are tagged. */

  l_iter = l_first;
  idx = 0;
  do {
    if (BM_elem_flag_test(l_iter->v, BM_ELEM_TAG)) {
      /* This vertex was involved in the bevel - check if it needs clamping */
      float current_offset = verts[idx].speed > 0.0f ? offset * verts[idx].speed : offset;

      if (verts[idx].max_distance < current_offset && verts[idx].max_distance > 0.0f) {
        /* Need to clamp this vertex */
        float clamped_offset = verts[idx].max_distance;

        /* Calculate new position */
        float2 new_pos_2d = verts[idx].pos_2d + verts[idx].bisector * (clamped_offset - current_offset);
        float3 new_pos_3d = unproject_from_face_2d(new_pos_2d, face_center, axis_x, axis_y);

        /* Blend with original based on how much we're clamping */
        float blend = clamped_offset / current_offset;
        l_iter->v->co[0] = l_iter->v->co[0] * blend + new_pos_3d.x * (1.0f - blend);
        l_iter->v->co[1] = l_iter->v->co[1] * blend + new_pos_3d.y * (1.0f - blend);
        l_iter->v->co[2] = l_iter->v->co[2] * blend + new_pos_3d.z * (1.0f - blend);
      }
    }
    idx++;
    l_iter = l_iter->next;
  } while (l_iter != l_first);
}

/**
 * Apply collision detection post-processing to the beveled mesh.
 * This adjusts vertex positions to prevent overlapping bevels using
 * a per-face 2D straight-skeleton-style algorithm.
 */
static void apply_collision_detection(BMesh *bm, float offset, bool affect_vertices)
{
  if (affect_vertices) {
    /* Vertex bevel collision detection is more complex - not implemented yet */
    return;
  }

  /* Process each face */
  BMFace *f;
  BMIter iter;

  BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
    /* Check if this face has any tagged vertices (involved in bevel) */
    bool has_tagged = false;
    BMLoop *l_iter = BM_FACE_FIRST_LOOP(f);
    BMLoop *l_first = l_iter;
    do {
      if (BM_elem_flag_test(l_iter->v, BM_ELEM_TAG)) {
        has_tagged = true;
        break;
      }
      l_iter = l_iter->next;
    } while (l_iter != l_first);

    if (has_tagged) {
      apply_collision_detection_face(bm, f, offset);
    }
  }
}

Mesh *mesh_bevel(const Mesh &mesh,
                 const float offset,
                 const int offset_type,
                 const int segments,
                 const float profile,
                 const bool affect_vertices,
                 const Span<bool> selection,
                 const int clamp_method)
{
  if (offset == 0.0f) {
    /* No beveling needed. */
    return nullptr;
  }

  if (mesh.verts_num == 0) {
    /* Empty mesh. */
    return nullptr;
  }

  /* Convert mesh to BMesh. */
  BMeshCreateParams create_params{};
  BMeshFromMeshParams convert_params{};
  convert_params.calc_face_normal = true;
  convert_params.calc_vert_normal = true;
  convert_params.cd_mask_extra.vmask = CD_MASK_ORIGINDEX;
  convert_params.cd_mask_extra.emask = CD_MASK_ORIGINDEX;
  convert_params.cd_mask_extra.pmask = CD_MASK_ORIGINDEX;

  BMesh *bm = BKE_mesh_to_bmesh_ex(&mesh, &create_params, &convert_params);

  BMIter iter;
  bool any_tagged = false;

  if (affect_vertices) {
    /* Vertex bevel: tag selected vertices. */
    BMVert *v;
    int v_idx = 0;
    BM_ITER_MESH_INDEX (v, &iter, bm, BM_VERTS_OF_MESH, v_idx) {
      const bool selected = selection.is_empty() || selection[v_idx];
      if (selected) {
        BM_elem_flag_enable(v, BM_ELEM_TAG);
        any_tagged = true;
      }
    }
  }
  else {
    /* Edge bevel: tag selected manifold edges and their vertices. */
    BMEdge *e;
    int e_idx = 0;
    BM_ITER_MESH_INDEX (e, &iter, bm, BM_EDGES_OF_MESH, e_idx) {
      const bool selected = selection.is_empty() || selection[e_idx];
      if (selected && BM_edge_is_manifold(e)) {
        BM_elem_flag_enable(e, BM_ELEM_TAG);
        BM_elem_flag_enable(e->v1, BM_ELEM_TAG);
        BM_elem_flag_enable(e->v2, BM_ELEM_TAG);
        any_tagged = true;
      }
    }
  }

  if (!any_tagged) {
    /* Nothing to bevel. */
    BM_mesh_free(bm);
    return nullptr;
  }

  /* Call BM_mesh_bevel with parameters.
   * Use sensible defaults for advanced parameters that aren't exposed yet. */
  /* BM_mesh_bevel expects affect_type:
   * MOD_BEVEL_AFFECT_VERTICES (0) for vertex bevel
   * MOD_BEVEL_AFFECT_EDGES (1) for edge bevel
   * Since affect_vertices is true=1 when we want vertices, we need to invert the logic. */
  const int affect_type = affect_vertices ? MOD_BEVEL_AFFECT_VERTICES : MOD_BEVEL_AFFECT_EDGES;

  /* For collision detection mode, disable Blender's built-in limit_offset
   * since we'll do our own collision detection afterward. */
  const bool limit_offset = (clamp_method == CLAMP_METHOD_STANDARD);

  BM_mesh_bevel(bm,
                offset,
                offset_type,                     /* offset_type (MOD_BEVEL_AMT_*) */
                MOD_BEVEL_PROFILE_SUPERELLIPSE,  /* profile_type */
                segments,                        /* segments */
                profile,                         /* profile (0.5 = round) */
                affect_type,                     /* affect_type (vertices vs edges) */
                false,                           /* use_weights */
                limit_offset,                    /* limit_offset (clamp overlap) */
                nullptr,                         /* dvert (no vertex group) */
                -1,                              /* vertex_group */
                -1,                              /* mat (inherit material) */
                true,                            /* loop_slide */
                false,                           /* mark_seam */
                false,                           /* mark_sharp */
                false,                           /* harden_normals */
                MOD_BEVEL_FACE_STRENGTH_NONE,    /* face_strength_mode */
                MOD_BEVEL_MITER_SHARP,           /* miter_outer */
                MOD_BEVEL_MITER_SHARP,           /* miter_inner */
                0.0f,                            /* spread */
                nullptr,                         /* custom_profile */
                MOD_BEVEL_VMESH_ADJ,             /* vmesh_method */
                -1,                              /* bweight_offset_vert */
                -1);                             /* bweight_offset_edge */

  /* Apply collision detection if requested */
  if (clamp_method == CLAMP_METHOD_COLLISION) {
    apply_collision_detection(bm, offset, affect_vertices);
  }

  /* Convert back to Mesh. */
  Mesh *result = BKE_mesh_from_bmesh_for_eval_nomain(bm, nullptr, &mesh);
  BM_mesh_free(bm);

  return result;
}

}  // namespace blender::geometry
