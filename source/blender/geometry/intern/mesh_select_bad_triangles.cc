/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup geo
 *
 * Select vertices/faces based on triangle quality metrics for cleanup workflows.
 */

#include "GEO_mesh_select_bad_triangles.hh"

#include "BKE_mesh.hh"

#include "BLI_array.hh"
#include "BLI_math_base.h"
#include "BLI_math_vector.h"
#include "BLI_set.hh"
#include "BLI_task.hh"

namespace blender::geometry {

/* Calculate the minimum angle in a triangle (in radians). */
static float calculate_triangle_min_angle(const float3 &v0, const float3 &v1, const float3 &v2)
{
  /* Calculate vectors for each edge. */
  float3 edge01 = v1 - v0;
  float3 edge12 = v2 - v1;
  float3 edge20 = v0 - v2;

  const float len01 = len_v3(edge01);
  const float len12 = len_v3(edge12);
  const float len20 = len_v3(edge20);

  if (len01 == 0.0f || len12 == 0.0f || len20 == 0.0f) {
    return 0.0f; /* Degenerate */
  }

  /* Calculate angles at each vertex using dot product. */
  const float angle0 = acosf(clamp_f(-dot_v3v3(edge01, edge20) / (len01 * len20), -1.0f, 1.0f));
  const float angle1 = acosf(clamp_f(-dot_v3v3(edge12, edge01) / (len12 * len01), -1.0f, 1.0f));
  const float angle2 = acosf(clamp_f(-dot_v3v3(edge20, edge12) / (len20 * len12), -1.0f, 1.0f));

  return min_fff(angle0, angle1, angle2);
}

/* Calculate the maximum angle in a triangle (in radians). */
static float calculate_triangle_max_angle(const float3 &v0, const float3 &v1, const float3 &v2)
{
  /* Calculate vectors for each edge. */
  float3 edge01 = v1 - v0;
  float3 edge12 = v2 - v1;
  float3 edge20 = v0 - v2;

  const float len01 = len_v3(edge01);
  const float len12 = len_v3(edge12);
  const float len20 = len_v3(edge20);

  if (len01 == 0.0f || len12 == 0.0f || len20 == 0.0f) {
    return M_PI; /* Degenerate */
  }

  /* Calculate angles at each vertex using dot product. */
  const float angle0 = acosf(clamp_f(-dot_v3v3(edge01, edge20) / (len01 * len20), -1.0f, 1.0f));
  const float angle1 = acosf(clamp_f(-dot_v3v3(edge12, edge01) / (len12 * len01), -1.0f, 1.0f));
  const float angle2 = acosf(clamp_f(-dot_v3v3(edge20, edge12) / (len20 * len12), -1.0f, 1.0f));

  return max_fff(angle0, angle1, angle2);
}

/* Calculate triangle area using cross product. */
static float calculate_triangle_area(const float3 &v0, const float3 &v1, const float3 &v2)
{
  float3 edge01 = v1 - v0;
  float3 edge02 = v2 - v0;
  float3 cross_product;
  cross_v3_v3v3(cross_product, edge01, edge02);
  return len_v3(cross_product) * 0.5f;
}

void select_bad_triangles(const Mesh &mesh,
                          const float min_angle_radians,
                          const float max_angle_radians,
                          const float min_area,
                          const float max_area,
                          const float falloff_distance,
                          Array<bool> &r_vertex_selection,
                          Array<float> &r_vertex_weights)
{
  const Span<float3> positions = mesh.vert_positions();
  const OffsetIndices faces = mesh.faces();
  const Span<int> corner_verts = mesh.corner_verts();

  /* Initialize output arrays. */
  r_vertex_selection.reinitialize(mesh.verts_num);
  r_vertex_weights.reinitialize(mesh.verts_num);
  r_vertex_selection.fill(false);
  r_vertex_weights.fill(0.0f);

  /* Track which vertices are part of bad triangles. */
  Set<int> bad_verts;

  /* Check each face. */
  for (const int face_i : faces.index_range()) {
    const IndexRange face = faces[face_i];

    /* Only process triangles. */
    if (face.size() != 3) {
      continue;
    }

    const int v0_i = corner_verts[face[0]];
    const int v1_i = corner_verts[face[1]];
    const int v2_i = corner_verts[face[2]];

    const float3 &v0 = positions[v0_i];
    const float3 &v1 = positions[v1_i];
    const float3 &v2 = positions[v2_i];

    bool is_bad = false;

    /* Check minimum angle. */
    if (min_angle_radians > 0.0f) {
      const float min_angle = calculate_triangle_min_angle(v0, v1, v2);
      if (min_angle < min_angle_radians) {
        is_bad = true;
      }
    }

    /* Check maximum angle. */
    if (!is_bad && max_angle_radians < M_PI) {
      const float max_angle = calculate_triangle_max_angle(v0, v1, v2);
      if (max_angle > max_angle_radians) {
        is_bad = true;
      }
    }

    /* Check area. */
    if (!is_bad && (min_area > 0.0f || max_area < FLT_MAX)) {
      const float area = calculate_triangle_area(v0, v1, v2);
      if (area < min_area || area > max_area) {
        is_bad = true;
      }
    }

    /* Mark vertices of bad triangles. */
    if (is_bad) {
      bad_verts.add(v0_i);
      bad_verts.add(v1_i);
      bad_verts.add(v2_i);
    }
  }

  /* Set initial selection. */
  for (const int vert_i : bad_verts) {
    r_vertex_selection[vert_i] = true;
    r_vertex_weights[vert_i] = 1.0f;
  }

  /* Falloff disabled for testing. */
  UNUSED_VARS(falloff_distance);
}

}  // namespace blender::geometry
