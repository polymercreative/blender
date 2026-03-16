/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup geo
 *
 * Polygon Inset via Straight Skeleton
 *
 * TODO: Implement proper straight skeleton algorithm
 * This is currently a placeholder that returns the original mesh.
 */

#include "GEO_mesh_skeleton_inset.hh"

#include "BKE_mesh.hh"

#include "DNA_mesh_types.h"

namespace blender::geometry {

std::optional<SkeletonInsetResult> mesh_skeleton_inset(const Mesh &mesh,
                                                       Span<bool> selection,
                                                       float distance,
                                                       float depth,
                                                       SkeletonInsetMode mode)
{
  UNUSED_VARS(selection, distance, depth, mode);

  /* TODO: Implement straight skeleton inset algorithm
   *
   * Requirements:
   * - Handle split events when reflex vertices collide with opposite edges
   * - Handle edge events when adjacent bisectors meet
   * - Support multiple LAVs (List of Active Vertices) for proper topology changes
   * - Compute proper offset polygons at specified distance
   *
   * Current status: Returns original mesh unchanged to avoid crashes
   */

  if (mesh.faces_num == 0) {
    return std::nullopt;
  }

  /* For now, just return a copy of the original mesh */
  Mesh *result = BKE_mesh_copy_for_eval(mesh);

  SkeletonInsetResult output;
  output.mesh = result;
  /* Empty index arrays - no faces were modified */
  output.inner_face_indices = Vector<int>();
  output.outer_face_indices = Vector<int>();

  return output;
}

}  // namespace blender::geometry
