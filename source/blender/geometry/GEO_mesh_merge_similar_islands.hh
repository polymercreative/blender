/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_string_ref.hh"

struct Mesh;

namespace blender::geometry {

/** Mode for determining final attribute value when merging islands. */
enum class MergeSimilarMode {
  /** Use value from the largest island in the merged group. */
  Largest = 0,
  /** Average all values in the merged group. */
  Average = 1,
};

/** Mode for calculating difference between vector attributes. */
enum class VectorDifferenceMode {
  /** Euclidean distance: length(a - b). Good for positions/colors. */
  Distance = 0,
  /** Angular difference: 1 - dot(normalize(a), normalize(b)). Good for normals/directions. */
  Angular = 1,
};

/**
 * Merge attribute islands that have similar values.
 *
 * Islands are groups of contiguous faces with the same attribute value.
 * Adjacent islands whose attribute values differ by less than the threshold
 * are merged together, with the final value determined by the merge mode.
 *
 * \param mesh: The input mesh (not modified).
 * \param attribute_name: Name of the face attribute to analyze and modify.
 * \param threshold: Maximum difference between attribute values to allow merging.
 * \param merge_mode: How to determine the final value for merged islands.
 * \param vector_mode: How to calculate difference for vector types.
 * \param iterations: Maximum number of merge passes (0 = unlimited).
 * \return A new mesh with the modified attribute, or nullptr on failure.
 */
Mesh *mesh_merge_similar_attribute_islands(const Mesh &mesh,
                                           StringRef attribute_name,
                                           float threshold,
                                           MergeSimilarMode merge_mode,
                                           VectorDifferenceMode vector_mode,
                                           int iterations = 0);

}  // namespace blender::geometry
