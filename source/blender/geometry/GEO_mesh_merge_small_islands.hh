/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_string_ref.hh"

struct Mesh;

/** \file
 * \ingroup geo
 */

namespace blender::geometry {

/**
 * Merge small attribute islands into neighboring large islands.
 *
 * Finds contiguous regions (islands) where faces share the same attribute value,
 * measures their size, and propagates values from large islands onto neighboring
 * small islands that are below the threshold.
 *
 * \param mesh: The input mesh to process.
 * \param attribute_name: Name of the face attribute to analyze and propagate.
 * \param threshold: Minimum face count for an island to be considered "large".
 *                   Small islands (< threshold) will inherit from neighbors.
 *
 * \returns The modified mesh with small islands merged into large neighbors,
 * or nullptr if the attribute doesn't exist or operation failed.
 */
Mesh *mesh_merge_small_attribute_islands(const Mesh &mesh,
                                         StringRef attribute_name,
                                         int threshold);

}  // namespace blender::geometry
