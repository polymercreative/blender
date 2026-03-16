/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <optional>

struct Mesh;

/** \file
 * \ingroup geo
 */

namespace blender::geometry {

/**
 * Fix 3-edge poles (vertices with exactly 3 edges) by collapsing pairs of them
 * that share a face, then deleting remaining faces with only 3-edge vertices.
 * Applies smoothing to the result.
 *
 * \param mesh: The input mesh to process.
 * \param smooth_iterations: Number of smoothing iterations to apply (default 4).
 *
 * \returns #std::nullopt if the mesh has no 3-edge poles to fix, in order to
 * avoid copying the input. Otherwise returns the new mesh with fixed topology.
 */
std::optional<Mesh *> mesh_fix_poles(const Mesh &mesh, int smooth_iterations = 4);

}  // namespace blender::geometry
