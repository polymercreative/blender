/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup geo
 */

#include "BLI_array.hh"
#include "BLI_virtual_array.hh"

struct Mesh;

namespace blender::geometry {

/**
 * Adaptive remeshing based on dyntopo algorithm from sculpt mode.
 *
 * Performs iterative edge splits and collapses to achieve target edge lengths,
 * with support for attribute-based masking and per-vertex resolution control.
 *
 * \param mesh: Input mesh (will be triangulated internally)
 * \param remesh_mask: Per-vertex mask (0.0 = skip, 1.0 = remesh)
 * \param target_edge_length: Per-vertex target edge length
 * \param iterations: Number of refinement passes (default: 3-5)
 * \param preserve_boundaries: Keep boundary edges between masked/unmasked regions fixed
 * \return: Remeshed mesh (triangulated)
 */
Mesh *adaptive_remesh_mesh(const Mesh &mesh,
                           const VArray<float> &remesh_mask,
                           const VArray<float> &target_edge_length,
                           int iterations,
                           bool preserve_boundaries);

}  // namespace blender::geometry
