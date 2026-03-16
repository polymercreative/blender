/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup geo
 *
 * Mesh decimation functions wrapping BMesh decimate operations.
 */

#include "BLI_span.hh"

struct Mesh;

namespace blender::geometry {

/**
 * Mode for decimation operation.
 */
enum class DecimateMode {
  /** Edge collapse using quadric error metrics. */
  Collapse = 0,
  /** Reverse subdivision. */
  UnSubdivide = 1,
  /** Dissolve planar faces. */
  Planar = 2,
};

/**
 * Decimate a mesh using edge collapse (QEM-based).
 *
 * \param mesh: The input mesh (not modified).
 * \param ratio: Target ratio of faces to keep [0.0 - 1.0].
 * \param selection: Per-vertex selection. Empty span = all selected. false = protected.
 * \param use_vertex_group: Whether to use vertex weights.
 * \param vertex_weights: Optional per-vertex weights [0.0 - 1.0], size must match mesh verts.
 * \param vertex_group_factor: Influence of vertex weights.
 * \param use_symmetry: Enable symmetry-aware decimation.
 * \param symmetry_axis: Axis for symmetry (0=X, 1=Y, 2=Z), ignored if use_symmetry is false.
 * \param triangulate: Output triangulated mesh instead of trying to preserve quads.
 * \return A new decimated mesh, or nullptr on failure.
 */
Mesh *mesh_decimate_collapse(const Mesh &mesh,
                             float ratio,
                             Span<bool> selection,
                             bool use_vertex_group,
                             const float *vertex_weights,
                             float vertex_group_factor,
                             bool use_symmetry,
                             int symmetry_axis,
                             bool triangulate);

/**
 * Decimate a mesh by reversing subdivision.
 *
 * \param mesh: The input mesh (not modified).
 * \param iterations: Number of un-subdivide iterations.
 * \param selection: Per-vertex selection. Empty span = all selected. Only selected verts processed.
 * \return A new decimated mesh, or nullptr on failure.
 */
Mesh *mesh_decimate_unsubdivide(const Mesh &mesh, int iterations, Span<bool> selection);

/**
 * Decimate a mesh by dissolving planar faces.
 *
 * \param mesh: The input mesh (not modified).
 * \param angle_limit: Maximum angle between face normals to dissolve (radians).
 * \param dissolve_boundaries: Also dissolve vertices on mesh boundaries.
 * \param delimit: Delimit dissolve by UVs, normals, materials, seams, or sharp edges.
 * \param selection: Per-vertex selection. Empty span = all selected. Only selected verts/edges processed.
 * \return A new decimated mesh, or nullptr on failure.
 */
Mesh *mesh_decimate_planar(const Mesh &mesh,
                           float angle_limit,
                           bool dissolve_boundaries,
                           int delimit,
                           Span<bool> selection);

}  // namespace blender::geometry
