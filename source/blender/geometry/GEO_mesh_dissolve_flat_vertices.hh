/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup geo
 */

struct Mesh;

namespace blender::geometry {

/**
 * Dissolve vertices with low curvature or thin faces to simplify flat regions.
 *
 * Designed for tessellated displacement meshes and post-decimation cleanup.
 * Removes unnecessary vertices in flat areas and fixes thin/sliver triangles
 * created by decimation. Only dissolves vertices with 3-4 edges.
 *
 * Vertices are dissolved if EITHER condition is met:
 * - Low curvature (flat area), OR
 * - Part of a thin/sliver face (small angle)
 *
 * \param mesh: Input triangulated mesh
 * \param curvature_threshold: Dissolve vertices with curvature below this value (0.0-1.0)
 * \param min_angle_radians: Dissolve vertices in faces with angles below this (radians)
 * \return: Simplified mesh
 */
Mesh *dissolve_flat_vertices_mesh(const Mesh &mesh,
                                  float curvature_threshold,
                                  float min_angle_radians);

}  // namespace blender::geometry
