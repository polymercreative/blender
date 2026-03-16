/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup geo
 */

#include "BLI_array.hh"

struct Mesh;

namespace blender::geometry {

/**
 * Select vertices/faces based on triangle quality metrics.
 *
 * Identifies "bad" triangles using configurable criteria (min/max angle, min/max area)
 * and outputs boolean selection + weighted falloff for procedural cleanup workflows.
 *
 * \param mesh: Input mesh
 * \param min_angle_radians: Select faces with angles below this (0 = disabled)
 * \param max_angle_radians: Select faces with angles above this (PI = disabled)
 * \param min_area: Select faces with area below this (0 = disabled)
 * \param max_area: Select faces with area above this (FLT_MAX = disabled)
 * \param falloff_distance: Extend selection outward by this distance (0 = no falloff)
 * \param r_vertex_selection: Output boolean selection per vertex
 * \param r_vertex_weights: Output falloff weights per vertex (1.0 = selected, 0.0 = unselected)
 */
void select_bad_triangles(const Mesh &mesh,
                          float min_angle_radians,
                          float max_angle_radians,
                          float min_area,
                          float max_area,
                          float falloff_distance,
                          Array<bool> &r_vertex_selection,
                          Array<float> &r_vertex_weights);

}  // namespace blender::geometry
