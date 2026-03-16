/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <optional>

#include "BLI_span.hh"
#include "BLI_vector.hh"

struct Mesh;

/** \file
 * \ingroup geo
 */

namespace blender::geometry {

/**
 * Mode for skeleton inset operation.
 */
enum class SkeletonInsetMode {
  Region = 0,      /* Connected region boundaries (primary use case) */
  Individual = 1,  /* Each face independently */
};

/**
 * Result of skeleton inset operation.
 */
struct SkeletonInsetResult {
  Mesh *mesh;
  Vector<int> inner_face_indices;  /* Inset face(s) - may be multiple after splits */
  Vector<int> outer_face_indices;  /* Rim faces connecting original to inset */
};

/**
 * Perform skeleton inset using CGAL-style straight skeleton algorithm.
 *
 * This uses event-driven wavefront propagation with proper topology changes:
 * - Edge events: When edges collapse, vertices merge at skeleton vertices
 * - Split events: When reflex vertices hit opposite edges, polygons split
 * - Holes: Inner contours participate and merge with outer via split events
 *
 * Handles any polygon shape including concave and those with holes.
 * Properly merges vertices at skeleton events and splits polygons as needed.
 *
 * \param mesh: The input mesh to process.
 * \param selection: Face selection mask (true = inset this face).
 * \param distance: Inset distance (wavefront propagation time).
 * \param depth: Perpendicular extrusion depth.
 * \param mode: Region (process connected regions) or Individual (each face separately).
 *
 * \returns #std::nullopt if no faces selected or mesh has no faces.
 * Otherwise returns result with new mesh and face index arrays.
 */
std::optional<SkeletonInsetResult> mesh_skeleton_inset(const Mesh &mesh,
                                                       Span<bool> selection,
                                                       float distance,
                                                       float depth,
                                                       SkeletonInsetMode mode);

}  // namespace blender::geometry
