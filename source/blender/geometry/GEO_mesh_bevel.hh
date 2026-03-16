/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup geo
 *
 * Mesh bevel functions wrapping BMesh bevel operations.
 */

#include "BLI_span.hh"

struct Mesh;

namespace blender::geometry {

/**
 * Bevel edges or vertices of a mesh.
 *
 * \param mesh: The input mesh (not modified).
 * \param offset: Bevel offset amount.
 * \param offset_type: How to interpret offset (0=OFFSET, 1=WIDTH, 2=DEPTH, 3=PERCENT, 4=ABSOLUTE).
 * \param segments: Number of segments in the bevel (1+).
 * \param profile: Profile shape factor (0.0-1.0, 0.5 = round).
 * \param affect_vertices: True to bevel vertices, false to bevel edges.
 * \param selection: Per-element selection. For vertex mode: vertex selection.
 *                   For edge mode: edge selection. Empty = all selected.
 * \param clamp_method: 0=Standard (Blender's limit_offset), 1=Collision detection.
 * \return A new beveled mesh, or nullptr if no beveling occurred.
 */
Mesh *mesh_bevel(const Mesh &mesh,
                 float offset,
                 int offset_type,
                 int segments,
                 float profile,
                 bool affect_vertices,
                 Span<bool> selection,
                 int clamp_method);

}  // namespace blender::geometry
