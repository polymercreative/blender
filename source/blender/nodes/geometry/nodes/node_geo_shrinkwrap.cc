/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "MEM_guardedalloc.h"

#include "GEO_foreach_geometry.hh"

#include "BKE_bvhutils.hh"
#include "BKE_mesh.hh"
#include "BKE_shrinkwrap.hh"

#include "BLI_math_vector.hh"
#include "BLI_task.hh"

#include "DNA_modifier_enums.h"
#include "DNA_node_types.h"

#include "NOD_rna_define.hh"

#include "RNA_access.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_shrinkwrap_cc {

NODE_STORAGE_FUNCS(NodeGeometryShrinkwrap)

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();

  b.add_input<decl::Geometry>("Mesh")
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Mesh to shrinkwrap");

  b.add_input<decl::Geometry>("Target")
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Target mesh to wrap onto");

  b.add_input<decl::Float>("Offset")
      .default_value(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Distance to keep from target surface");

  if (node != nullptr) {
    const NodeGeometryShrinkwrap &storage = node_storage(*node);
    if (storage.wrap_method == MOD_SHRINKWRAP_PROJECT) {
      b.add_input<decl::Float>("Project Limit")
          .default_value(0.0f)
          .min(0.0f)
          .subtype(PROP_DISTANCE)
          .description("Limit the distance for projection (0 = unlimited)");

      b.add_input<decl::Int>("Subdivision Levels")
          .default_value(0)
          .min(0)
          .max(6)
          .description("Subdivision levels for more accurate vertex normals");
    }
  }

  b.add_output<decl::Geometry>("Mesh").propagate_all();
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout->prop(ptr, "wrap_method", UI_ITEM_NONE, "", ICON_NONE);

  const int wrap_method = RNA_enum_get(ptr, "wrap_method");
  if (ELEM(wrap_method,
           MOD_SHRINKWRAP_PROJECT,
           MOD_SHRINKWRAP_NEAREST_SURFACE,
           MOD_SHRINKWRAP_TARGET_PROJECT))
  {
    layout->prop(ptr, "wrap_mode", UI_ITEM_NONE, "", ICON_NONE);
  }

  if (wrap_method == MOD_SHRINKWRAP_PROJECT) {
    layout->prop(ptr, "proj_axis", UI_ITEM_R_EXPAND, "Axis", ICON_NONE);
    layout->prop(ptr, "cull_face", UI_ITEM_R_EXPAND, std::nullopt, ICON_NONE);
  }
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryShrinkwrap *data = MEM_callocN<NodeGeometryShrinkwrap>(__func__);
  data->wrap_method = MOD_SHRINKWRAP_NEAREST_SURFACE;
  data->wrap_mode = MOD_SHRINKWRAP_ON_SURFACE;
  data->proj_axis = MOD_SHRINKWRAP_PROJECT_OVER_NORMAL;
  data->cull_face = 0;
  node->storage = data;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh");
  GeometrySet target_set = params.extract_input<GeometrySet>("Target");
  const NodeGeometryShrinkwrap &storage = node_storage(params.node());

  const float offset = params.extract_input<float>("Offset");

  /* Check if we have a target mesh. */
  if (!target_set.has_mesh()) {
    params.set_output("Mesh", std::move(geometry_set));
    return;
  }

  const Mesh *target_mesh = target_set.get_mesh();
  if (target_mesh->verts_num == 0) {
    params.set_output("Mesh", std::move(geometry_set));
    return;
  }

  /* Build appropriate BVH tree based on wrap method. */
  bke::BVHTreeFromMesh tree_data;

  if (storage.wrap_method == MOD_SHRINKWRAP_NEAREST_VERTEX) {
    /* For nearest vertex mode, use vertex BVH. */
    tree_data = target_mesh->bvh_verts();
  }
  else {
    /* For surface/project modes, use corner triangles BVH. */
    tree_data = target_mesh->bvh_corner_tris();

    /* Verify target mesh has faces. */
    if (target_mesh->faces_num == 0) {
      params.error_message_add(NodeWarningType::Error,
                                "Target mesh must have faces for surface shrinkwrap");
      params.set_output("Mesh", std::move(geometry_set));
      return;
    }
  }

  if (tree_data.tree == nullptr) {
    params.error_message_add(NodeWarningType::Error,
                              "Failed to build BVH tree from target mesh");
    params.set_output("Mesh", std::move(geometry_set));
    return;
  }

  /* Process each mesh in the geometry set. */
  geometry::foreach_real_geometry(geometry_set, [&](GeometrySet &geometry_set) {
    if (const Mesh *source_mesh = geometry_set.get_mesh()) {
      if (source_mesh->verts_num == 0) {
        return;
      }

      /* Create a mutable copy of the source mesh. */
      Mesh *result_mesh = BKE_mesh_copy_for_eval(*source_mesh);

      /* Get vertex positions as mutable span. */
      MutableSpan<float3> positions = result_mesh->vert_positions_for_write();

      /* Process vertices based on wrap method and mode. */
      const int wrap_method = storage.wrap_method;
      const int wrap_mode = storage.wrap_mode;

      threading::parallel_for(positions.index_range(), 1024, [&](const IndexRange range) {
        for (const int i : range) {
          const float3 original_pos = positions[i];

          if (wrap_method == MOD_SHRINKWRAP_NEAREST_SURFACE ||
              wrap_method == MOD_SHRINKWRAP_NEAREST_VERTEX)
          {
            /* Find nearest point (surface or vertex). */
            BVHTreeNearest nearest;
            nearest.index = -1;
            nearest.dist_sq = FLT_MAX;

            BLI_bvhtree_find_nearest(
                tree_data.tree, original_pos, &nearest, tree_data.nearest_callback, &tree_data);

            if (nearest.index != -1) {
              const float3 hit_co = float3(nearest.co);
              const float3 hit_no = float3(nearest.no);

              /* Apply snap mode with offset. Mirrors BKE_shrinkwrap_snap_point_to_surface(). */
              switch (wrap_mode) {
                case MOD_SHRINKWRAP_ON_SURFACE:
                  if (offset == 0.0f) {
                    positions[i] = hit_co;
                  }
                  else {
                    /* Offset along line from original to hit. */
                    const float3 dir = math::normalize(hit_co - original_pos);
                    positions[i] = hit_co + dir * offset;
                  }
                  break;

                case MOD_SHRINKWRAP_INSIDE: {
                  /* Always move inside, offset distance from surface. */
                  const float3 dir = math::normalize(original_pos - hit_co);
                  const float dist = math::distance(original_pos, hit_co);
                  if (dist > offset) {
                    positions[i] = hit_co + dir * offset;
                  }
                  break;
                }

                case MOD_SHRINKWRAP_OUTSIDE: {
                  /* Always move outside, offset distance from surface. */
                  const float3 dir = math::normalize(original_pos - hit_co);
                  positions[i] = hit_co + dir * offset;
                  break;
                }

                case MOD_SHRINKWRAP_OUTSIDE_SURFACE:
                  if (offset == 0.0f) {
                    positions[i] = hit_co;
                  }
                  else {
                    /* Offset along line from original, but only if outside. */
                    const float3 dir = math::normalize(hit_co - original_pos);
                    if (math::dot(dir, hit_no) < 0.0f) {
                      positions[i] = hit_co - dir * offset;
                    }
                    else {
                      positions[i] = hit_co + dir * offset;
                    }
                  }
                  break;

                case MOD_SHRINKWRAP_ABOVE_SURFACE:
                  /* Offset along surface normal. */
                  positions[i] = hit_co + hit_no * offset;
                  break;

                default:
                  positions[i] = hit_co;
                  break;
              }
            }
          }
          else if (wrap_method == MOD_SHRINKWRAP_PROJECT) {
            /* Project mode: cast ray from vertex along projection axis. */
            const float3 ray_origin = original_pos;
            float3 ray_direction;

            /* Determine projection direction. */
            if (storage.proj_axis == MOD_SHRINKWRAP_PROJECT_OVER_X_AXIS) {
              ray_direction = float3(1.0f, 0.0f, 0.0f);
            }
            else if (storage.proj_axis == MOD_SHRINKWRAP_PROJECT_OVER_Y_AXIS) {
              ray_direction = float3(0.0f, 1.0f, 0.0f);
            }
            else if (storage.proj_axis == MOD_SHRINKWRAP_PROJECT_OVER_Z_AXIS) {
              ray_direction = float3(0.0f, 0.0f, 1.0f);
            }
            else {
              /* PROJECT_OVER_NORMAL: would need vertex normals, skip for now. */
              continue;
            }

            BVHTreeRayHit hit;
            hit.index = -1;
            hit.dist = FLT_MAX;

            /* Cast ray in both directions if enabled. */
            bool hit_found = false;

            /* Positive direction. */
            if (BLI_bvhtree_ray_cast(tree_data.tree,
                                     ray_origin,
                                     ray_direction,
                                     0.0f,
                                     &hit,
                                     tree_data.raycast_callback,
                                     &tree_data) != -1)
            {
              hit_found = true;
            }

            /* Negative direction. */
            BVHTreeRayHit hit_neg;
            hit_neg.index = -1;
            hit_neg.dist = FLT_MAX;

            if (BLI_bvhtree_ray_cast(tree_data.tree,
                                     ray_origin,
                                     -ray_direction,
                                     0.0f,
                                     &hit_neg,
                                     tree_data.raycast_callback,
                                     &tree_data) != -1)
            {
              /* Use closest hit. */
              if (!hit_found || hit_neg.dist < hit.dist) {
                hit = hit_neg;
                hit_found = true;
              }
            }

            if (hit_found) {
              const float3 hit_co = float3(hit.co);
              const float3 hit_no = float3(hit.no);

              /* Apply offset. */
              if (offset == 0.0f) {
                positions[i] = hit_co;
              }
              else {
                positions[i] = hit_co + hit_no * offset;
              }
            }
          }
        }
      });

      geometry_set.replace_mesh(result_mesh);
    }
  });

  params.set_output("Mesh", std::move(geometry_set));
}

static void node_rna(StructRNA *srna)
{
  static const EnumPropertyItem wrap_method_items[] = {
      {MOD_SHRINKWRAP_NEAREST_SURFACE,
       "NEAREST_SURFACE",
       0,
       "Nearest Surface Point",
       "Shrink to the nearest point on the target surface"},
      {MOD_SHRINKWRAP_PROJECT,
       "PROJECT",
       0,
       "Project",
       "Project vertices along a specified axis onto the target"},
      {MOD_SHRINKWRAP_NEAREST_VERTEX,
       "NEAREST_VERTEX",
       0,
       "Nearest Vertex",
       "Shrink to the nearest vertex on the target mesh"},
      {MOD_SHRINKWRAP_TARGET_PROJECT,
       "TARGET_PROJECT",
       0,
       "Target Normal Project",
       "Project vertices along target's normal onto the target"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static const EnumPropertyItem wrap_mode_items[] = {
      {MOD_SHRINKWRAP_ON_SURFACE,
       "ON_SURFACE",
       0,
       "On Surface",
       "Vertices are constrained to the target surface"},
      {MOD_SHRINKWRAP_INSIDE,
       "INSIDE",
       0,
       "Inside",
       "Vertices are constrained to be inside the target mesh"},
      {MOD_SHRINKWRAP_OUTSIDE,
       "OUTSIDE",
       0,
       "Outside",
       "Vertices are constrained to be outside the target mesh"},
      {MOD_SHRINKWRAP_OUTSIDE_SURFACE,
       "OUTSIDE_SURFACE",
       0,
       "Outside Surface",
       "Vertices are constrained to be outside the target surface"},
      {MOD_SHRINKWRAP_ABOVE_SURFACE,
       "ABOVE_SURFACE",
       0,
       "Above Surface",
       "Vertices are constrained to be above the target surface"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static const EnumPropertyItem proj_axis_items[] = {
      {MOD_SHRINKWRAP_PROJECT_OVER_NORMAL,
       "PROJECT_OVER_NORMAL",
       0,
       "Vertex Normal",
       "Project along vertex normal"},
      {MOD_SHRINKWRAP_PROJECT_OVER_X_AXIS, "X", 0, "X", "Project along X axis"},
      {MOD_SHRINKWRAP_PROJECT_OVER_Y_AXIS, "Y", 0, "Y", "Project along Y axis"},
      {MOD_SHRINKWRAP_PROJECT_OVER_Z_AXIS, "Z", 0, "Z", "Project along Z axis"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static const EnumPropertyItem cull_face_items[] = {
      {0, "OFF", 0, "Off", "No face culling"},
      {1, "FRONT", 0, "Front", "Cull front faces"},
      {2, "BACK", 0, "Back", "Cull back faces"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "wrap_method",
                    "Wrap Method",
                    "How to constrain vertices to the target mesh",
                    wrap_method_items,
                    NOD_storage_enum_accessors(wrap_method),
                    MOD_SHRINKWRAP_NEAREST_SURFACE);

  RNA_def_node_enum(srna,
                    "wrap_mode",
                    "Snap Mode",
                    "How to constrain vertices to the target surface",
                    wrap_mode_items,
                    NOD_storage_enum_accessors(wrap_mode),
                    MOD_SHRINKWRAP_ON_SURFACE);

  RNA_def_node_enum(srna,
                    "proj_axis",
                    "Projection Axis",
                    "Axis along which to project vertices",
                    proj_axis_items,
                    NOD_storage_enum_accessors(proj_axis),
                    MOD_SHRINKWRAP_PROJECT_OVER_NORMAL);

  RNA_def_node_enum(srna,
                    "cull_face",
                    "Face Cull",
                    "Which faces to ignore during projection",
                    cull_face_items,
                    NOD_storage_enum_accessors(cull_face),
                    0);
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeShrinkwrap", GEO_NODE_SHRINKWRAP);
  ntype.ui_name = "Shrinkwrap";
  ntype.ui_description =
      "Wrap a mesh around a target mesh by moving vertices to the nearest point on the target "
      "surface, projecting along an axis, or snapping to vertices";
  ntype.enum_name_legacy = "SHRINKWRAP";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  blender::bke::node_type_storage(
      ntype, "NodeGeometryShrinkwrap", node_free_standard_storage, node_copy_standard_storage);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  blender::bke::node_register_type(ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_shrinkwrap_cc
