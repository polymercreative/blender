/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_foreach_geometry.hh"
#include "GEO_mesh_merge_small_islands.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_merge_small_islands_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh")
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Mesh with face attribute to clean up");
  b.add_output<decl::Geometry>("Mesh").propagate_all().align_with_previous();
  b.add_input<decl::String>("Attribute")
      .description(
          "Face attribute to analyze. Islands are contiguous regions with the same value");
  b.add_input<decl::Int>("Threshold")
      .default_value(10)
      .min(1)
      .description(
          "Minimum face count for an island to be considered large. "
          "Smaller islands will inherit values from neighboring large islands");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh");
  const std::string attribute_name = params.extract_input<std::string>("Attribute");
  const int threshold = std::max(params.extract_input<int>("Threshold"), 1);

  if (attribute_name.empty()) {
    params.error_message_add(NodeWarningType::Error, "Attribute name cannot be empty");
    params.set_output("Mesh", std::move(geometry_set));
    return;
  }

  geometry::foreach_real_geometry(geometry_set, [&](GeometrySet &geometry_set) {
    if (const Mesh *mesh = geometry_set.get_mesh()) {
      Mesh *result = geometry::mesh_merge_small_attribute_islands(
          *mesh, attribute_name, threshold);
      if (result) {
        geometry_set.replace_mesh(result);
      }
      else {
        /* No changes needed or attribute not found - keep original. */
      }
    }
  });

  params.set_output("Mesh", std::move(geometry_set));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeMergeSmallIslands", GEO_NODE_MERGE_SMALL_ISLANDS);
  ntype.ui_name = "Merge Small Attribute Islands";
  ntype.ui_description =
      "Merge small islands of contiguous same-attribute faces into neighboring large islands. "
      "Useful for cleaning up mesh segmentation and removing small attribute regions";
  ntype.enum_name_legacy = "MERGE_SMALL_ISLANDS";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_merge_small_islands_cc
