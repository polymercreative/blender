/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_foreach_geometry.hh"
#include "GEO_mesh_fix_poles.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_fix_poles_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh")
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Mesh to fix 3-edge poles on");
  b.add_output<decl::Geometry>("Mesh").propagate_all().align_with_previous();
  b.add_input<decl::Int>("Smooth Iterations")
      .default_value(4)
      .min(0)
      .max(10)
      .description("Number of smoothing iterations to apply after fixing poles");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh");
  const int smooth_iterations = std::max(params.extract_input<int>("Smooth Iterations"), 0);

  geometry::foreach_real_geometry(geometry_set, [&](GeometrySet &geometry_set) {
    if (const Mesh *mesh = geometry_set.get_mesh()) {
      std::optional<Mesh *> result = geometry::mesh_fix_poles(*mesh, smooth_iterations);
      if (result) {
        geometry_set.replace_mesh(*result);
      }
    }
  });

  params.set_output("Mesh", std::move(geometry_set));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeFixPoles", GEO_NODE_FIX_POLES);
  ntype.ui_name = "Fix Poles";
  ntype.ui_description =
      "Fix 3-edge poles (vertices with exactly 3 edges) by collapsing pairs that share a face. "
      "Useful for cleaning up meshes after voxel remeshing";
  ntype.enum_name_legacy = "FIX_POLES";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_fix_poles_cc
