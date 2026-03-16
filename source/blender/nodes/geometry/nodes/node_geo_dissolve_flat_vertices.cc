/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_mesh_dissolve_flat_vertices.hh"

#include "BKE_mesh.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_dissolve_flat_vertices_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Mesh")
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Input mesh with tessellated triangles");
  b.add_input<decl::Float>("Curvature Threshold")
      .default_value(0.05f)
      .min(0.0f)
      .max(1.0f)
      .description("Dissolve vertices with curvature below this threshold (lower = flatter)");
  b.add_input<decl::Float>("Min Angle")
      .default_value(10.0f)
      .min(0.0f)
      .max(180.0f)
      .subtype(PROP_ANGLE)
      .description(
          "Dissolve vertices in faces with angles below this threshold (degrees). "
          "Use 5-15 degrees to cleanup thin triangles from decimation");
  b.add_output<decl::Geometry>("Mesh").propagate_all();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh");

  /* Only process mesh geometry. */
  if (!geometry_set.has_mesh()) {
    params.set_output("Mesh", std::move(geometry_set));
    return;
  }

  const Mesh &mesh = *geometry_set.get_mesh();
  if (mesh.verts_num == 0) {
    params.set_output("Mesh", std::move(geometry_set));
    return;
  }

  /* Get parameters. */
  const float curvature_threshold = params.extract_input<float>("Curvature Threshold");
  const float min_angle_radians = params.extract_input<float>("Min Angle");
  /* Note: Min Angle has PROP_ANGLE subtype, so Blender automatically converts
   * the user's degree input to radians before we receive it here. */

  /* Perform flat vertex dissolution. */
  Mesh *result_mesh = geometry::dissolve_flat_vertices_mesh(
      mesh, curvature_threshold, min_angle_radians);

  if (result_mesh) {
    geometry_set.replace_mesh(result_mesh);
  }

  params.set_output("Mesh", std::move(geometry_set));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeDissolveFlatVertices", GEO_NODE_DISSOLVE_FLAT_VERTICES);
  ntype.ui_name = "Dissolve Flat Vertices";
  ntype.ui_description =
      "Remove low-curvature vertices from tessellated meshes to reduce polycount in flat areas "
      "while preserving detail in curved regions";
  ntype.enum_name_legacy = "DISSOLVE_FLAT_VERTICES";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_dissolve_flat_vertices_cc
