/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_mesh_adaptive_remesh.hh"

#include "BKE_attribute.hh"
#include "BKE_mesh.hh"

#include "node_geometry_util.hh"

#include <fmt/format.h>

namespace blender::nodes::node_geo_adaptive_remesh_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Mesh")
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Input mesh to remesh");
  b.add_input<decl::String>("Remesh Mask")
      .default_value("remesh_mask")
      .description("Attribute defining which regions to remesh (0.0 = skip, 1.0 = remesh)");
  b.add_input<decl::String>("Target Edge Length")
      .default_value("target_edge_length")
      .description("Attribute controlling per-vertex target edge resolution");
  b.add_input<decl::Int>("Iterations")
      .default_value(3)
      .min(1)
      .max(20)
      .description("Number of refinement passes");
  b.add_input<decl::Bool>("Preserve Boundaries")
      .default_value(true)
      .description("Keep boundary edges between masked/unmasked regions fixed");
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

  /* Get attribute names. */
  const std::string remesh_mask_name = params.extract_input<std::string>("Remesh Mask");
  const std::string target_length_name = params.extract_input<std::string>("Target Edge Length");

  /* Get parameters. */
  const int iterations = params.extract_input<int>("Iterations");
  const bool preserve_boundaries = params.extract_input<bool>("Preserve Boundaries");

  /* Read attributes. */
  const bke::AttributeAccessor attributes = mesh.attributes();

  /* Get remesh mask attribute. Default to 1.0 (remesh everything) if not found. */
  VArray<float> remesh_mask = *attributes.lookup_or_default<float>(
      remesh_mask_name, bke::AttrDomain::Point, 1.0f);

  /* Get target edge length attribute. Default to 1.0 if not found. */
  VArray<float> target_edge_length = *attributes.lookup_or_default<float>(
      target_length_name, bke::AttrDomain::Point, 1.0f);

  /* Warn if attributes are missing. */
  if (!attributes.contains(remesh_mask_name)) {
    params.error_message_add(
        NodeWarningType::Info,
        fmt::format(fmt::runtime(TIP_("Remesh mask attribute \"{}\" not found, remeshing entire mesh")),
                    remesh_mask_name));
  }
  if (!attributes.contains(target_length_name)) {
    params.error_message_add(
        NodeWarningType::Info,
        fmt::format(fmt::runtime(
                        TIP_("Target edge length attribute \"{}\" not found, using default value 1.0")),
                    target_length_name));
  }

  /* Perform adaptive remeshing. */
  Mesh *result_mesh = geometry::adaptive_remesh_mesh(
      mesh, remesh_mask, target_edge_length, iterations, preserve_boundaries);

  if (result_mesh) {
    geometry_set.replace_mesh(result_mesh);
  }

  params.set_output("Mesh", std::move(geometry_set));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeAdaptiveRemesh", GEO_NODE_ADAPTIVE_REMESH);
  ntype.ui_name = "Adaptive Remesh";
  ntype.ui_description =
      "Remesh areas of a mesh based on attributes, achieving target edge lengths through "
      "iterative splits and collapses";
  ntype.enum_name_legacy = "ADAPTIVE_REMESH";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_adaptive_remesh_cc
