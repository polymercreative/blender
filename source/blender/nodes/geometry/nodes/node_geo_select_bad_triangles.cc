/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_mesh_select_bad_triangles.hh"

#include "BKE_mesh.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_select_bad_triangles_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Float>("Min Angle")
      .default_value(0.0f)
      .min(0.0f)
      .max(180.0f)
      .subtype(PROP_ANGLE)
      .description("Select faces with angles below this threshold (degrees, 0 = disabled)");
  b.add_input<decl::Float>("Max Angle")
      .default_value(180.0f)
      .min(0.0f)
      .max(180.0f)
      .subtype(PROP_ANGLE)
      .description("Select faces with angles above this threshold (degrees, 180 = disabled)");
  b.add_input<decl::Float>("Min Area")
      .default_value(0.0f)
      .min(0.0f)
      .description("Select faces with area below this (0 = disabled)");
  b.add_input<decl::Float>("Max Area")
      .default_value(FLT_MAX)
      .min(0.0f)
      .description("Select faces with area above this");
  b.add_input<decl::Float>("Falloff Distance")
      .default_value(0.0f)
      .min(0.0f)
      .description("Extend selection outward by this distance with smooth falloff");
  b.add_output<decl::Bool>("Selection")
      .field_source()
      .description("Boolean selection of bad triangle vertices");
  b.add_output<decl::Float>("Weight")
      .field_source()
      .description("Smooth falloff weights (1.0 = selected, 0.0 = unselected)");
}

class BadTriangleSelectionFieldInput final : public bke::MeshFieldInput {
  float min_angle_radians_;
  float max_angle_radians_;
  float min_area_;
  float max_area_;
  float falloff_distance_;
  bool output_weight_;

 public:
  BadTriangleSelectionFieldInput(float min_angle_radians,
                                  float max_angle_radians,
                                  float min_area,
                                  float max_area,
                                  float falloff_distance,
                                  bool output_weight)
      : bke::MeshFieldInput(output_weight ? CPPType::get<float>() : CPPType::get<bool>(),
                            "Bad Triangle Selection"),
        min_angle_radians_(min_angle_radians),
        max_angle_radians_(max_angle_radians),
        min_area_(min_area),
        max_area_(max_area),
        falloff_distance_(falloff_distance),
        output_weight_(output_weight)
  {
    category_ = Category::Generated;
  }

  GVArray get_varray_for_context(const Mesh &mesh,
                                  const AttrDomain domain,
                                  const IndexMask & /*mask*/) const final
  {
    if (mesh.verts_num == 0) {
      return {};
    }

    /* Calculate selections. */
    Array<bool> vertex_selection;
    Array<float> vertex_weights;
    geometry::select_bad_triangles(mesh,
                                   min_angle_radians_,
                                   max_angle_radians_,
                                   min_area_,
                                   max_area_,
                                   falloff_distance_,
                                   vertex_selection,
                                   vertex_weights);

    if (output_weight_) {
      return mesh.attributes().adapt_domain<float>(
          VArray<float>::from_container(std::move(vertex_weights)), AttrDomain::Point, domain);
    }
    else {
      return mesh.attributes().adapt_domain<bool>(
          VArray<bool>::from_container(std::move(vertex_selection)), AttrDomain::Point, domain);
    }
  }

  uint64_t hash() const final
  {
    return get_default_hash(get_default_hash(min_angle_radians_, max_angle_radians_),
                            get_default_hash(min_area_, max_area_),
                            get_default_hash(falloff_distance_, output_weight_));
  }

  bool is_equal_to(const fn::FieldNode &other) const final
  {
    if (const BadTriangleSelectionFieldInput *other_field =
            dynamic_cast<const BadTriangleSelectionFieldInput *>(&other))
    {
      return min_angle_radians_ == other_field->min_angle_radians_ &&
             max_angle_radians_ == other_field->max_angle_radians_ &&
             min_area_ == other_field->min_area_ && max_area_ == other_field->max_area_ &&
             falloff_distance_ == other_field->falloff_distance_ &&
             output_weight_ == other_field->output_weight_;
    }
    return false;
  }

  std::optional<AttrDomain> preferred_domain(const Mesh & /*mesh*/) const final
  {
    return AttrDomain::Point;
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  /* Get parameters (angles are already in radians due to PROP_ANGLE subtype). */
  const float min_angle_radians = params.extract_input<float>("Min Angle");
  const float max_angle_radians = params.extract_input<float>("Max Angle");
  const float min_area = params.extract_input<float>("Min Area");
  const float max_area = params.extract_input<float>("Max Area");
  const float falloff_distance = params.extract_input<float>("Falloff Distance");

  Field<bool> selection_field{std::make_shared<BadTriangleSelectionFieldInput>(
      min_angle_radians, max_angle_radians, min_area, max_area, falloff_distance, false)};
  Field<float> weight_field{std::make_shared<BadTriangleSelectionFieldInput>(
      min_angle_radians, max_angle_radians, min_area, max_area, falloff_distance, true)};

  params.set_output("Selection", std::move(selection_field));
  params.set_output("Weight", std::move(weight_field));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeSelectBadTriangles", GEO_NODE_SELECT_BAD_TRIANGLES);
  ntype.ui_name = "Select Bad Triangles";
  ntype.ui_description =
      "Select vertices based on triangle quality metrics (min/max angle, min/max area) with "
      "smooth falloff for cleanup workflows";
  ntype.enum_name_legacy = "SELECT_BAD_TRIANGLES";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_select_bad_triangles_cc
