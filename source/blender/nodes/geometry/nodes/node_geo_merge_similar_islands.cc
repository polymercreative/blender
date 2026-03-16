/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_foreach_geometry.hh"
#include "GEO_mesh_merge_similar_islands.hh"

#include "DNA_node_types.h"

#include "NOD_rna_define.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_merge_similar_islands_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Mesh")
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Mesh with face attribute to merge");
  b.add_input<decl::String>("Attribute")
      .description(
          "Face attribute to analyze. Islands are contiguous regions with the same value");
  b.add_input<decl::Float>("Threshold")
      .default_value(0.1f)
      .min(0.0f)
      .description(
          "Maximum difference between attribute values for islands to merge. "
          "For vectors, this is the Euclidean distance or angular difference");
  b.add_input<decl::Int>("Iterations")
      .default_value(1)
      .min(0)
      .description(
          "Maximum number of merge passes. 0 = unlimited (merge until stable). "
          "1 = only merge direct neighbors that meet threshold");
  b.add_output<decl::Geometry>("Mesh").propagate_all();
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout->prop(ptr, "merge_mode", UI_ITEM_NONE, "", ICON_NONE);
  layout->prop(ptr, "vector_mode", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = int(geometry::MergeSimilarMode::Largest);
  node->custom2 = int(geometry::VectorDifferenceMode::Distance);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh");
  const std::string attribute_name = params.extract_input<std::string>("Attribute");
  const float threshold = std::max(params.extract_input<float>("Threshold"), 0.0f);
  const int iterations = std::max(params.extract_input<int>("Iterations"), 0);
  const bNode &node = params.node();
  const geometry::MergeSimilarMode merge_mode = geometry::MergeSimilarMode(node.custom1);
  const geometry::VectorDifferenceMode vector_mode = geometry::VectorDifferenceMode(node.custom2);

  if (attribute_name.empty()) {
    params.error_message_add(NodeWarningType::Error, "Attribute name cannot be empty");
    params.set_output("Mesh", std::move(geometry_set));
    return;
  }

  geometry::foreach_real_geometry(geometry_set, [&](GeometrySet &geometry_set) {
    if (const Mesh *mesh = geometry_set.get_mesh()) {
      Mesh *result = geometry::mesh_merge_similar_attribute_islands(
          *mesh, attribute_name, threshold, merge_mode, vector_mode, iterations);
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

static void node_rna(StructRNA *srna)
{
  static const EnumPropertyItem merge_mode_items[] = {
      {int(geometry::MergeSimilarMode::Largest),
       "LARGEST",
       0,
       "Largest",
       "Use attribute value from the largest island in the merged group"},
      {int(geometry::MergeSimilarMode::Average),
       "AVERAGE",
       0,
       "Average",
       "Average attribute values of all islands in the merged group"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static const EnumPropertyItem vector_mode_items[] = {
      {int(geometry::VectorDifferenceMode::Distance),
       "DISTANCE",
       0,
       "Distance",
       "Euclidean distance between vectors. Good for positions and colors"},
      {int(geometry::VectorDifferenceMode::Angular),
       "ANGULAR",
       0,
       "Angular",
       "Angular difference between vectors. Good for normals and directions"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "merge_mode",
                    "Merge Mode",
                    "How to determine the final attribute value when merging islands",
                    merge_mode_items,
                    NOD_inline_enum_accessors(custom1),
                    int(geometry::MergeSimilarMode::Largest));

  RNA_def_node_enum(srna,
                    "vector_mode",
                    "Vector Mode",
                    "How to calculate difference for vector attributes",
                    vector_mode_items,
                    NOD_inline_enum_accessors(custom2),
                    int(geometry::VectorDifferenceMode::Distance));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeMergeSimilarIslands", GEO_NODE_MERGE_SIMILAR_ISLANDS);
  ntype.ui_name = "Merge Similar Attribute Islands";
  ntype.ui_description =
      "Merge adjacent attribute islands that have similar values. "
      "Useful for simplifying mesh segmentation by combining similar regions";
  ntype.enum_name_legacy = "MERGE_SIMILAR_ISLANDS";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_merge_similar_islands_cc
