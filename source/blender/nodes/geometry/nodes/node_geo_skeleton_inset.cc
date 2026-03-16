/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Skeleton Inset Node - Inset faces using straight skeleton algorithm.
 *
 * Uses CGAL-style event-driven wavefront propagation for proper angular merging
 * where vertices collapse and polygons may split. Handles any polygon shape
 * including concave and those with holes.
 */

#include "BKE_mesh.hh"

#include "GEO_foreach_geometry.hh"
#include "GEO_mesh_skeleton_inset.hh"

#include "DNA_node_types.h"

#include "NOD_rna_define.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_skeleton_inset_cc {

enum class InsetMode {
  Region = 0,
  Individual = 1,
};

struct AttributeOutputs {
  std::optional<std::string> inner_id;
  std::optional<std::string> outer_id;
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Mesh")
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Mesh to inset faces on");

  b.add_input<decl::Bool>("Selection")
      .default_value(true)
      .hide_value()
      .field_on_all()
      .description("Faces to inset");

  b.add_input<decl::Float>("Distance")
      .default_value(0.1f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Inset distance (wavefront propagation time)");

  b.add_input<decl::Float>("Depth")
      .default_value(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Perpendicular extrusion depth");

  b.add_output<decl::Geometry>("Mesh").propagate_all();
  b.add_output<decl::Bool>("Inner")
      .field_on_all()
      .description("The inner inset faces (may be multiple after splits)");
  b.add_output<decl::Bool>("Outer")
      .field_on_all()
      .description("The surrounding rim faces");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout->prop(ptr, "mode", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = int(InsetMode::Region);  /* Region is default/primary mode */
}

static void save_selection_as_attribute(bke::MutableAttributeAccessor attributes,
                                        const StringRef id,
                                        const bke::AttrDomain domain,
                                        const IndexMask &selection)
{
  BLI_assert(!attributes.contains(id));
  bke::SpanAttributeWriter<bool> attribute = attributes.lookup_or_add_for_write_span<bool>(id,
                                                                                           domain);
  selection.to_bools(attribute.span);
  attribute.finish();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh");
  const bNode &node = params.node();
  const InsetMode mode = InsetMode(node.custom1);

  const float distance = params.get_input<float>("Distance");
  const float depth = params.get_input<float>("Depth");
  const Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");

  /* Get output attribute IDs if they are connected */
  AttributeOutputs attribute_outputs;
  attribute_outputs.inner_id = params.get_output_anonymous_attribute_id_if_needed("Inner");
  attribute_outputs.outer_id = params.get_output_anonymous_attribute_id_if_needed("Outer");

  if (distance <= 0.0f) {
    params.set_output("Mesh", std::move(geometry_set));
    return;
  }

  geometry::foreach_real_geometry(geometry_set, [&](GeometrySet &geometry_set) {
    if (const Mesh *mesh = geometry_set.get_mesh()) {
      const bke::MeshFieldContext field_context{*mesh, bke::AttrDomain::Face};
      const int faces_num = mesh->faces_num;

      fn::FieldEvaluator selection_evaluator{field_context, faces_num};
      selection_evaluator.add(selection_field);
      selection_evaluator.evaluate();
      const VArray<bool> selection_varray = selection_evaluator.get_evaluated<bool>(0);

      Array<bool> selection(faces_num);
      selection_varray.materialize(selection.as_mutable_span());

      std::optional<geometry::SkeletonInsetResult> result = geometry::mesh_skeleton_inset(
          *mesh,
          selection,
          distance,
          depth,
          geometry::SkeletonInsetMode(int(mode)));

      if (result.has_value() && result->mesh) {
        /* Apply selection attributes if connected */
        if (attribute_outputs.inner_id || attribute_outputs.outer_id) {
          bke::MutableAttributeAccessor attributes = result->mesh->attributes_for_write();
          IndexMaskMemory memory;

          if (attribute_outputs.inner_id && !result->inner_face_indices.is_empty()) {
            const IndexMask inner_mask = IndexMask::from_indices(
                result->inner_face_indices.as_span(), memory);
            save_selection_as_attribute(
                attributes, *attribute_outputs.inner_id, bke::AttrDomain::Face, inner_mask);
          }

          if (attribute_outputs.outer_id && !result->outer_face_indices.is_empty()) {
            const IndexMask outer_mask = IndexMask::from_indices(
                result->outer_face_indices.as_span(), memory);
            save_selection_as_attribute(
                attributes, *attribute_outputs.outer_id, bke::AttrDomain::Face, outer_mask);
          }
        }

        geometry_set.replace_mesh(result->mesh);
      }
    }
  });

  params.set_output("Mesh", std::move(geometry_set));
}

static void node_rna(StructRNA *srna)
{
  static const EnumPropertyItem mode_items[] = {
      {int(InsetMode::Region),
       "REGION",
       0,
       "Region",
       "Inset the boundary of the selected face region (handles holes)"},
      {int(InsetMode::Individual),
       "INDIVIDUAL",
       0,
       "Individual",
       "Inset each selected face independently"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "mode",
                    "Mode",
                    "How to compute the skeleton inset",
                    mode_items,
                    NOD_inline_enum_accessors(custom1),
                    int(InsetMode::Region));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeSkeletonInset", GEO_NODE_SKELETON_INSET);
  ntype.ui_name = "Skeleton Inset";
  ntype.ui_description = "Inset faces using straight skeleton algorithm with proper angular merging";
  ntype.enum_name_legacy = "SKELETON_INSET";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  blender::bke::node_register_type(ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_skeleton_inset_cc
