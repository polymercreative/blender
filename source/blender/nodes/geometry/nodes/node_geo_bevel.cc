/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "MEM_guardedalloc.h"

#include "BKE_mesh.hh"

#include "GEO_foreach_geometry.hh"
#include "GEO_mesh_bevel.hh"

#include "DNA_modifier_types.h"
#include "DNA_node_types.h"

#include "NOD_rna_define.hh"

#include "RNA_access.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_bevel_cc {

NODE_STORAGE_FUNCS(NodeGeometryBevel)

enum class BevelMode {
  Vertices = 0,
  Edges = 1,
};

enum class BevelOffsetType {
  Offset = 0,
  Width = 1,
  Depth = 2,
  Percent = 3,
  Absolute = 4,
};

enum class ClampMethod {
  Standard = 0,
  Collision = 1,
};

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();

  b.add_input<decl::Geometry>("Mesh")
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Mesh to bevel");

  if (node != nullptr) {
    const NodeGeometryBevel &storage = node_storage(*node);
    const BevelMode mode = BevelMode(storage.mode);

    if (mode == BevelMode::Vertices) {
      b.add_input<decl::Bool>("Selection")
          .default_value(true)
          .hide_value()
          .field_on_all()
          .description("Vertices to bevel");
    }
    else {
      b.add_input<decl::Bool>("Selection")
          .default_value(true)
          .hide_value()
          .field_on_all()
          .description("Edges to bevel");
    }
  }
  else {
    b.add_input<decl::Bool>("Selection")
        .default_value(true)
        .hide_value()
        .field_on_all()
        .description("Elements to bevel");
  }

  b.add_input<decl::Float>("Amount")
      .default_value(0.1f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Bevel offset amount");

  b.add_input<decl::Int>("Segments")
      .default_value(1)
      .min(1)
      .max(100)
      .description("Number of segments in the bevel");

  b.add_input<decl::Float>("Profile")
      .default_value(0.5f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR)
      .description("Profile shape (0.5 = round)");

  b.add_output<decl::Geometry>("Mesh").propagate_all();
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.prop(ptr, "mode", ui::ITEM_R_EXPAND, std::nullopt, ICON_NONE);
  layout.prop(ptr, "offset_type", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  layout.prop(ptr, "clamp_method", UI_ITEM_NONE, std::nullopt, ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryBevel *data = MEM_new<NodeGeometryBevel>(__func__);
  data->mode = int8_t(BevelMode::Edges);
  data->offset_type = int8_t(BevelOffsetType::Offset);
  node->storage = data;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh");
  const NodeGeometryBevel &storage = node_storage(params.node());
  const BevelMode mode = BevelMode(storage.mode);
  const BevelOffsetType offset_type = BevelOffsetType(storage.offset_type);
  const ClampMethod clamp_method = ClampMethod(storage.clamp_method);

  const float amount = params.get_input<float>("Amount");
  const int segments = std::max(params.get_input<int>("Segments"), 1);
  const float profile = std::clamp(params.get_input<float>("Profile"), 0.0f, 1.0f);
  const Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");

  /* Early exit if amount is zero. */
  if (amount == 0.0f) {
    params.set_output("Mesh", std::move(geometry_set));
    return;
  }

  const bool affect_vertices = (mode == BevelMode::Vertices);

  geometry::foreach_real_geometry(geometry_set, [&](GeometrySet &geometry_set) {
    if (const Mesh *mesh = geometry_set.get_mesh()) {
      /* Evaluate selection field on appropriate domain. */
      const bke::AttrDomain domain = affect_vertices ? bke::AttrDomain::Point :
                                                       bke::AttrDomain::Edge;
      const bke::MeshFieldContext field_context{*mesh, domain};
      const int domain_size = affect_vertices ? mesh->verts_num : mesh->edges_num;

      fn::FieldEvaluator selection_evaluator{field_context, domain_size};
      selection_evaluator.add(selection_field);
      selection_evaluator.evaluate();
      const VArray<bool> selection_varray = selection_evaluator.get_evaluated<bool>(0);

      /* Convert to plain array for passing to geometry functions. */
      Array<bool> selection(domain_size);
      selection_varray.materialize(selection.as_mutable_span());

      Mesh *result = geometry::mesh_bevel(*mesh,
                                          amount,
                                          int(offset_type),
                                          segments,
                                          profile,
                                          affect_vertices,
                                          selection,
                                          int(clamp_method));

      if (result) {
        geometry_set.replace_mesh(result);
      }
    }
  });

  params.set_output("Mesh", std::move(geometry_set));
}

static void node_rna(StructRNA *srna)
{
  static const EnumPropertyItem mode_items[] = {
      {int(BevelMode::Vertices), "VERTICES", 0, "Vertices", "Bevel selected vertices"},
      {int(BevelMode::Edges), "EDGES", 0, "Edges", "Bevel selected edges"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static const EnumPropertyItem offset_type_items[] = {
      {int(BevelOffsetType::Offset),
       "OFFSET",
       0,
       "Offset",
       "Amount is offset of new edges from original"},
      {int(BevelOffsetType::Width), "WIDTH", 0, "Width", "Amount is width of new face"},
      {int(BevelOffsetType::Depth),
       "DEPTH",
       0,
       "Depth",
       "Amount is perpendicular distance from original edge"},
      {int(BevelOffsetType::Percent),
       "PERCENT",
       0,
       "Percent",
       "Amount is percent of adjacent edge length"},
      {int(BevelOffsetType::Absolute),
       "ABSOLUTE",
       0,
       "Absolute",
       "Amount is absolute distance along adjacent edge"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "mode",
                    "Mode",
                    "What to bevel",
                    mode_items,
                    NOD_storage_enum_accessors(mode),
                    int(BevelMode::Edges));

  RNA_def_node_enum(srna,
                    "offset_type",
                    "Offset Type",
                    "How to interpret the offset amount",
                    offset_type_items,
                    NOD_storage_enum_accessors(offset_type),
                    int(BevelOffsetType::Offset));

  static const EnumPropertyItem clamp_method_items[] = {
      {int(ClampMethod::Standard),
       "STANDARD",
       0,
       "Standard",
       "Use Blender's built-in overlap limiting"},
      {int(ClampMethod::Collision),
       "COLLISION",
       0,
       "Collision",
       "Per-face collision detection for cleaner straight-skeleton-style bevels"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "clamp_method",
                    "Clamp Method",
                    "How to handle overlapping bevels",
                    clamp_method_items,
                    NOD_storage_enum_accessors(clamp_method),
                    int(ClampMethod::Standard));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeBevel", GEO_NODE_BEVEL);
  ntype.ui_name = "Bevel";
  ntype.ui_description = "Round or chamfer edges and vertices of a mesh";
  ntype.enum_name_legacy = "BEVEL";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  blender::bke::node_type_storage(
      ntype, "NodeGeometryBevel", node_free_standard_storage, node_copy_standard_storage);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  blender::bke::node_register_type(ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_bevel_cc
