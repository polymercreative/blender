/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "MEM_guardedalloc.h"

#include "BKE_mesh.hh"

#include "GEO_foreach_geometry.hh"
#include "GEO_mesh_decimate.hh"

#include "DNA_node_types.h"

#include "NOD_rna_define.hh"

#include "RNA_access.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_decimate_cc {

NODE_STORAGE_FUNCS(NodeGeometryDecimate)

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();

  b.add_input<decl::Geometry>("Mesh")
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Mesh to decimate");

  b.add_input<decl::Bool>("Selection")
      .default_value(true)
      .hide_value()
      .field_on_all()
      .description("Vertices to include in decimation");

  if (node != nullptr) {
    const NodeGeometryDecimate &storage = node_storage(*node);
    const geometry::DecimateMode mode = geometry::DecimateMode(storage.mode);

    switch (mode) {
      case geometry::DecimateMode::Collapse:
        b.add_input<decl::Float>("Ratio")
            .default_value(0.5f)
            .min(0.0f)
            .max(1.0f)
            .subtype(PROP_FACTOR)
            .description("Target ratio of faces to keep (0.0 = maximum reduction, 1.0 = no change)");
        b.add_input<decl::Bool>("Symmetry")
            .default_value(false)
            .description("Maintain symmetry on an axis");
        b.add_input<decl::Bool>("Triangulate")
            .default_value(false)
            .description("Output triangulated mesh");
        break;
      case geometry::DecimateMode::UnSubdivide:
        b.add_input<decl::Int>("Iterations")
            .default_value(1)
            .min(0)
            .max(100)
            .description("Number of times to un-subdivide the mesh");
        break;
      case geometry::DecimateMode::Planar:
        b.add_input<decl::Float>("Angle Limit")
            .default_value(0.0872665f) /* 5 degrees */
            .min(0.0f)
            .max(M_PI)
            .subtype(PROP_ANGLE)
            .description("Maximum angle between face normals to dissolve");
        b.add_input<decl::Bool>("All Boundaries")
            .default_value(false)
            .description("Dissolve vertices on mesh boundaries");
        break;
    }
  }

  b.add_output<decl::Geometry>("Mesh").propagate_all();
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout->prop(ptr, "mode", UI_ITEM_NONE, "", ICON_NONE);

  const int mode = RNA_enum_get(ptr, "mode");
  if (mode == int(geometry::DecimateMode::Collapse)) {
    layout->prop(ptr, "symmetry_axis", UI_ITEM_R_EXPAND, std::nullopt, ICON_NONE);
  }
  else if (mode == int(geometry::DecimateMode::Planar)) {
    layout->prop(ptr, "delimit", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  }
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryDecimate *data = MEM_callocN<NodeGeometryDecimate>(__func__);
  data->mode = int(geometry::DecimateMode::Collapse);
  data->symmetry_axis = 0; /* X axis */
  data->delimit = 0;       /* No delimit flags */
  node->storage = data;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh");
  const NodeGeometryDecimate &storage = node_storage(params.node());
  const geometry::DecimateMode mode = geometry::DecimateMode(storage.mode);

  /* Extract selection field. */
  const Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");

  geometry::foreach_real_geometry(geometry_set, [&](GeometrySet &geometry_set) {
    if (const Mesh *mesh = geometry_set.get_mesh()) {
      /* Evaluate selection field to boolean array on vertex domain. */
      const bke::MeshFieldContext field_context{*mesh, bke::AttrDomain::Point};
      fn::FieldEvaluator selection_evaluator{field_context, mesh->verts_num};
      selection_evaluator.add(selection_field);
      selection_evaluator.evaluate();
      const VArray<bool> selection_varray = selection_evaluator.get_evaluated<bool>(0);

      /* Convert to plain array for passing to geometry functions. */
      Array<bool> selection(mesh->verts_num);
      selection_varray.materialize(selection.as_mutable_span());

      Mesh *result = nullptr;

      switch (mode) {
        case geometry::DecimateMode::Collapse: {
          const float ratio = std::clamp(params.get_input<float>("Ratio"), 0.0f, 1.0f);
          const bool use_symmetry = params.get_input<bool>("Symmetry");
          const bool triangulate = params.get_input<bool>("Triangulate");

          result = geometry::mesh_decimate_collapse(*mesh,
                                                    ratio,
                                                    selection,
                                                    false, /* use_vertex_group */
                                                    nullptr, /* vertex_weights */
                                                    1.0f, /* vertex_group_factor */
                                                    use_symmetry,
                                                    storage.symmetry_axis,
                                                    triangulate);
          break;
        }
        case geometry::DecimateMode::UnSubdivide: {
          const int iterations = std::max(params.get_input<int>("Iterations"), 0);
          result = geometry::mesh_decimate_unsubdivide(*mesh, iterations, selection);
          break;
        }
        case geometry::DecimateMode::Planar: {
          const float angle_limit = std::max(params.get_input<float>("Angle Limit"), 0.0f);
          const bool dissolve_boundaries = params.get_input<bool>("All Boundaries");
          result = geometry::mesh_decimate_planar(*mesh,
                                                  angle_limit,
                                                  dissolve_boundaries,
                                                  storage.delimit,
                                                  selection);
          break;
        }
      }

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
      {int(geometry::DecimateMode::Collapse),
       "COLLAPSE",
       0,
       "Collapse",
       "Use edge collapse with quadric error metrics to reduce face count"},
      {int(geometry::DecimateMode::UnSubdivide),
       "UNSUBDIV",
       0,
       "Un-Subdivide",
       "Reverse subdivision to reduce face count"},
      {int(geometry::DecimateMode::Planar),
       "PLANAR",
       0,
       "Planar",
       "Dissolve faces that share similar normals"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static const EnumPropertyItem symmetry_axis_items[] = {
      {0, "X", 0, "X", "X axis"},
      {1, "Y", 0, "Y", "Y axis"},
      {2, "Z", 0, "Z", "Z axis"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static const EnumPropertyItem delimit_items[] = {
      {0, "NONE", 0, "None", "No delimiting"},
      {1, "NORMAL", 0, "Normal", "Delimit by face normals"},
      {2, "MATERIAL", 0, "Material", "Delimit by material"},
      {4, "SEAM", 0, "Seam", "Delimit by edge seams"},
      {8, "SHARP", 0, "Sharp", "Delimit by sharp edges"},
      {16, "UV", 0, "UVs", "Delimit by UV coordinates"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "mode",
                    "Mode",
                    "Decimation method to use",
                    mode_items,
                    NOD_storage_enum_accessors(mode),
                    int(geometry::DecimateMode::Collapse));

  RNA_def_node_enum(srna,
                    "symmetry_axis",
                    "Symmetry Axis",
                    "Axis of symmetry for symmetric decimation",
                    symmetry_axis_items,
                    NOD_storage_enum_accessors(symmetry_axis),
                    0);

  RNA_def_node_enum(srna,
                    "delimit",
                    "Delimit",
                    "Limit dissolving to boundaries of these properties",
                    delimit_items,
                    NOD_storage_enum_accessors(delimit),
                    0);
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeDecimate", GEO_NODE_DECIMATE);
  ntype.ui_name = "Decimate";
  ntype.ui_description =
      "Reduce the number of faces in a mesh while preserving shape. "
      "Supports edge collapse (QEM), un-subdivide, and planar dissolve methods";
  ntype.enum_name_legacy = "DECIMATE";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  blender::bke::node_type_storage(
      ntype, "NodeGeometryDecimate", node_free_standard_storage, node_copy_standard_storage);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  blender::bke::node_register_type(ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_decimate_cc
