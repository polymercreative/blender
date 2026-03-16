/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Inset Faces Node - Inset selected faces of a mesh.
 */

#include "BKE_mesh.hh"

#include "GEO_foreach_geometry.hh"

#include "DNA_node_types.h"

#include "NOD_rna_define.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "bmesh.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_inset_cc {

enum class InsetMode {
  Individual = 0,
  Region = 1,
};

struct AttributeOutputs {
  std::optional<std::string> inner_id;
  std::optional<std::string> outer_id;
};

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();

  b.add_input<decl::Geometry>("Mesh")
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Mesh to inset faces on");

  b.add_input<decl::Bool>("Selection")
      .default_value(true)
      .hide_value()
      .field_on_all()
      .description("Faces to inset");

  b.add_input<decl::Float>("Thickness")
      .default_value(0.01f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Thickness of the inset");

  b.add_input<decl::Float>("Depth")
      .default_value(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Depth of the inset (positive = inward, negative = outward)");

  if (node != nullptr) {
    const InsetMode mode = InsetMode(node->custom1);
    if (mode == InsetMode::Region) {
      b.add_input<decl::Bool>("Boundary")
          .default_value(true)
          .description("Inset face boundaries");
    }
  }

  b.add_input<decl::Bool>("Even Offset")
      .default_value(true)
      .description("Scale the offset to give more even thickness");

  b.add_input<decl::Bool>("Relative Offset")
      .default_value(false)
      .description("Scale the offset by surrounding edge lengths");

  b.add_output<decl::Geometry>("Mesh").propagate_all();
  b.add_output<decl::Bool>("Inner").field_on_all().description("The inner inset faces");
  b.add_output<decl::Bool>("Outer").field_on_all().description("The surrounding outer faces");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout->prop(ptr, "mode", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = int(InsetMode::Individual);
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

/**
 * Perform face inset using BMesh operators (matches Blender's modeling operator).
 * Returns the result mesh and tracks which faces are top (inset) and side faces.
 */
static Mesh *inset_mesh(const Mesh &mesh,
                        const InsetMode mode,
                        const float thickness,
                        const float depth,
                        const bool use_boundary,
                        const bool use_even_offset,
                        const bool use_relative_offset,
                        const Span<bool> selection,
                        const AttributeOutputs &attribute_outputs)
{
  if (mesh.faces_num == 0) {
    return nullptr;
  }

  /* Check if any faces are selected */
  bool any_selected = false;
  if (selection.is_empty()) {
    any_selected = true;
  }
  else {
    for (const bool selected : selection) {
      if (selected) {
        any_selected = true;
        break;
      }
    }
  }

  if (!any_selected) {
    return nullptr;
  }

  /* Convert mesh to BMesh with tool flags enabled for operator use */
  BMeshCreateParams create_params{};
  create_params.use_toolflags = true; /* Required for BMesh operators */
  BMeshFromMeshParams convert_params{};
  convert_params.calc_face_normal = true;
  convert_params.calc_vert_normal = true;

  BMesh *bm = BKE_mesh_to_bmesh_ex(&mesh, &create_params, &convert_params);

  /* Tag selected faces using BM_ELEM_TAG */
  BM_mesh_elem_hflag_disable_all(bm, BM_FACE, BM_ELEM_TAG, false);

  BMIter iter;
  BMFace *f;
  int f_idx = 0;
  BM_ITER_MESH_INDEX (f, &iter, bm, BM_FACES_OF_MESH, f_idx) {
    bool selected = selection.is_empty() || selection[f_idx];
    if (selected) {
      BM_elem_flag_enable(f, BM_ELEM_TAG);
    }
  }

  /* Use BMesh operators - these are the same operators used by the actual inset tool */
  BMOperator op;

  if (mode == InsetMode::Individual) {
    /* Individual face inset using BMesh operator */
    BMO_op_initf(bm,
                 &op,
                 BMO_FLAG_DEFAULTS,
                 "inset_individual faces=%hf use_even_offset=%b use_relative_offset=%b "
                 "use_interpolate=%b thickness=%f depth=%f",
                 BM_ELEM_TAG,
                 use_even_offset,
                 use_relative_offset,
                 true, /* use_interpolate */
                 thickness,
                 depth);
  }
  else {
    /* Region inset using BMesh operator */
    BMO_op_initf(bm,
                 &op,
                 BMO_FLAG_DEFAULTS,
                 "inset_region faces=%hf use_boundary=%b use_even_offset=%b use_relative_offset=%b "
                 "use_interpolate=%b thickness=%f depth=%f use_outset=%b use_edge_rail=%b",
                 BM_ELEM_TAG,
                 use_boundary,
                 use_even_offset,
                 use_relative_offset,
                 true,  /* use_interpolate */
                 thickness,
                 depth,
                 false, /* use_outset */
                 false  /* use_edge_rail */);
  }

  BMO_op_exec(bm, &op);

  /* Collect outer faces (new faces from faces.out) before finishing the operator.
   * The faces.out slot contains faces flagged with ELE_NEW - the newly created outer faces.
   * We always need to collect these to properly compute Inner = selection - Outer. */
  Vector<BMFace *> outer_faces;
  BMOIter oiter;
  BMFace *face_out;
  BMO_ITER (face_out, &oiter, op.slots_out, "faces.out", BM_FACE) {
    outer_faces.append(face_out);
  }

  BMO_op_finish(bm, &op);

  BM_mesh_normals_update(bm);

  Mesh *result = BKE_mesh_from_bmesh_for_eval_nomain(bm, nullptr, &mesh);

  /* Now create the selection attributes on the result mesh */
  if (attribute_outputs.inner_id || attribute_outputs.outer_id) {
    bke::MutableAttributeAccessor attributes = result->attributes_for_write();

    /* Build index masks for inner and outer faces.
     * Inner = original selected faces (still have BM_ELEM_TAG from before the operation)
     * Outer = new faces created by the inset (from faces.out) */
    Vector<int> inner_indices;
    Vector<int> outer_indices;

    /* Create a set of outer face pointers for quick lookup */
    Set<BMFace *> outer_face_set;
    for (BMFace *face : outer_faces) {
      outer_face_set.add(face);
    }

    f_idx = 0;
    BM_ITER_MESH_INDEX (f, &iter, bm, BM_FACES_OF_MESH, f_idx) {
      if (outer_face_set.contains(f)) {
        /* Outer faces are the new faces from faces.out */
        outer_indices.append(f_idx);
      }
      else if (BM_elem_flag_test(f, BM_ELEM_TAG)) {
        /* Inner faces are the original selected faces MINUS outer faces */
        inner_indices.append(f_idx);
      }
    }

    IndexMaskMemory memory;

    if (attribute_outputs.inner_id) {
      const IndexMask inner_mask = IndexMask::from_indices(inner_indices.as_span(), memory);
      save_selection_as_attribute(
          attributes, *attribute_outputs.inner_id, bke::AttrDomain::Face, inner_mask);
    }

    if (attribute_outputs.outer_id) {
      const IndexMask outer_mask = IndexMask::from_indices(outer_indices.as_span(), memory);
      save_selection_as_attribute(
          attributes, *attribute_outputs.outer_id, bke::AttrDomain::Face, outer_mask);
    }
  }

  BM_mesh_free(bm);

  return result;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh");
  const bNode &node = params.node();
  const InsetMode mode = InsetMode(node.custom1);

  const float thickness = params.get_input<float>("Thickness");
  const float depth = params.get_input<float>("Depth");
  const Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");

  bool use_boundary = true;
  if (mode == InsetMode::Region) {
    use_boundary = params.get_input<bool>("Boundary");
  }

  const bool use_even_offset = params.get_input<bool>("Even Offset");
  const bool use_relative_offset = params.get_input<bool>("Relative Offset");

  /* Get output attribute IDs if they are connected */
  AttributeOutputs attribute_outputs;
  attribute_outputs.inner_id = params.get_output_anonymous_attribute_id_if_needed("Inner");
  attribute_outputs.outer_id = params.get_output_anonymous_attribute_id_if_needed("Outer");

  if (thickness == 0.0f && depth == 0.0f) {
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

      Mesh *result = inset_mesh(*mesh,
                                mode,
                                thickness,
                                depth,
                                use_boundary,
                                use_even_offset,
                                use_relative_offset,
                                selection,
                                attribute_outputs);

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
      {int(InsetMode::Individual),
       "INDIVIDUAL",
       0,
       "Individual",
       "Inset each selected face independently"},
      {int(InsetMode::Region),
       "REGION",
       0,
       "Region",
       "Inset the boundary of the selected face region"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "mode",
                    "Mode",
                    "How to inset faces",
                    mode_items,
                    NOD_inline_enum_accessors(custom1),
                    int(InsetMode::Individual));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeInset", GEO_NODE_INSET);
  ntype.ui_name = "Inset Faces";
  ntype.ui_description = "Inset selected faces of a mesh";
  ntype.enum_name_legacy = "INSET";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  blender::bke::node_register_type(ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_inset_cc
