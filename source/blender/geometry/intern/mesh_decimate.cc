/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_mesh_decimate.hh"

#include "BKE_mesh.hh"

#include "bmesh.hh"
#include "bmesh_tools.hh"

#include "MEM_guardedalloc.h"

namespace blender::geometry {

Mesh *mesh_decimate_collapse(const Mesh &mesh,
                             const float ratio,
                             const Span<bool> selection,
                             const bool use_vertex_group,
                             const float *vertex_weights,
                             const float vertex_group_factor,
                             const bool use_symmetry,
                             const int symmetry_axis,
                             const bool triangulate)
{
  if (ratio >= 1.0f) {
    /* No decimation needed. */
    return nullptr;
  }

  if (mesh.faces_num <= 3) {
    /* Not enough faces to decimate. */
    return nullptr;
  }

  /* Convert mesh to BMesh. */
  BMeshCreateParams create_params{};
  BMeshFromMeshParams convert_params{};
  convert_params.calc_face_normal = true;
  convert_params.calc_vert_normal = true;
  convert_params.cd_mask_extra.vmask = CD_MASK_ORIGINDEX;
  convert_params.cd_mask_extra.emask = CD_MASK_ORIGINDEX;
  convert_params.cd_mask_extra.pmask = CD_MASK_ORIGINDEX;

  BMesh *bm = BKE_mesh_to_bmesh_ex(&mesh, &create_params, &convert_params);

  /* Build vertex weights combining selection and weights.
   * Selection: false = 0.0 (protected), true = 1.0 (can decimate).
   * If both selection and vertex_weights are provided, multiply them. */
  float *vweights = nullptr;
  const int vert_count = bm->totvert;
  const bool has_selection = !selection.is_empty();
  const bool has_weights = use_vertex_group && vertex_weights != nullptr;

  if (has_selection || has_weights) {
    vweights = MEM_new_array_uninitialized<float>(size_t(vert_count), __func__);

    for (int i = 0; i < vert_count; i++) {
      float weight = 1.0f;

      /* Apply selection: false = protected (0.0), true = can decimate (1.0). */
      if (has_selection) {
        weight = selection[i] ? 1.0f : 0.0f;
      }

      /* Multiply by vertex group weights if provided. */
      if (has_weights) {
        weight *= vertex_weights[i];
      }

      vweights[i] = weight;
    }
  }

  /* Perform decimation. */
  const int sym_axis = use_symmetry ? symmetry_axis : -1;
  const float symmetry_eps = 0.00002f;

  BM_mesh_decimate_collapse(bm,
                            ratio,
                            vweights,
                            vertex_group_factor,
                            triangulate,
                            sym_axis,
                            symmetry_eps);

  if (vweights) {
    MEM_delete(vweights);
  }

  /* Convert back to Mesh. */
  Mesh *result = BKE_mesh_from_bmesh_for_eval_nomain(bm, nullptr, &mesh);
  BM_mesh_free(bm);

  return result;
}

Mesh *mesh_decimate_unsubdivide(const Mesh &mesh,
                                const int iterations,
                                const Span<bool> selection)
{
  if (iterations <= 0) {
    /* No decimation needed. */
    return nullptr;
  }

  /* Convert mesh to BMesh. */
  BMeshCreateParams create_params{};
  BMeshFromMeshParams convert_params{};
  convert_params.calc_face_normal = false;
  convert_params.calc_vert_normal = false;
  convert_params.cd_mask_extra.vmask = CD_MASK_ORIGINDEX;
  convert_params.cd_mask_extra.emask = CD_MASK_ORIGINDEX;
  convert_params.cd_mask_extra.pmask = CD_MASK_ORIGINDEX;

  BMesh *bm = BKE_mesh_to_bmesh_ex(&mesh, &create_params, &convert_params);

  /* Apply selection tagging if provided. */
  if (!selection.is_empty()) {
    BMIter iter;
    BMVert *v;
    int v_index = 0;

    /* Tag only selected vertices. */
    BM_ITER_MESH_INDEX (v, &iter, bm, BM_VERTS_OF_MESH, v_index) {
      if (selection[v_index]) {
        BM_elem_flag_enable(v, BM_ELEM_TAG);
      }
      else {
        BM_elem_flag_disable(v, BM_ELEM_TAG);
      }
    }

    /* Use tag_only mode to respect selection. */
    BM_mesh_decimate_unsubdivide_ex(bm, iterations, true);
  }
  else {
    /* No selection: process all vertices. */
    BM_mesh_decimate_unsubdivide(bm, iterations);
  }

  /* Convert back to Mesh. */
  Mesh *result = BKE_mesh_from_bmesh_for_eval_nomain(bm, nullptr, &mesh);
  BM_mesh_free(bm);

  return result;
}

Mesh *mesh_decimate_planar(const Mesh &mesh,
                           const float angle_limit,
                           const bool dissolve_boundaries,
                           const int delimit,
                           const Span<bool> selection)
{
  if (angle_limit <= 0.0f) {
    /* No decimation needed. */
    return nullptr;
  }

  /* Convert mesh to BMesh. */
  BMeshCreateParams create_params{};
  BMeshFromMeshParams convert_params{};
  convert_params.calc_face_normal = true;
  convert_params.calc_vert_normal = false;
  convert_params.cd_mask_extra.vmask = CD_MASK_ORIGINDEX;
  convert_params.cd_mask_extra.emask = CD_MASK_ORIGINDEX;
  convert_params.cd_mask_extra.pmask = CD_MASK_ORIGINDEX;

  BMesh *bm = BKE_mesh_to_bmesh_ex(&mesh, &create_params, &convert_params);

  if (!selection.is_empty()) {
    /* Build filtered arrays of selected vertices and edges. */
    BMIter iter;
    BMVert *v;
    BMEdge *e;
    int v_index = 0;

    /* First, tag selected vertices for quick lookup. */
    BM_ITER_MESH_INDEX (v, &iter, bm, BM_VERTS_OF_MESH, v_index) {
      if (selection[v_index]) {
        BM_elem_flag_enable(v, BM_ELEM_TAG);
      }
      else {
        BM_elem_flag_disable(v, BM_ELEM_TAG);
      }
    }

    /* Count selected vertices and edges. */
    int vinput_len = 0;
    int einput_len = 0;

    BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
      if (BM_elem_flag_test(v, BM_ELEM_TAG)) {
        vinput_len++;
      }
    }

    BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
      /* Only include edges where both vertices are selected. */
      if (BM_elem_flag_test(e->v1, BM_ELEM_TAG) && BM_elem_flag_test(e->v2, BM_ELEM_TAG)) {
        einput_len++;
      }
    }

    /* Allocate arrays. */
    BMVert **vinput_arr = MEM_new_array_uninitialized<BMVert *>(size_t(vinput_len), __func__);
    BMEdge **einput_arr = MEM_new_array_uninitialized<BMEdge *>(size_t(einput_len), __func__);

    /* Fill arrays. */
    int v_out_index = 0;
    int e_out_index = 0;

    BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
      if (BM_elem_flag_test(v, BM_ELEM_TAG)) {
        vinput_arr[v_out_index++] = v;
      }
    }

    BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
      if (BM_elem_flag_test(e->v1, BM_ELEM_TAG) && BM_elem_flag_test(e->v2, BM_ELEM_TAG)) {
        einput_arr[e_out_index++] = e;
      }
    }

    /* Perform planar dissolve on selected elements. */
    BM_mesh_decimate_dissolve_ex(bm,
                                 angle_limit,
                                 dissolve_boundaries,
                                 BMO_Delimit(delimit),
                                 vinput_arr,
                                 vinput_len,
                                 einput_arr,
                                 einput_len,
                                 0);

    MEM_delete(vinput_arr);
    MEM_delete(einput_arr);
  }
  else {
    /* No selection: process all vertices and edges. */
    BM_mesh_decimate_dissolve(bm, angle_limit, dissolve_boundaries, BMO_Delimit(delimit));
  }

  /* Convert back to Mesh. */
  Mesh *result = BKE_mesh_from_bmesh_for_eval_nomain(bm, nullptr, &mesh);
  BM_mesh_free(bm);

  return result;
}

}  // namespace blender::geometry
