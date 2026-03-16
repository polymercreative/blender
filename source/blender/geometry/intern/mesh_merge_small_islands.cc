/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup geo
 *
 * Merge small attribute islands into neighboring large islands.
 * Useful for cleaning up mesh segmentation and attribute regions.
 */

#include "GEO_mesh_merge_small_islands.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_legacy_convert.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"

#include "BLI_atomic_disjoint_set.hh"
#include "BLI_color.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_task.hh"
#include "BLI_vector.hh"

namespace blender::geometry {

/**
 * Build face-to-face adjacency map via shared edges.
 */
static void build_face_adjacency(const Span<int> /*corner_verts*/,
                                 const Span<int> corner_edges,
                                 const OffsetIndices<int> faces,
                                 const int edges_num,
                                 Array<Vector<int>> &r_face_neighbors)
{
  /* Map each edge to the faces that use it. */
  Array<Vector<int, 2>> edge_to_faces(edges_num);
  for (const int face_i : faces.index_range()) {
    for (const int edge : corner_edges.slice(faces[face_i])) {
      edge_to_faces[edge].append(face_i);
    }
  }

  /* Build face neighbor lists. */
  r_face_neighbors.reinitialize(faces.size());
  threading::parallel_for(faces.index_range(), 512, [&](const IndexRange range) {
    for (const int face_i : range) {
      for (const int edge : corner_edges.slice(faces[face_i])) {
        for (const int neighbor_face : edge_to_faces[edge]) {
          if (neighbor_face != face_i) {
            r_face_neighbors[face_i].append_non_duplicates(neighbor_face);
          }
        }
      }
    }
  });
}

/**
 * Find islands of faces with matching attribute values using union-find.
 * Returns island IDs for each face and the total count of islands.
 */
template<typename T>
static void find_attribute_islands(const Span<T> attribute_values,
                                   const Array<Vector<int>> &face_neighbors,
                                   Array<int> &r_island_ids,
                                   int &r_island_count)
{
  const int faces_num = attribute_values.size();
  AtomicDisjointSet islands(faces_num);

  /* Join faces that are adjacent and have the same attribute value. */
  threading::parallel_for(IndexRange(faces_num), 512, [&](const IndexRange range) {
    for (const int face_i : range) {
      const T &value = attribute_values[face_i];
      for (const int neighbor_face : face_neighbors[face_i]) {
        if (attribute_values[neighbor_face] == value) {
          islands.join(face_i, neighbor_face);
        }
      }
    }
  });

  /* Get reduced island IDs. */
  r_island_ids.reinitialize(faces_num);
  islands.calc_reduced_ids(r_island_ids);
  r_island_count = islands.count_sets();
}

/**
 * Count the size (number of faces) of each island.
 */
static void count_island_sizes(const Span<int> island_ids,
                               const int island_count,
                               Array<int> &r_island_sizes)
{
  r_island_sizes.reinitialize(island_count);
  r_island_sizes.fill(0);

  for (const int island_id : island_ids) {
    r_island_sizes[island_id]++;
  }
}

/**
 * Propagate attribute values from large islands to neighboring small islands.
 * Each small island inherits from its largest neighboring large island.
 */
template<typename T>
static void propagate_to_small_islands(MutableSpan<T> attribute_values,
                                       const Span<int> island_ids,
                                       const Span<int> island_sizes,
                                       const Array<Vector<int>> &face_neighbors,
                                       const int threshold)
{
  const int faces_num = attribute_values.size();
  const int island_count = island_sizes.size();

  /* Build map: island_id -> faces in that island. */
  Array<Vector<int>> island_faces(island_count);
  for (int face_i = 0; face_i < faces_num; face_i++) {
    island_faces[island_ids[face_i]].append(face_i);
  }

  /* Process each small island to find and inherit from largest neighbor. */
  threading::parallel_for(IndexRange(island_count), 64, [&](const IndexRange range) {
    for (const int island_id : range) {
      if (island_sizes[island_id] >= threshold) {
        continue; /* Skip large islands. */
      }

      /* Find the largest neighboring large island. */
      T largest_neighbor_value;
      int largest_neighbor_size = 0;
      bool found_large_neighbor = false;

      for (const int face_i : island_faces[island_id]) {
        for (const int neighbor_face : face_neighbors[face_i]) {
          const int neighbor_island = island_ids[neighbor_face];
          const int neighbor_size = island_sizes[neighbor_island];

          if (neighbor_size >= threshold && neighbor_size > largest_neighbor_size) {
            largest_neighbor_size = neighbor_size;
            largest_neighbor_value = attribute_values[neighbor_face];
            found_large_neighbor = true;
          }
        }
      }

      if (!found_large_neighbor) {
        continue; /* No large neighbors found - keep original values. */
      }

      /* Apply largest neighbor's value to all faces in this small island. */
      for (const int face_i : island_faces[island_id]) {
        attribute_values[face_i] = largest_neighbor_value;
      }
    }
  });
}

/**
 * Type-specific processing for different attribute types.
 */
template<typename T>
static void process_typed_attribute(Mesh &result_mesh,
                                    const bke::AttributeAccessor attributes,
                                    StringRef attribute_name,
                                    const Array<Vector<int>> &face_neighbors,
                                    const int threshold)
{
  /* Read attribute values. */
  const VArraySpan<T> src_values = *attributes.lookup<T>(attribute_name, bke::AttrDomain::Face);
  if (src_values.is_empty()) {
    return;
  }

  /* Find islands of same attribute values. */
  Array<int> island_ids;
  int island_count = 0;
  find_attribute_islands(src_values, face_neighbors, island_ids, island_count);

  /* Count island sizes. */
  Array<int> island_sizes;
  count_island_sizes(island_ids, island_count, island_sizes);

  /* Get writable attribute. */
  bke::MutableAttributeAccessor result_attributes = result_mesh.attributes_for_write();
  bke::SpanAttributeWriter<T> dst_values = result_attributes.lookup_for_write_span<T>(
      attribute_name);
  if (!dst_values) {
    return;
  }

  /* Copy source values to destination. */
  dst_values.span.copy_from(src_values);

  /* Propagate from large to small islands. */
  propagate_to_small_islands(
      dst_values.span, island_ids, island_sizes, face_neighbors, threshold);

  dst_values.finish();
}

Mesh *mesh_merge_small_attribute_islands(const Mesh &mesh,
                                         const StringRef attribute_name,
                                         const int threshold)
{
  if (threshold <= 0) {
    /* No merging needed. */
    return nullptr;
  }

  const bke::AttributeAccessor attributes = mesh.attributes();
  const std::optional<bke::AttributeMetaData> meta_data = attributes.lookup_meta_data(
      attribute_name);

  if (!meta_data || meta_data->domain != bke::AttrDomain::Face) {
    /* Attribute doesn't exist or isn't on face domain. */
    return nullptr;
  }

  /* Build face adjacency. */
  const Span<int> corner_verts = mesh.corner_verts();
  const Span<int> corner_edges = mesh.corner_edges();
  const OffsetIndices faces = mesh.faces();
  Array<Vector<int>> face_neighbors;
  build_face_adjacency(corner_verts, corner_edges, faces, mesh.edges_num, face_neighbors);

  /* Create result mesh (copy input). */
  Mesh *result = BKE_mesh_copy_for_eval(mesh);

  /* Process based on attribute type. */
  const std::optional<eCustomDataType> custom_data_type = blender::bke::attr_type_to_custom_data_type(
      meta_data->data_type);
  if (!custom_data_type) {
    BKE_id_free(nullptr, result);
    return nullptr;
  }

  switch (*custom_data_type) {
    case CD_PROP_INT32:
      process_typed_attribute<int>(*result, attributes, attribute_name, face_neighbors, threshold);
      break;
    case CD_PROP_FLOAT:
      process_typed_attribute<float>(
          *result, attributes, attribute_name, face_neighbors, threshold);
      break;
    case CD_PROP_FLOAT3:
      process_typed_attribute<float3>(
          *result, attributes, attribute_name, face_neighbors, threshold);
      break;
    case CD_PROP_COLOR:
      process_typed_attribute<ColorGeometry4f>(
          *result, attributes, attribute_name, face_neighbors, threshold);
      break;
    case CD_PROP_BYTE_COLOR:
      process_typed_attribute<ColorGeometry4b>(
          *result, attributes, attribute_name, face_neighbors, threshold);
      break;
    default:
      /* Unsupported attribute type. */
      BKE_id_free(nullptr, result);
      return nullptr;
  }

  return result;
}

}  // namespace blender::geometry
