/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_mesh_merge_similar_islands.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_legacy_convert.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"

#include "BLI_array.hh"
#include "BLI_atomic_disjoint_set.hh"
#include "BLI_color.hh"
#include "BLI_map.hh"
#include "BLI_math_vector.hh"
#include "BLI_set.hh"
#include "BLI_task.hh"
#include "BLI_vector.hh"

#include "DNA_mesh_types.h"

namespace blender::geometry {

/**
 * Build face-to-face adjacency based on shared edges.
 */
static Array<Vector<int>> build_face_adjacency(const Mesh &mesh)
{
  const Span<int2> edges = mesh.edges();
  const OffsetIndices<int> faces = mesh.faces();
  const Span<int> corner_edges = mesh.corner_edges();

  const int faces_num = mesh.faces_num;

  /* Build edge-to-faces map. */
  Map<int, Vector<int>> edge_to_faces;
  for (const int face_i : IndexRange(faces_num)) {
    for (const int corner : faces[face_i]) {
      const int edge_i = corner_edges[corner];
      edge_to_faces.lookup_or_add_default(edge_i).append(face_i);
    }
  }

  /* Build face-to-face adjacency. */
  Array<Vector<int>> face_neighbors(faces_num);
  for (const auto &item : edge_to_faces.items()) {
    const Vector<int> &faces_on_edge = item.value;
    if (faces_on_edge.size() == 2) {
      const int face_a = faces_on_edge[0];
      const int face_b = faces_on_edge[1];
      face_neighbors[face_a].append(face_b);
      face_neighbors[face_b].append(face_a);
    }
  }

  return face_neighbors;
}

/**
 * Find islands of contiguous faces with the same attribute value.
 * Returns island ID for each face.
 */
template<typename T>
static Array<int> find_attribute_islands(const Span<T> attribute_values,
                                         const Array<Vector<int>> &face_neighbors,
                                         int &r_island_count)
{
  const int faces_num = attribute_values.size();
  AtomicDisjointSet disjoint_set(faces_num);

  /* Union faces with same attribute value that are adjacent. */
  threading::parallel_for(IndexRange(faces_num), 512, [&](const IndexRange range) {
    for (const int face_i : range) {
      const T &value = attribute_values[face_i];
      for (const int neighbor_face : face_neighbors[face_i]) {
        if (attribute_values[neighbor_face] == value) {
          disjoint_set.join(face_i, neighbor_face);
        }
      }
    }
  });

  /* Convert to sequential island IDs. */
  Array<int> island_ids(faces_num);
  Map<int, int> root_to_island;
  int island_count = 0;

  for (int face_i = 0; face_i < faces_num; face_i++) {
    const int root = disjoint_set.find_root(face_i);
    const int island_id = root_to_island.lookup_or_add_cb(root, [&]() { return island_count++; });
    island_ids[face_i] = island_id;
  }

  r_island_count = island_count;
  return island_ids;
}

/**
 * Calculate difference between two values based on type.
 */
static float calculate_difference(const int a, const int b, VectorDifferenceMode /*vector_mode*/)
{
  return float(std::abs(a - b));
}

static float calculate_difference(const float a,
                                  const float b,
                                  VectorDifferenceMode /*vector_mode*/)
{
  return std::abs(a - b);
}

static float calculate_difference(const float3 &a,
                                  const float3 &b,
                                  VectorDifferenceMode vector_mode)
{
  if (vector_mode == VectorDifferenceMode::Angular) {
    const float len_a = math::length(a);
    const float len_b = math::length(b);
    if (len_a < 1e-6f || len_b < 1e-6f) {
      return 0.0f; /* Degenerate vectors treated as similar. */
    }
    const float3 norm_a = a / len_a;
    const float3 norm_b = b / len_b;
    const float dot = math::clamp(math::dot(norm_a, norm_b), -1.0f, 1.0f);
    return 1.0f - dot; /* 0 = identical direction, 2 = opposite. */
  }
  /* Distance mode. */
  return math::length(a - b);
}

static float calculate_difference(const ColorGeometry4f &a,
                                  const ColorGeometry4f &b,
                                  VectorDifferenceMode /*vector_mode*/)
{
  const float dr = a.r - b.r;
  const float dg = a.g - b.g;
  const float db = a.b - b.b;
  const float da = a.a - b.a;
  return std::sqrt(dr * dr + dg * dg + db * db + da * da);
}

static float calculate_difference(const ColorGeometry4b &a,
                                  const ColorGeometry4b &b,
                                  VectorDifferenceMode /*vector_mode*/)
{
  const float dr = (float(a.r) - float(b.r)) / 255.0f;
  const float dg = (float(a.g) - float(b.g)) / 255.0f;
  const float db = (float(a.b) - float(b.b)) / 255.0f;
  const float da = (float(a.a) - float(b.a)) / 255.0f;
  return std::sqrt(dr * dr + dg * dg + db * db + da * da);
}

/**
 * Average two values (for merge mode).
 */
static int average_values(const int a, const int b)
{
  return (a + b + 1) / 2; /* Round to nearest. */
}

static float average_values(const float a, const float b)
{
  return (a + b) * 0.5f;
}

static float3 average_values(const float3 &a, const float3 &b)
{
  return (a + b) * 0.5f;
}

static ColorGeometry4f average_values(const ColorGeometry4f &a, const ColorGeometry4f &b)
{
  return ColorGeometry4f(
      (a.r + b.r) * 0.5f, (a.g + b.g) * 0.5f, (a.b + b.b) * 0.5f, (a.a + b.a) * 0.5f);
}

static ColorGeometry4b average_values(const ColorGeometry4b &a, const ColorGeometry4b &b)
{
  return ColorGeometry4b(uint8_t((int(a.r) + int(b.r)) / 2),
                         uint8_t((int(a.g) + int(b.g)) / 2),
                         uint8_t((int(a.b) + int(b.b)) / 2),
                         uint8_t((int(a.a) + int(b.a)) / 2));
}

/**
 * Build island adjacency and find pairs that should merge.
 */
template<typename T>
static void merge_similar_islands(MutableSpan<T> attribute_values,
                                  const Array<Vector<int>> &face_neighbors,
                                  const float threshold,
                                  const MergeSimilarMode merge_mode,
                                  const VectorDifferenceMode vector_mode,
                                  const int max_iterations)
{
  const int faces_num = attribute_values.size();
  bool changed = true;
  int iteration = 0;

  /* Iterate until no more merges occur or max iterations reached. */
  while (changed && (max_iterations == 0 || iteration < max_iterations)) {
    changed = false;
    iteration++;

    /* Find current islands. */
    int island_count = 0;
    Array<int> island_ids = find_attribute_islands<T>(
        attribute_values.as_span(), face_neighbors, island_count);

    if (island_count <= 1) {
      break;
    }

    /* Build island data: representative value and size. */
    Array<T> island_values(island_count);
    Array<int> island_sizes(island_count, 0);
    Array<bool> island_value_set(island_count, false);

    for (int face_i = 0; face_i < faces_num; face_i++) {
      const int island_id = island_ids[face_i];
      island_sizes[island_id]++;
      if (!island_value_set[island_id]) {
        island_values[island_id] = attribute_values[face_i];
        island_value_set[island_id] = true;
      }
    }

    /* Find island adjacencies and compute differences. */
    Set<std::pair<int, int>> checked_pairs;
    Map<std::pair<int, int>, float> island_differences;

    for (int face_i = 0; face_i < faces_num; face_i++) {
      const int island_a = island_ids[face_i];
      for (const int neighbor_face : face_neighbors[face_i]) {
        const int island_b = island_ids[neighbor_face];
        if (island_a == island_b) {
          continue;
        }

        /* Normalize pair order for deduplication. */
        const auto pair = island_a < island_b ? std::make_pair(island_a, island_b) :
                                                std::make_pair(island_b, island_a);

        if (checked_pairs.contains(pair)) {
          continue;
        }
        checked_pairs.add(pair);

        const float diff = calculate_difference(
            island_values[island_a], island_values[island_b], vector_mode);
        island_differences.add(pair, diff);
      }
    }

    /* Find pairs to merge (difference < threshold). */
    AtomicDisjointSet island_unions(island_count);
    Array<T> merged_values(island_count);
    for (int i = 0; i < island_count; i++) {
      merged_values[i] = island_values[i];
    }

    for (const auto &item : island_differences.items()) {
      if (item.value < threshold) {
        const int island_a = item.key.first;
        const int island_b = item.key.second;

        /* Determine merged value based on mode. */
        T new_value;
        if (merge_mode == MergeSimilarMode::Largest) {
          new_value = island_sizes[island_a] >= island_sizes[island_b] ? island_values[island_a] :
                                                                         island_values[island_b];
        }
        else {
          new_value = average_values(island_values[island_a], island_values[island_b]);
        }

        island_unions.join(island_a, island_b);

        /* Store merged value at the root. */
        const int root = island_unions.find_root(island_a);
        merged_values[root] = new_value;

        changed = true;
      }
    }

    if (!changed) {
      break;
    }

    /* Apply merged values to all faces. */
    for (int face_i = 0; face_i < faces_num; face_i++) {
      const int island_id = island_ids[face_i];
      const int root = island_unions.find_root(island_id);
      attribute_values[face_i] = merged_values[root];
    }
  }
}

/**
 * Process attribute based on its type.
 */
static bool process_typed_attribute(Mesh &mesh,
                                    const StringRef attribute_name,
                                    const Array<Vector<int>> &face_neighbors,
                                    const float threshold,
                                    const MergeSimilarMode merge_mode,
                                    const VectorDifferenceMode vector_mode,
                                    const int max_iterations)
{
  using namespace bke;

  MutableAttributeAccessor attributes = mesh.attributes_for_write();
  const std::optional<AttributeMetaData> meta_data = attributes.lookup_meta_data(attribute_name);

  if (!meta_data) {
    return false;
  }

  if (meta_data->domain != AttrDomain::Face) {
    return false;
  }

  const std::optional<eCustomDataType> custom_data_type =
      blender::bke::attr_type_to_custom_data_type(meta_data->data_type);

  if (!custom_data_type) {
    return false;
  }

  switch (*custom_data_type) {
    case CD_PROP_INT32: {
      GSpanAttributeWriter attr = attributes.lookup_for_write_span(attribute_name);
      if (attr) {
        merge_similar_islands<int>(
            attr.span.typed<int>(), face_neighbors, threshold, merge_mode, vector_mode, max_iterations);
        attr.finish();
        return true;
      }
      break;
    }
    case CD_PROP_FLOAT: {
      GSpanAttributeWriter attr = attributes.lookup_for_write_span(attribute_name);
      if (attr) {
        merge_similar_islands<float>(
            attr.span.typed<float>(), face_neighbors, threshold, merge_mode, vector_mode, max_iterations);
        attr.finish();
        return true;
      }
      break;
    }
    case CD_PROP_FLOAT3: {
      GSpanAttributeWriter attr = attributes.lookup_for_write_span(attribute_name);
      if (attr) {
        merge_similar_islands<float3>(
            attr.span.typed<float3>(), face_neighbors, threshold, merge_mode, vector_mode, max_iterations);
        attr.finish();
        return true;
      }
      break;
    }
    case CD_PROP_COLOR: {
      GSpanAttributeWriter attr = attributes.lookup_for_write_span(attribute_name);
      if (attr) {
        merge_similar_islands<ColorGeometry4f>(attr.span.typed<ColorGeometry4f>(),
                                               face_neighbors,
                                               threshold,
                                               merge_mode,
                                               vector_mode,
                                               max_iterations);
        attr.finish();
        return true;
      }
      break;
    }
    case CD_PROP_BYTE_COLOR: {
      GSpanAttributeWriter attr = attributes.lookup_for_write_span(attribute_name);
      if (attr) {
        merge_similar_islands<ColorGeometry4b>(attr.span.typed<ColorGeometry4b>(),
                                               face_neighbors,
                                               threshold,
                                               merge_mode,
                                               vector_mode,
                                               max_iterations);
        attr.finish();
        return true;
      }
      break;
    }
    default:
      break;
  }

  return false;
}

Mesh *mesh_merge_similar_attribute_islands(const Mesh &mesh,
                                           const StringRef attribute_name,
                                           const float threshold,
                                           const MergeSimilarMode merge_mode,
                                           const VectorDifferenceMode vector_mode,
                                           const int iterations)
{
  /* Verify attribute exists and is on face domain. */
  const bke::AttributeAccessor attributes = mesh.attributes();
  const std::optional<bke::AttributeMetaData> meta_data = attributes.lookup_meta_data(
      attribute_name);

  if (!meta_data) {
    return nullptr;
  }

  if (meta_data->domain != bke::AttrDomain::Face) {
    return nullptr;
  }

  /* Copy mesh for modification. */
  Mesh *result = BKE_mesh_copy_for_eval(mesh);

  /* Build face adjacency. */
  const Array<Vector<int>> face_neighbors = build_face_adjacency(*result);

  /* Process the attribute. */
  if (!process_typed_attribute(
          *result, attribute_name, face_neighbors, threshold, merge_mode, vector_mode, iterations))
  {
    BKE_id_free(nullptr, result);
    return nullptr;
  }

  return result;
}

}  // namespace blender::geometry
