/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Hard Bevel Node - Straight skeleton style beveling with collision detection.
 *
 * This is a C++ port of the Hard Bevel addon by Kushiro (GPL licensed).
 * The algorithm works by:
 * 1. Doing a tiny bevel (0.001) to create the topology
 * 2. For each face adjacent to beveled edges, projecting to 2D
 * 3. Computing collision-aware vertex positions using straight skeleton math
 * 4. Applying the new positions back to the mesh
 */

#include "MEM_guardedalloc.h"

#include "BKE_mesh.hh"

#include "GEO_foreach_geometry.hh"

#include "DNA_modifier_types.h"
#include "DNA_node_types.h"

#include "NOD_rna_define.hh"

#include "RNA_access.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "BLI_array.hh"
#include "BLI_math_geom.h"
#include "BLI_math_matrix.h"
#include "BLI_math_vector.h"
#include "BLI_vector.hh"

#include "bmesh.hh"
#include "bmesh_tools.hh"

#include "node_geometry_util.hh"

#include <cmath>

namespace blender::nodes::node_geo_hard_bevel_cc {

/* -------------------------------------------------------------------- */
/** \name Hard Bevel Data Structures
 * \{ */

/**
 * Virtual loop structure for 2D face processing.
 * Mirrors the Python Loop class from Hard Bevel.
 */
struct HBLoop {
  float3 co;           /* Vertex coordinate (in 2D face space, z=0) */
  BMVert *real_vert;   /* Pointer to actual BMVert */
  float deg;           /* sin(angle/2) - movement speed factor */
  int index;           /* Index in face */
  HBLoop *link_loop_next; /* Next loop in face */
  HBLoop *link_loop_prev; /* Previous loop in face */
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Utility Functions
 * \{ */

/**
 * Build transformation matrix to project 3D face to 2D XY plane.
 * Returns the matrix that transforms FROM 3D world TO 2D face-local.
 * (This matches get_matrix_face in Python which returns mat.inverted())
 */
static float4x4 get_matrix_face(BMFace *f)
{
  BMLoop *l_first = BM_FACE_FIRST_LOOP(f);

  /* Find first non-zero edge direction */
  float3 m1(0.0f);
  BMLoop *l_iter = l_first;
  do {
    m1 = float3(l_iter->next->v->co) - float3(l_iter->v->co);
    if (math::length(m1) > 0.0f) {
      break;
    }
    l_iter = l_iter->next;
  } while (l_iter != l_first);

  if (math::length(m1) < 1e-8f) {
    return float4x4::identity();
  }

  float3 normal(f->no);
  float3 m2 = math::cross(normal, m1);
  float3 center(l_first->v->co);

  /* Normalize */
  m1 = math::normalize(m1);
  m2 = math::normalize(m2);
  normal = math::normalize(normal);

  /* Build 4x4 transformation matrix.
   *
   * In Blender's float4x4, mat[i] accesses COLUMN i (column-major storage).
   * Python builds rows then transposes. We build columns directly (post-transpose form):
   * - Column 0 = X axis (m1)
   * - Column 1 = Y axis (m2)
   * - Column 2 = Z axis (normal)
   * - Column 3 = Translation (center)
   *
   * This creates a matrix that transforms FROM local face space TO world space.
   * We return the inverse, which transforms FROM world TO local (3D to 2D).
   */
  float4x4 mat;
  mat[0] = float4(m1.x, m1.y, m1.z, 0.0f);
  mat[1] = float4(m2.x, m2.y, m2.z, 0.0f);
  mat[2] = float4(normal.x, normal.y, normal.z, 0.0f);
  mat[3] = float4(center.x, center.y, center.z, 1.0f);

  return math::invert(mat);
}

/**
 * Link loops in circular list (matching Python link_loops).
 */
static void link_loops(Array<HBLoop> &ps)
{
  const int n = ps.size();
  for (int i = 0; i < n; i++) {
    ps[i].link_loop_next = &ps[(i + 1) % n];
    ps[(i + 1) % n].link_loop_prev = &ps[i];
  }
}

/**
 * Calculate angle at vertex p1 between its two adjacent edges.
 */
static float calc_angle(const HBLoop &p1)
{
  const HBLoop *p2 = p1.link_loop_next;
  const HBLoop *p3 = p1.link_loop_prev;

  float3 m1 = p2->co - p1.co;
  float3 m2 = p3->co - p1.co;

  float len1 = math::length(m1);
  float len2 = math::length(m2);

  if (len1 < 1e-8f || len2 < 1e-8f) {
    return 0.0f;
  }

  m1 = m1 / len1;
  m2 = m2 / len2;

  float dot = std::clamp(math::dot(m1, m2), -1.0f, 1.0f);
  return acosf(dot);
}

/**
 * Get the bisector direction (inward pointing) for a vertex.
 * This is get_mid from Python.
 */
static float3 get_mid(const float3 &m1, const float3 &m2, const float3 &sn)
{
  float3 c1 = math::cross(m1, sn) * -1.0f;
  float3 c2 = math::cross(m2, sn);

  float len1 = math::length(c1);
  float len2 = math::length(c2);

  if (len1 > 1e-8f) c1 = c1 / len1;
  if (len2 > 1e-8f) c2 = c2 / len2;

  float3 c3 = c1 + c2;
  float len3 = math::length(c3);

  if (len3 > 1e-8f) {
    return c3 / len3;
  }
  return float3(0.0f);
}

/**
 * Get bisector direction for a loop vertex (get_ps_mid from Python).
 */
static float3 get_ps_mid(const HBLoop &p1, const float3 &sn)
{
  const HBLoop *p2 = p1.link_loop_next;
  const HBLoop *p3 = p1.link_loop_prev;
  float3 m1 = p2->co - p1.co;
  float3 m2 = p3->co - p1.co;
  return get_mid(m1, m2, sn);
}

/**
 * 2D line-line intersection (infinite lines, not segments).
 * Returns intersection point or nullopt if parallel.
 * This matches Python's intersect2d which uses intersect_line_line_2d.
 */
static std::optional<float3> intersect_2d(const float3 &p1, const float3 &p2,
                                          const float3 &p3, const float3 &p4)
{
  float2 a1(p1.x, p1.y);
  float2 a2(p2.x, p2.y);
  float2 b1(p3.x, p3.y);
  float2 b2(p4.x, p4.y);

  float result[2];
  int isect = isect_seg_seg_v2_point(a1, a2, b1, b2, result);

  if (isect >= 1) {
    return float3(result[0], result[1], 0.0f);
  }
  return std::nullopt;
}

/**
 * Point-in-polygon test using winding number algorithm.
 * Matches Python's is_inside() function (line 985-997).
 */
static bool is_inside(const float3 &point, const Vector<float3> &polygon)
{
  int wn = 0;
  const int n = polygon.size();

  for (int i = 0; i < n; i++) {
    int next = (i + 1) % n;
    if (polygon[i].y <= point.y) {
      if (polygon[next].y > point.y) {
        /* Upward crossing */
        float is_left = (polygon[next].x - polygon[i].x) * (point.y - polygon[i].y) -
                        (point.x - polygon[i].x) * (polygon[next].y - polygon[i].y);
        if (is_left > 0) {
          wn++;
        }
      }
    }
    else {
      if (polygon[next].y <= point.y) {
        /* Downward crossing */
        float is_left = (polygon[next].x - polygon[i].x) * (point.y - polygon[i].y) -
                        (point.x - polygon[i].x) * (polygon[next].y - polygon[i].y);
        if (is_left < 0) {
          wn--;
        }
      }
    }
  }
  return wn > 0;
}

/**
 * Check if points are colinear.
 * Matches Python's is_colinear() function.
 */
static bool is_colinear(const float3 &p1, const float3 &p2, const float3 &p3)
{
  float3 m1 = p2 - p1;
  float3 m2 = p3 - p1;
  float len1 = math::length(m1);
  float len2 = math::length(m2);
  if (len1 < 1e-8f || len2 < 1e-8f) {
    return true;
  }
  float d1 = math::dot(m1 / len1, m2 / len2);
  return fabsf(d1) > 0.99f;
}

/**
 * Safe sqrt that handles negative values.
 * Matches Python's sqrt() function.
 */
static float safe_sqrt(float x)
{
  return sqrtf(fabsf(x));
}

/**
 * Compute collision time when two vertices move along their bisectors
 * and the edge between them would hit a stationary point.
 * Matches Python's get_est() function (line 508-520).
 *
 * This is a complex algebraic formula that solves for when a moving edge
 * (defined by two vertices moving at different speeds along different directions)
 * passes through a point.
 */
static std::optional<float> get_est(const float3 &a, const float3 &b,
                                    const float3 &m1, const float3 &m2,
                                    const float3 &k)
{
  float m1_x = m1.x, m1_y = m1.y;
  float m2_x = m2.x, m2_y = m2.y;
  float x_a = a.x, y_a = a.y;
  float x_b = b.x, y_b = b.y;
  float x_k = k.x, y_k = k.y;

  float div = m1_y * m2_x - m1_x * m2_y;
  if (fabsf(div) < 1e-10f) {
    return std::nullopt;
  }

  /* This massive formula comes from solving the system of equations for when
   * the line from (a + m1*t) to (b + m2*t) passes through point k */
  float sqrt_term = safe_sqrt(
      m2_y * m2_y * x_a * x_a - 2 * m1_y * m2_y * x_a * x_b + m1_y * m1_y * x_b * x_b +
      m2_x * m2_x * y_a * y_a + m1_x * m1_x * y_b * y_b +
      (m1_y * m1_y - 2 * m1_y * m2_y + m2_y * m2_y) * x_k * x_k +
      (m1_x * m1_x - 2 * m1_x * m2_x + m2_x * m2_x) * y_k * y_k +
      2 * ((m1_y * m2_y - m2_y * m2_y) * x_a - (m1_y * m1_y - m1_y * m2_y) * x_b) * x_k -
      2 * (m2_x * m2_y * x_a + (m1_y * m2_x - 2 * m1_x * m2_y) * x_b -
           (m1_y * m2_x - (2 * m1_x - m2_x) * m2_y) * x_k) * y_a -
      2 * (m1_x * m1_y * x_b + m1_x * m2_x * y_a - (2 * m1_y * m2_x - m1_x * m2_y) * x_a -
           (m1_x * m1_y - 2 * m1_y * m2_x + m1_x * m2_y) * x_k) * y_b -
      2 * ((2 * m1_y * m2_x - (m1_x + m2_x) * m2_y) * x_a -
           (m1_x * m1_y + m1_y * m2_x - 2 * m1_x * m2_y) * x_b +
           (m1_x * m1_y - m1_y * m2_x - (m1_x - m2_x) * m2_y) * x_k -
           (m1_x * m2_x - m2_x * m2_x) * y_a + (m1_x * m1_x - m1_x * m2_x) * y_b) * y_k);

  float t = 0.5f * (m2_y * x_a - m1_y * x_b + (m1_y - m2_y) * x_k -
                    m2_x * y_a + m1_x * y_b - (m1_x - m2_x) * y_k + sqrt_term) / div;

  if (t < 0) {
    return std::nullopt;
  }
  return t;
}

/**
 * Simplified version for when both vertices move in the same direction.
 * Matches Python's get_est_same() function (line 482-491).
 */
static std::optional<float> get_est_same(const float3 &a, const float3 &b,
                                         const float3 &m1, const float3 &k)
{
  float m1_x = m1.x, m1_y = m1.y;
  float x_a = a.x, y_a = a.y;
  float x_b = b.x, y_b = b.y;
  float x_k = k.x, y_k = k.y;

  float div = m1_y * x_a - m1_y * x_b - m1_x * y_a + m1_x * y_b;
  if (fabsf(div) < 1e-10f) {
    return std::nullopt;
  }

  float t = ((x_b - x_k) * y_a - (x_a - x_k) * y_b + (x_a - x_b) * y_k) / div;
  if (t < 0) {
    return std::nullopt;
  }
  return t;
}

/**
 * Line-plane intersection.
 * Matches Python's line_plane_intersection() function (line 960-969).
 * Returns the intersection point, or nullopt if parallel or outside segment.
 */
static std::optional<float3> line_plane_intersection(const float3 &P1, const float3 &P2,
                                                     const float3 &P0, const float3 &normal)
{
  float num = math::dot(normal, P0 - P1);
  float den = math::dot(normal, P2 - P1);

  if (fabsf(den) < 1e-6f) {
    return std::nullopt;
  }

  float t = num / den;
  if (t < 0 || t > 1) {
    return std::nullopt;
  }

  return P1 + t * (P2 - P1);
}

/**
 * Get surface normal of a quad polygon.
 * Matches Python's get_sn() function (line 904-912).
 */
static float3 get_sn(const Vector<float3> &poly)
{
  if (poly.size() < 3) {
    return float3(0, 0, 1);
  }
  float3 AB = poly[1] - poly[0];
  float3 AC = poly[2] - poly[0];
  float3 n = math::cross(AB, AC);
  float len = math::length(n);
  if (len < 1e-8f) {
    return float3(0, 0, 1);
  }
  return n / len;
}

/**
 * Check if a point is near a line segment.
 * Matches Python's point_line_near() function (line 917-928).
 */
static bool point_line_near(const float3 &pin, const float3 &pe1, const float3 &pe2, float offset)
{
  float3 m1 = pe2 - pe1;
  float3 m2 = pin - pe1;
  float m1_dot = math::dot(m1, m1);
  if (m1_dot < 1e-10f) {
    return math::length(m2) < offset;
  }
  float pro = math::dot(m1, m2) / m1_dot;
  float3 pro_v = pe1 + m1 * pro;
  float3 h1 = pin - pro_v;
  return math::length(h1) < offset;
}

/**
 * Line-polygon intersection in 3D.
 * Matches Python's line_poly_intersect() function (line 932-957).
 *
 * This checks if the line from p1 to p2 intersects the polygon (which is a "wall"
 * formed by a moving edge, where Z represents time).
 */
static std::optional<float3> line_poly_intersect(const float3 &p1_co, const float3 &p2_co,
                                                  const Vector<float3> &poly,
                                                  const float3 &normal, float offset)
{
  if (poly.size() < 4) {
    return std::nullopt;
  }

  /* Compute center of polygon */
  float3 cen(0, 0, 0);
  for (const float3 &p : poly) {
    cen += p;
  }
  cen /= float(poly.size());

  /* Line-plane intersection */
  auto pin_opt = line_plane_intersection(p1_co, p2_co, cen, normal);
  if (!pin_opt.has_value()) {
    return std::nullopt;
  }
  float3 pin = *pin_opt;

  /* Expand polygon slightly for tolerance */
  Vector<float3> poly2;
  for (const float3 &p : poly) {
    float3 m1 = p - cen;
    float len = math::length(m1);
    if (len < 1e-8f) {
      poly2.append(p);
    }
    else {
      float3 m2 = m1 / len;
      poly2.append(cen + m2 * (len + offset));
    }
  }

  /* Check if near the side edges of the quad */
  bool n1 = point_line_near(pin, poly2[0], poly2[3], offset);
  bool n2 = point_line_near(pin, poly2[1], poly2[2], offset);

  if (n1 || n2) {
    return pin;
  }

  /* Check if inside polygon using 2D projection */
  if (is_inside(pin, poly2)) {
    return pin;
  }

  return std::nullopt;
}

/**
 * Get maximum bounding distance for a face (used to extend collision rays).
 */
static float get_max_bound(const Array<HBLoop> &ps)
{
  float max_dist = 0.0f;
  for (int i = 0; i < ps.size(); i++) {
    for (int j = i + 1; j < ps.size(); j++) {
      float dist = math::distance(ps[i].co, ps[j].co);
      if (dist > max_dist) {
        max_dist = dist;
      }
    }
  }
  return max_dist * 2.0f;
}

/**
 * Check if an edge (by vertex indices in the loop) is in the beveled edge set.
 */
static bool check_bound_mov(const Array<HBLoop> &ps, int i, const Set<std::pair<int, int>> &pes)
{
  int i2 = (i + 1) % int(ps.size());
  int idx1 = BM_elem_index_get(ps[i].real_vert);
  int idx2 = BM_elem_index_get(ps[i2].real_vert);

  return pes.contains({idx1, idx2}) || pes.contains({idx2, idx1});
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Fillet Functions
 * \{ */

/**
 * Wendland compact support kernel for RBF interpolation.
 * Matches Python's core_func() (line 1378-1382).
 */
static float wendland_kernel(float r, float d = 1.0f)
{
  if (r > d) {
    return 0.0f;
  }
  float rd = r / d;
  float t = 1.0f - rd;
  return t * t * t * t * (4.0f * rd + 1.0f);
}

/**
 * Gaussian kernel for RBF interpolation.
 * Matches Python's engine2() (line 1349-1350).
 */
static float gaussian_kernel(float r, float epsilon = 1.0f)
{
  return expf(-epsilon * r * r);
}

/**
 * Solve least squares system Ax = b using QR decomposition (Householder).
 * Returns solution x. This replaces numpy.linalg.lstsq.
 */
static Vector<float3> solve_lstsq(const Vector<Vector<float>> &A,
                                   const Vector<float3> &b)
{
  const int m = A.size();
  if (m == 0) {
    return {};
  }
  const int n = A[0].size();

  /* Create augmented matrix [A | b] for each component */
  Vector<Vector<float>> Ax(m), Ay(m), Az(m);
  for (int i = 0; i < m; i++) {
    Ax[i].resize(n + 1);
    Ay[i].resize(n + 1);
    Az[i].resize(n + 1);
    for (int j = 0; j < n; j++) {
      Ax[i][j] = A[i][j];
      Ay[i][j] = A[i][j];
      Az[i][j] = A[i][j];
    }
    Ax[i][n] = b[i].x;
    Ay[i][n] = b[i].y;
    Az[i][n] = b[i].z;
  }

  /* Gaussian elimination with partial pivoting for each component */
  auto solve_component = [&](Vector<Vector<float>> &aug) -> Vector<float> {
    const int rows = aug.size();
    const int cols = aug[0].size() - 1;

    for (int col = 0; col < std::min(rows, cols); col++) {
      /* Find pivot */
      int max_row = col;
      float max_val = fabsf(aug[col][col]);
      for (int row = col + 1; row < rows; row++) {
        if (fabsf(aug[row][col]) > max_val) {
          max_val = fabsf(aug[row][col]);
          max_row = row;
        }
      }

      /* Swap rows */
      if (max_row != col) {
        std::swap(aug[col], aug[max_row]);
      }

      /* Check for singular matrix */
      if (fabsf(aug[col][col]) < 1e-10f) {
        continue;
      }

      /* Eliminate below */
      for (int row = col + 1; row < rows; row++) {
        float factor = aug[row][col] / aug[col][col];
        for (int j = col; j <= cols; j++) {
          aug[row][j] -= factor * aug[col][j];
        }
      }
    }

    /* Back substitution */
    Vector<float> x(cols, 0.0f);
    for (int i = std::min(rows, cols) - 1; i >= 0; i--) {
      if (fabsf(aug[i][i]) < 1e-10f) {
        continue;
      }
      float sum = aug[i][cols];
      for (int j = i + 1; j < cols; j++) {
        sum -= aug[i][j] * x[j];
      }
      x[i] = sum / aug[i][i];
    }
    return x;
  };

  Vector<float> sol_x = solve_component(Ax);
  Vector<float> sol_y = solve_component(Ay);
  Vector<float> sol_z = solve_component(Az);

  Vector<float3> result(n);
  for (int i = 0; i < n; i++) {
    result[i] = float3(sol_x[i], sol_y[i], sol_z[i]);
  }
  return result;
}

/**
 * RBF interpolation using Wendland kernel.
 * Matches Python's move_forward() (line 1385-1408).
 *
 * Given boundary points that moved from old to new positions,
 * compute how interior points should move using RBF interpolation.
 */
static Vector<float3> move_forward_wendland(const Vector<float3> &points,
                                             const Vector<float3> &boundary_old,
                                             const Vector<float3> &boundary_new,
                                             float support_distance = 1.0f)
{
  const int n_boundary = boundary_old.size();
  const int n_points = points.size();

  if (n_boundary == 0 || n_points == 0) {
    return points;
  }

  /* Build RBF matrix */
  Vector<Vector<float>> Phi(n_boundary);
  for (int i = 0; i < n_boundary; i++) {
    Phi[i].resize(n_boundary);
    for (int j = 0; j < n_boundary; j++) {
      float r = math::distance(boundary_old[i], boundary_old[j]);
      Phi[i][j] = wendland_kernel(r, support_distance);
    }
  }

  /* Compute displacements */
  Vector<float3> D(n_boundary);
  for (int i = 0; i < n_boundary; i++) {
    D[i] = boundary_new[i] - boundary_old[i];
  }

  /* Solve for weights */
  Vector<float3> weights = solve_lstsq(Phi, D);

  /* Apply to interior points */
  Vector<float3> new_points(n_points);
  for (int i = 0; i < n_points; i++) {
    float3 displacement(0, 0, 0);
    for (int j = 0; j < n_boundary; j++) {
      float r = math::distance(boundary_old[j], points[i]);
      displacement += weights[j] * wendland_kernel(r, support_distance);
    }
    new_points[i] = points[i] + displacement;
  }

  return new_points;
}

/**
 * RBF interpolation using Gaussian kernel.
 * Matches Python's move_forward_type2() (line 1352-1374).
 */
static Vector<float3> move_forward_gaussian(const Vector<float3> &points,
                                             const Vector<float3> &boundary_old,
                                             const Vector<float3> &boundary_new,
                                             float epsilon = 1.0f)
{
  const int n_boundary = boundary_old.size();
  const int n_points = points.size();

  if (n_boundary == 0 || n_points == 0) {
    return points;
  }

  /* Build RBF matrix */
  Vector<Vector<float>> Phi(n_boundary);
  for (int i = 0; i < n_boundary; i++) {
    Phi[i].resize(n_boundary);
    for (int j = 0; j < n_boundary; j++) {
      float r = math::distance(boundary_old[i], boundary_old[j]);
      Phi[i][j] = gaussian_kernel(r, epsilon);
    }
  }

  /* Compute displacements */
  Vector<float3> D(n_boundary);
  for (int i = 0; i < n_boundary; i++) {
    D[i] = boundary_new[i] - boundary_old[i];
  }

  /* Solve for weights */
  Vector<float3> weights = solve_lstsq(Phi, D);

  /* Apply to interior points */
  Vector<float3> new_points(n_points);
  for (int i = 0; i < n_points; i++) {
    float3 displacement(0, 0, 0);
    for (int j = 0; j < n_boundary; j++) {
      float r = math::distance(boundary_old[j], points[i]);
      displacement += weights[j] * gaussian_kernel(r, epsilon);
    }
    new_points[i] = points[i] + displacement;
  }

  return new_points;
}

/**
 * 3D line-line intersection (finds closest point between two lines).
 * Matches Python's intersect3d() (line 1632-1638).
 */
static std::optional<float3> intersect_3d(const float3 &p1, const float3 &p2,
                                           const float3 &p3, const float3 &p4)
{
  float3 d1 = p2 - p1;
  float3 d2 = p4 - p3;
  float3 r = p1 - p3;

  float a = math::dot(d1, d1);
  float b = math::dot(d1, d2);
  float c = math::dot(d2, d2);
  float d = math::dot(d1, r);
  float e = math::dot(d2, r);

  float denom = a * c - b * b;
  if (fabsf(denom) < 1e-10f) {
    return std::nullopt;
  }

  float t = (b * e - c * d) / denom;
  float s = (a * e - b * d) / denom;

  float3 closest1 = p1 + d1 * t;
  float3 closest2 = p3 + d2 * s;

  return (closest1 + closest2) * 0.5f;
}

/**
 * Evaluate cubic Bezier curve at parameter t.
 */
static float3 bezier_eval(const float3 &p0, const float3 &p1,
                          const float3 &p2, const float3 &p3, float t)
{
  float u = 1.0f - t;
  float tt = t * t;
  float uu = u * u;
  float uuu = uu * u;
  float ttt = tt * t;

  return uuu * p0 + 3.0f * uu * t * p1 + 3.0f * u * tt * p2 + ttt * p3;
}

/**
 * Interpolate Bezier curve to get 'segments' intermediate points.
 * Matches Python's mathutils.geometry.interpolate_bezier().
 */
static Vector<float3> interpolate_bezier(const float3 &p0, const float3 &p1,
                                          const float3 &p2, const float3 &p3,
                                          int segments)
{
  Vector<float3> result(segments);
  for (int i = 0; i < segments; i++) {
    float t = float(i) / float(segments - 1);
    result[i] = bezier_eval(p0, p1, p2, p3, t);
  }
  return result;
}

/**
 * Get averaged normal for vertices from surrounding faces.
 * Matches Python's get_outer_normal() (line 1614-1628).
 */
static Map<BMVert *, float3> get_outer_normal(BMesh * /*bm*/, const Set<BMFace *> &faces)
{
  Map<BMVert *, Vector<float3>> vmap;

  /* Collect face normals for each vertex */
  for (BMFace *f : faces) {
    BMLoop *l_first = BM_FACE_FIRST_LOOP(f);
    BMLoop *l_iter = l_first;
    do {
      if (!vmap.contains(l_iter->v)) {
        vmap.add(l_iter->v, Vector<float3>());
      }
      vmap.lookup(l_iter->v).append(float3(f->no));
      l_iter = l_iter->next;
    } while (l_iter != l_first);
  }

  /* Average normals */
  Map<BMVert *, float3> result;
  for (auto item : vmap.items()) {
    float3 avg(0, 0, 0);
    for (const float3 &n : item.value) {
      avg += n;
    }
    avg /= float(item.value.size());
    float len = math::length(avg);
    if (len > 1e-8f) {
      avg /= len;
    }
    result.add(item.key, avg);
  }

  return result;
}

/**
 * Check if two vectors are colinear (pointing same direction).
 */
static bool colinear_vecs(const float3 &v1, const float3 &v2)
{
  float3 m1 = math::normalize(v1);
  float3 m2 = math::normalize(v2);
  return fabsf(math::dot(m1, m2)) > 0.999f;
}

/**
 * Compute Bezier curve points for bending an edge.
 * Matches Python's get_bend_sub() (line 1211-1276).
 *
 * Given two vertices and their surface normals, compute intermediate points
 * along a Bezier curve that bends toward the intersection of the normal directions.
 */
static std::optional<Vector<float3>> get_bend_sub(BMVert *v1, BMVert *v2,
                                                   const Map<BMVert *, float3> &snmap,
                                                   int cut)
{
  if (!snmap.contains(v1) || !snmap.contains(v2)) {
    return std::nullopt;
  }

  float3 s1 = snmap.lookup(v1);
  float3 s2 = snmap.lookup(v2);

  float3 c1 = math::cross(s1, s2);
  if (math::length(c1) < 1e-8f) {
    return std::nullopt;
  }

  /* Compute tangent directions perpendicular to normals */
  float3 ss1 = math::cross(c1, s1);
  float3 ss2 = math::cross(s2, c1);

  float3 p1_co(v1->co);
  float3 p2_co(v2->co);

  if (colinear_vecs(ss1, ss2)) {
    return std::nullopt;
  }

  /* Find intersection of tangent lines */
  auto pin = intersect_3d(p1_co, p1_co + ss1, p2_co, p2_co + ss2);
  if (!pin.has_value()) {
    return std::nullopt;
  }

  /* Compute Bezier control points */
  float3 m1 = *pin - p1_co;
  float3 m2 = *pin - p2_co;

  /* Interpolate Bezier curve */
  Vector<float3> bs = interpolate_bezier(
      p1_co, p1_co + m1 * 0.5f, p2_co + m2 * 0.5f, p2_co, cut + 2);

  /* Return only the intermediate points (skip endpoints) */
  Vector<float3> result;
  for (int i = 1; i < bs.size() - 1; i++) {
    result.append(bs[i]);
  }
  return result;
}

/**
 * Structure for vertex ring in corner faces.
 * Each ring is a list of (corner_vert, subdivision_verts) pairs.
 * Matches Python's vs2 structure from get_sub_slice().
 */
using VertexRing = Vector<std::pair<BMVert *, Vector<BMVert *>>>;

/**
 * Subdivide an edge and return per-loop subdivision info.
 * Matches Python's divide_edge() (line 1687-1701).
 *
 * Returns pair of (loop1, sub1), (loop2, sub2) where sub2 is reversed.
 */
static std::optional<std::pair<std::pair<BMLoop *, Vector<BMVert *>>,
                               std::pair<BMLoop *, Vector<BMVert *>>>>
divide_edge(BMesh *bm, BMEdge *e, int cut)
{
  if (e->l == nullptr || e->l->radial_next == e->l) {
    /* Edge doesn't have exactly 2 loops */
    return std::nullopt;
  }

  BMLoop *p1 = e->l;
  BMLoop *p2 = e->l->radial_next;

  float3 m1 = float3(p2->v->co) - float3(p1->v->co);
  float3 m2 = m1 / float(cut + 1);

  Vector<BMVert *> sub;
  for (int i = 0; i < cut; i++) {
    float3 pos = float3(p1->v->co) + m2 * float(i + 1);
    BMVert *v = BM_vert_create(bm, pos, nullptr, BM_CREATE_NOP);
    sub.append(v);
  }

  /* Reverse for second loop */
  Vector<BMVert *> sub_rev;
  for (int i = sub.size() - 1; i >= 0; i--) {
    sub_rev.append(sub[i]);
  }

  return std::make_pair(std::make_pair(p1, sub), std::make_pair(p2, sub_rev));
}

/**
 * Create nested vertex rings for a corner face.
 * Matches Python's get_sub_slice() (line 1446-1480).
 *
 * ps = input ring of (corner_vert, subdivision_verts)
 * cut = number of cuts
 *
 * Returns list of rings from outer to inner.
 */
static Vector<VertexRing> get_sub_slice(BMesh *bm, const VertexRing &ps, int cut)
{
  if (cut > 1) {
    /* Compute center of corner vertices */
    float3 cen(0, 0, 0);
    for (const auto &pair : ps) {
      cen += float3(pair.first->co);
    }
    cen /= float(ps.size());

    /* Create inner corner vertices */
    Vector<BMVert *> inner;
    for (const auto &pair : ps) {
      BMVert *p1 = pair.first;
      float3 mid = float3(p1->co) - cen;
      float3 m1 = mid / float(cut + 1);
      float3 new_co = float3(p1->co) - m1 * 2.0f;
      BMVert *v1 = BM_vert_create(bm, new_co, nullptr, BM_CREATE_NOP);
      inner.append(v1);
    }

    /* Create inner ring with subdivisions between inner corners */
    VertexRing vs2;
    int inner_cut = cut - 2;
    for (int i = 0; i < inner.size(); i++) {
      int i2 = (i + 1) % inner.size();
      BMVert *p1 = inner[i];
      BMVert *p2 = inner[i2];

      Vector<BMVert *> sub;
      if (inner_cut > 0) {
        float3 m1 = float3(p2->co) - float3(p1->co);
        float3 m2 = m1 / float(inner_cut + 1);
        for (int k = 0; k < inner_cut; k++) {
          float3 pos = float3(p1->co) + m2 * float(k + 1);
          BMVert *v = BM_vert_create(bm, pos, nullptr, BM_CREATE_NOP);
          sub.append(v);
        }
      }
      vs2.append({p1, sub});
    }

    /* Recurse for deeper rings */
    Vector<VertexRing> vs3 = get_sub_slice(bm, vs2, cut - 2);

    /* Return [ps] + vs3 */
    Vector<VertexRing> result;
    result.append(ps);
    for (const VertexRing &ring : vs3) {
      result.append(ring);
    }
    return result;
  }
  else {
    return {ps};
  }
}

/**
 * Create faces connecting vertex rings.
 * Matches Python's connect_inner_face() (line 1549-1610).
 */
static Vector<BMVert *> connect_inner_face(BMesh *bm, const Vector<VertexRing> &vs2, int cut)
{
  /* First loop: Create quads at corners between adjacent rings
   * Python lines 1550-1566 */
  for (int i = 0; i < int(vs2.size()) - 1; i++) {
    int i2 = i + 1;
    const VertexRing &n1 = vs2[i];
    const VertexRing &n2 = vs2[i2];

    for (int k = 0; k < n1.size(); k++) {
      int k2 = (k + n1.size() - 1) % n1.size();
      BMVert *p1 = n1[k].first;
      const Vector<BMVert *> &sub1 = n1[k].second;
      BMVert *p2 = n2[k].first;
      const Vector<BMVert *> &sub2b = n1[k2].second;

      if (sub1.is_empty()) {
        continue;
      }
      if (sub2b.is_empty()) {
        continue;
      }

      BMVert *v1 = sub1[0];
      BMVert *v2 = sub2b.last();

      /* Create quad: p1, v1, p2, v2 */
      BMVert *verts[4] = {p1, v1, p2, v2};
      BM_face_create_verts(bm, verts, 4, nullptr, BM_CREATE_NOP, true);
    }
  }

  /* Second loop: Create quads connecting subdivisions between rings
   * Python lines 1567-1584 */
  for (int i = 0; i < int(vs2.size()) - 1; i++) {
    int i2 = i + 1;
    const VertexRing &n1 = vs2[i];
    const VertexRing &n2 = vs2[i2];

    for (int k = 0; k < n1.size(); k++) {
      int k2 = (k + 1) % n1.size();
      /* BMVert *p1 = n1[k].first; */  /* Unused, kept for symmetry. */
      const Vector<BMVert *> &sub1 = n1[k].second;
      BMVert *p2 = n2[k].first;
      const Vector<BMVert *> &sub2 = n2[k].second;
      BMVert *p2b = n2[k2].first;

      /* Build ins = [p2] + sub2 + [p2b] */
      Vector<BMVert *> ins;
      ins.append(p2);
      for (BMVert *sv : sub2) {
        ins.append(sv);
      }
      ins.append(p2b);

      for (int j = 0; j < int(sub1.size()) - 1; j++) {
        int j2 = j + 1;
        BMVert *v1 = sub1[j];
        BMVert *v2 = sub1[j2];
        BMVert *v3 = ins[j2];
        BMVert *v4 = ins[j];

        BMVert *verts[4] = {v1, v2, v3, v4};
        BM_face_create_verts(bm, verts, 4, nullptr, BM_CREATE_NOP, true);
      }
    }
  }

  /* Center handling - Python lines 1586-1610 */
  if (cut % 2 == 1) {
    /* Odd cut - create center vertex and triangular fans */
    const VertexRing &ps = vs2.last();
    Vector<BMVert *> vs;
    for (const auto &pair : ps) {
      vs.append(pair.first);
    }

    float3 cen(0, 0, 0);
    for (BMVert *v : vs) {
      cen += float3(v->co);
    }
    cen /= float(vs.size());

    BMVert *vcen = BM_vert_create(bm, cen, nullptr, BM_CREATE_NOP);

    for (int i = 0; i < ps.size(); i++) {
      int i2 = (i + ps.size() - 1) % ps.size();
      BMVert *p1 = ps[i].first;
      const Vector<BMVert *> &sub1 = ps[i].second;
      const Vector<BMVert *> &sub2 = ps[i2].second;

      if (!sub1.is_empty() && !sub2.is_empty()) {
        BMVert *v2 = sub1[0];
        BMVert *v3 = sub2.last();
        BMVert *verts[4] = {p1, v2, vcen, v3};
        BM_face_create_verts(bm, verts, 4, nullptr, BM_CREATE_NOP, true);
      }
    }
    return {vcen};
  }
  else {
    /* Even cut - create center polygon */
    const VertexRing &ps = vs2.last();
    Vector<BMVert *> vs;
    for (const auto &pair : ps) {
      vs.append(pair.first);
    }
    if (vs.size() >= 3) {
      BM_face_create_verts(bm, vs.data(), vs.size(), nullptr, BM_CREATE_NOP, true);
    }
    return {};
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Core Algorithm
 * \{ */

/**
 * Check if other vertices are "behind" the current vertex's movement.
 * Matches Python's get_ms_behind() function (line 352-419).
 *
 * This forms quadrilaterals from the movement trajectories and checks
 * if other vertices fall inside them, then computes collision times.
 */
static void get_ms_behind(int i,
                          const Array<HBLoop> &ps,
                          const Array<HBLoop> & /*pks*/,
                          Vector<float> &ms,
                          const Array<float3> &mids,
                          float plen)
{
  const int n = ps.size();
  int i2 = (i + 1) % n;
  int i3 = (i + n - 1) % n;

  const HBLoop &p1 = ps[i];
  const HBLoop &p2 = ps[i2];
  const HBLoop &p3 = ps[i3];

  if (p1.deg < 1e-8f) {
    return;
  }

  const float3 &mid1 = mids[i];
  const float3 &mid2 = mids[i2];
  const float3 &mid3 = mids[i3];

  /* Create quadrilateral corners for the forward edge trajectory */
  float3 k1 = p1.co;
  float3 k2 = p1.co + mid1 * plen;
  float3 k3 = p2.co;
  float3 k4 = p2.co + mid2 * plen;

  /* Create quadrilateral corners for the backward edge trajectory */
  float3 k5 = p3.co;
  float3 k6 = p3.co + mid3 * plen;

  /* Check each other vertex */
  for (int k = 0; k < n; k++) {
    if (k == i || k == i2 || k == i3) {
      continue;
    }

    const HBLoop &pk1 = ps[k];

    /* Check if pk1 is inside the forward edge trajectory quadrilateral */
    Vector<float3> quad1 = {k1, k3, k4, k2};
    if (is_inside(pk1.co, quad1)) {
      /* Skip if colinear */
      if (is_colinear(k1, k2, pk1.co) || is_colinear(k3, k4, pk1.co)) {
        continue;
      }

      if (p1.deg > 1e-8f && p2.deg > 1e-8f) {
        float3 dmid1 = mid1 / p1.deg;
        float3 dmid2 = mid2 / p2.deg;

        std::optional<float> d1;
        if (fabsf(math::dot(mid1, mid2) - 1.0f) < 0.01f) {
          /* Parallel movement */
          d1 = get_est_same(p1.co, p2.co, dmid1, pk1.co);
        }
        else {
          d1 = get_est(p1.co, p2.co, dmid1, dmid2, pk1.co);
        }

        if (d1.has_value()) {
          float dist = math::length(dmid1) * (*d1);
          ms.append(dist * 0.999f);
        }
      }
    }

    /* Check if pk1 is inside the backward edge trajectory quadrilateral */
    Vector<float3> quad2 = {k1, k2, k6, k5};
    if (is_inside(pk1.co, quad2)) {
      /* Skip if colinear */
      if (is_colinear(k1, k2, pk1.co) || is_colinear(k5, k6, pk1.co)) {
        continue;
      }

      if (p1.deg > 1e-8f && p3.deg > 1e-8f) {
        float3 dmid1 = mid1 / p1.deg;
        float3 dmid3 = mid3 / p3.deg;

        std::optional<float> d1;
        if (fabsf(math::dot(mid1, mid3) - 1.0f) < 0.01f) {
          /* Parallel movement */
          d1 = get_est_same(p3.co, p1.co, dmid3, pk1.co);
        }
        else {
          d1 = get_est(p3.co, p1.co, dmid3, dmid1, pk1.co);
        }

        if (d1.has_value()) {
          float dist = math::length(dmid1) * (*d1);
          ms.append(dist * 0.999f);
        }
      }
    }
  }
}

/**
 * Check collision with "walls" formed by moving edges.
 * Matches Python's get_ms_plane() function (line 651-667).
 *
 * Each edge sweeps out a 3D quad (wall) as its vertices move.
 * We check if the current vertex's trajectory pierces any wall.
 */
static void get_ms_plane(int i,
                         const Array<HBLoop> &ps,
                         const Array<HBLoop> &pks,
                         Vector<float> &ms,
                         float offset)
{
  const int n = ps.size();
  const HBLoop &p1 = ps[i];
  const HBLoop &pk = pks[i];

  for (int k = 0; k < n; k++) {
    int k2 = (k + 1) % n;
    if (k == i || k2 == i) {
      continue;
    }

    /* Build the 3D wall: original edge + phantom edge at z=maxlen */
    Vector<float3> poly = {
        ps[k].co,   /* Bottom left (z=0) */
        ps[k2].co,  /* Bottom right (z=0) */
        pks[k2].co, /* Top right (z=maxlen) */
        pks[k].co   /* Top left (z=maxlen) */
    };

    float3 normal = get_sn(poly);

    /* Check if vertex trajectory (p1 -> pk) intersects this wall */
    auto pin = line_poly_intersect(p1.co, pk.co, poly, normal, offset);
    if (pin.has_value()) {
      float3 pin_flat = *pin;
      pin_flat.z = 0;
      float d1 = math::distance(pin_flat, p1.co);
      ms.append(d1);
    }
  }
}

/**
 * Solve a single face - compute collision-aware vertex positions.
 * Returns map from BMVert to new 3D position.
 *
 * This is a faithful port of Python's solve_face() function (line 698-823).
 */
static Map<BMVert *, float3> solve_face(BMFace *face,
                                         const Set<std::pair<int, int>> &pes,
                                         float plen,
                                         float offset,
                                         const float4x4 &mat_to_2d,
                                         const float4x4 &mat_to_3d)
{
  Map<BMVert *, float3> result;

  const int face_len = face->len;
  if (face_len < 3) {
    return result;
  }

  const float3 sn(0.0f, 0.0f, 1.0f); /* Normal in 2D space */

  /* Create loop array from face (matching create_ps in Python) */
  Array<HBLoop> ps(face_len);
  BMLoop *l_first = BM_FACE_FIRST_LOOP(face);
  BMLoop *l_iter = l_first;
  int idx = 0;
  do {
    float3 co_3d(l_iter->v->co);
    float3 co_2d = math::transform_point(mat_to_2d, co_3d);
    co_2d.z = 0.0f; /* Flatten to 2D */

    ps[idx].co = co_2d;
    ps[idx].real_vert = l_iter->v;
    ps[idx].index = idx;
    ps[idx].deg = 0.0f;
    ps[idx].link_loop_next = nullptr;
    ps[idx].link_loop_prev = nullptr;

    idx++;
    l_iter = l_iter->next;
  } while (l_iter != l_first);

  /* Link loops circularly */
  link_loops(ps);

  float maxlen = get_max_bound(ps);

  /* Calculate deg (sin(angle/2)) and movement directions for each vertex.
   * Matches Python solve_face lines 704-731. */
  Array<float3> mids(face_len);
  Array<std::pair<bool, bool>> es(face_len);

  for (int i = 0; i < face_len; i++) {
    HBLoop &p1 = ps[i];
    p1.deg = sinf(calc_angle(p1) / 2.0f);

    int i3 = (i + face_len - 1) % face_len;

    bool e1 = check_bound_mov(ps, i, pes);
    bool e3 = check_bound_mov(ps, i3, pes);
    es[i] = {e1, e3};

    /* Determine movement direction based on which edges are bevel edges */
    if (e1 && e3) {
      /* Both adjacent edges are bevel edges - move along bisector */
      mids[i] = get_ps_mid(p1, sn);
    }
    else if (e1 && !e3) {
      /* Only forward edge is bevel - move toward previous vertex */
      float3 dir = p1.link_loop_prev->co - p1.co;
      float len = math::length(dir);
      mids[i] = (len > 1e-8f) ? dir / len : float3(0.0f);
    }
    else if (!e1 && e3) {
      /* Only backward edge is bevel - move toward next vertex */
      float3 dir = p1.link_loop_next->co - p1.co;
      float len = math::length(dir);
      mids[i] = (len > 1e-8f) ? dir / len : float3(0.0f);
    }
    else {
      /* Neither edge is bevel - don't move */
      mids[i] = float3(0.0f);
    }
  }

  /* Create "phantom" points at max distance along trajectories.
   * The Z coordinate represents time/distance traveled.
   * Matches Python solve_face lines 733-746. */
  Array<HBLoop> pks(face_len);
  for (int i = 0; i < face_len; i++) {
    const HBLoop &p1 = ps[i];
    float dmin = (p1.deg < 1e-8f) ? maxlen : (maxlen / p1.deg);
    float3 d1 = mids[i] * dmin;

    pks[i].co = p1.co + d1;
    pks[i].co.z = maxlen;  /* Z = time dimension */
    pks[i].real_vert = p1.real_vert;
  }

  /* Calculate final positions with collision detection.
   * Matches Python solve_face lines 748-795. */
  Array<HBLoop> ps2(face_len);

  for (int i = 0; i < face_len; i++) {
    const HBLoop &p1 = ps[i];
    const float3 &mid = mids[i];
    bool e1 = es[i].first;
    bool e3 = es[i].second;

    Vector<float> ms;

    /* Call the proper collision detection functions */
    get_ms_behind(i, ps, pks, ms, mids, plen);
    get_ms_plane(i, ps, pks, ms, offset);

    float d1;

    if (math::length(mid) < 1e-8f) {
      d1 = 0.0f;
    }
    else if (ms.is_empty()) {
      /* No collisions - use full offset */
      if (p1.deg < 1e-8f) {
        d1 = 0.0f;
      }
      else if (e1 && e3) {
        d1 = plen / p1.deg;
      }
      else {
        float deg = calc_angle(p1);
        float sin_deg = sinf(deg);
        d1 = (sin_deg > 1e-8f) ? (plen / sin_deg) : plen;
      }
    }
    else {
      /* Take minimum of collision distance and requested offset */
      float dlen = *std::min_element(ms.begin(), ms.end());

      float d2;
      if (p1.deg < 1e-8f) {
        d2 = 0.0f;
      }
      else if (e1 && e3) {
        d2 = plen / p1.deg;
      }
      else {
        float deg = calc_angle(p1);
        float sin_deg = sinf(deg);
        d2 = (sin_deg > 1e-8f) ? (plen / sin_deg) : plen;
      }
      d1 = std::min(dlen, d2);
    }

    /* Apply offset factor (1.0 - offset) like Python does at line 792 */
    float off2 = std::max(1.0f - offset, 0.0f);
    ps2[i].co = p1.co + mid * d1 * off2;
    ps2[i].real_vert = p1.real_vert;
  }

  /* Second pass: check for remaining intersections.
   * Matches Python solve_face lines 797-820.
   * IMPORTANT: Store results separately like Python does (in 'moves' list),
   * then apply after all checks. This ensures all intersection checks use
   * the ORIGINAL ps2 values, not values modified by earlier iterations. */
  Vector<std::pair<int, float3>> moves;

  for (int i = 0; i < face_len; i++) {
    const HBLoop &p1 = ps[i];
    Vector<std::pair<float, float3>> collisions;

    for (int k = 0; k < face_len; k++) {
      int k2 = (k + 1) % face_len;
      if (k == i || k2 == i) {
        continue;
      }

      /* Note: Python does intersect2d(p1, p2, p4, p3) - reversed order for second pair */
      auto pin = intersect_2d(p1.co, ps2[i].co, ps2[k2].co, ps2[k].co);
      if (pin.has_value()) {
        float3 m1 = *pin - p1.co;
        collisions.append({math::length(m1), m1});
      }
    }

    if (!collisions.is_empty()) {
      auto min_it = std::min_element(collisions.begin(), collisions.end(),
                                     [](const auto &a, const auto &b) { return a.first < b.first; });
      /* Store the move to apply later, don't modify ps2 yet */
      moves.append({i, p1.co + min_it->second * 0.999f});
    }
  }

  /* Apply all moves after all intersection checks are complete */
  for (const auto &move : moves) {
    ps2[move.first].co = move.second;
  }

  /* Convert back to 3D and store results */
  for (int i = 0; i < face_len; i++) {
    float3 new_co_2d = ps2[i].co;
    new_co_2d.z = 0.0f;
    float3 new_co_3d = math::transform_point(mat_to_3d, new_co_2d);
    result.add_overwrite(ps2[i].real_vert, new_co_3d);
  }

  return result;
}

/**
 * Main Hard Bevel algorithm entry point.
 */
static Mesh *hard_bevel_mesh(const Mesh &mesh,
                             float offset,
                             float threshold,
                             int segments,
                             int fillet_engine,
                             Span<bool> selection)
{
  /* segments parameter is "Number of Cuts" in Python, which uses cut = prop_cut - 1 */
  int cut = segments - 1;

  if (offset <= 0.0f) {
    return nullptr;
  }

  if (mesh.verts_num == 0) {
    return nullptr;
  }

  /* Convert mesh to BMesh */
  BMeshCreateParams create_params{};
  BMeshFromMeshParams convert_params{};
  convert_params.calc_face_normal = true;
  convert_params.calc_vert_normal = true;

  BMesh *bm = BKE_mesh_to_bmesh_ex(&mesh, &create_params, &convert_params);

  /* Clear all TAG flags */
  BMIter iter;
  BMEdge *e;
  BMVert *v;
  BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
    BM_elem_flag_disable(e, BM_ELEM_TAG);
  }
  BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
    BM_elem_flag_disable(v, BM_ELEM_TAG);
  }

  /* Tag edges to bevel */
  Vector<BMEdge *> sel_edges;
  int e_idx = 0;
  BM_ITER_MESH_INDEX (e, &iter, bm, BM_EDGES_OF_MESH, e_idx) {
    bool selected = selection.is_empty() || selection[e_idx];
    if (selected && BM_edge_is_manifold(e)) {
      sel_edges.append(e);
      BM_elem_flag_enable(e, BM_ELEM_TAG);
      BM_elem_flag_enable(e->v1, BM_ELEM_TAG);
      BM_elem_flag_enable(e->v2, BM_ELEM_TAG);
    }
  }

  if (sel_edges.is_empty()) {
    BM_mesh_free(bm);
    return nullptr;
  }

  /* fs1 = faces adjacent to selected edges (before bevel) - these will be modified */
  Set<BMFace *> fs1;
  for (BMEdge *edge : sel_edges) {
    BMLoop *l;
    BM_ITER_ELEM (l, &iter, edge, BM_LOOPS_OF_EDGE) {
      fs1.add(l->f);
    }
  }

  /* fso = faces NOT adjacent to selected edges (won't be touched by bevel) */
  Set<BMFace *> fso;
  BMFace *f;
  BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
    if (!fs1.contains(f)) {
      fso.add(f);
    }
  }

  /* CRITICAL: Clear BM_ELEM_TAG on all faces BEFORE bevel.
   * BM_mesh_bevel() internally tags newly created faces with BM_ELEM_TAG.
   * This is how bmesh.ops.bevel() gets res['faces'] - see bmo_bevel.cc line 88.
   * By clearing the tag first, we can identify new faces after bevel. */
  BM_mesh_elem_hflag_disable_all(bm, BM_FACE, BM_ELEM_TAG, false);

  /* Do tiny bevel (0.001) to create topology */
  BM_mesh_bevel(bm,
                0.001f,
                0,  /* offset_type = OFFSET */
                MOD_BEVEL_PROFILE_SUPERELLIPSE,
                1,  /* segments */
                0.5f,
                MOD_BEVEL_AFFECT_EDGES,
                false, false, nullptr, -1, -1, true, false, false, false,
                MOD_BEVEL_FACE_STRENGTH_NONE,
                MOD_BEVEL_MITER_SHARP, MOD_BEVEL_MITER_SHARP,
                0.0f, nullptr, MOD_BEVEL_VMESH_ADJ, -1, -1);

  /* Rebuild indices */
  BM_mesh_elem_table_ensure(bm, BM_VERT | BM_EDGE | BM_FACE);
  BM_mesh_elem_index_ensure(bm, BM_VERT | BM_EDGE | BM_FACE);

  /* Identify bevel strip faces using BM_ELEM_TAG - this is exactly how Python's
   * bmesh.ops.bevel() returns res['faces']. BM_mesh_bevel() tags newly created
   * faces, so we just collect all tagged faces. This is 100% accurate, unlike
   * our previous heuristic of "new quads". */
  Set<BMFace *> bevel_strip_faces;
  BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
    if (BM_elem_flag_test(f, BM_ELEM_TAG)) {
      bevel_strip_faces.add(f);
    }
  }

  /* pes = edges of bevel strip faces (vertex index pairs) */
  Set<std::pair<int, int>> pes;
  for (BMFace *bf : bevel_strip_faces) {
    BMLoop *l_first = BM_FACE_FIRST_LOOP(bf);
    BMLoop *l_iter = l_first;
    do {
      int idx1 = BM_elem_index_get(l_iter->v);
      int idx2 = BM_elem_index_get(l_iter->next->v);
      pes.add({idx1, idx2});
      l_iter = l_iter->next;
    } while (l_iter != l_first);
  }

  /* fs2 = all faces except strip faces and untouched original faces
   * This includes: corner faces AND modified original faces adjacent to bevel */
  Set<BMFace *> fs2;
  BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
    if (!fso.contains(f) && !bevel_strip_faces.contains(f)) {
      fs2.add(f);
    }
  }

  /* Collect vertex updates from all fs2 faces */
  Map<BMVert *, Vector<float3>> vmap;

  for (BMFace *face : fs2) {
    /* Initialize vmap entries */
    BMLoop *l_first = BM_FACE_FIRST_LOOP(face);
    BMLoop *l_iter = l_first;
    do {
      if (!vmap.contains(l_iter->v)) {
        vmap.add(l_iter->v, Vector<float3>());
      }
      l_iter = l_iter->next;
    } while (l_iter != l_first);

    /* Compute transformation matrices */
    float4x4 mat_to_2d = get_matrix_face(face);
    float4x4 mat_to_3d = math::invert(mat_to_2d);

    /* Solve face.
     * plen = bevel amount (user input)
     * threshold = collision detection threshold */
    Map<BMVert *, float3> face_positions = solve_face(face, pes, offset, threshold, mat_to_2d, mat_to_3d);

    for (auto item : face_positions.items()) {
      if (vmap.contains(item.key)) {
        vmap.lookup(item.key).append(item.value);
      }
    }
  }

  /* Apply positions - when multiple faces contribute, take closest to original */
  for (auto item : vmap.items()) {
    BMVert *vert = item.key;
    Vector<float3> &positions = item.value;

    if (positions.is_empty()) {
      continue;
    }
    else if (positions.size() == 1) {
      copy_v3_v3(vert->co, positions[0]);
    }
    else {
      float3 orig_co(vert->co);
      float min_dist = FLT_MAX;
      float3 best_pos = orig_co;

      for (const float3 &pos : positions) {
        float dist = math::distance(pos, orig_co);
        if (dist < min_dist) {
          min_dist = dist;
          best_pos = pos;
        }
      }
      copy_v3_v3(vert->co, best_pos);
    }
  }

  /* ========== FILLET SYSTEM ==========
   * When cut > 0, we add rounded fillets by:
   * 1. Subdividing bevel strip edges
   * 2. Creating interior grids in corner faces
   * 3. Bending edges using Bezier curves
   * 4. Smoothing interior vertices using RBF interpolation
   */
  if (cut > 0) {
    /* Recalculate face normals after vertex positions changed */
    BM_mesh_normals_update(bm);

    /* Get vertex normals for bending calculations */
    Map<BMVert *, float3> snmap = get_outer_normal(bm, fs2);

    /* Identify edges to subdivide:
     * These are edges of bevel strip faces that are shared with other bevel strips
     * (i.e., internal edges, not boundary edges) */
    Set<BMEdge *> edges_bevel;
    Set<BMEdge *> edges_border;

    for (BMFace *face : fs2) {
      BMLoop *l_first = BM_FACE_FIRST_LOOP(face);
      BMLoop *l_iter = l_first;
      do {
        edges_border.add(l_iter->e);
        l_iter = l_iter->next;
      } while (l_iter != l_first);
    }

    for (BMFace *face : bevel_strip_faces) {
      BMLoop *l_first = BM_FACE_FIRST_LOOP(face);
      BMLoop *l_iter = l_first;
      do {
        edges_bevel.add(l_iter->e);
        l_iter = l_iter->next;
      } while (l_iter != l_first);
    }

    /* hs = bevel edges that are NOT border edges (internal edges to subdivide) */
    Set<BMEdge *> hs;
    for (BMEdge *edge : edges_bevel) {
      if (!edges_border.contains(edge)) {
        hs.add(edge);
      }
    }

    /* Subdivide internal edges - store per-loop like Python does
     * Matches Python surround_net lines 1720-1727 */
    Map<BMLoop *, Vector<BMVert *>> submap;
    for (BMEdge *edge : hs) {
      auto sub = divide_edge(bm, edge, cut);
      if (sub.has_value()) {
        const auto &[loop1_data, loop2_data] = *sub;
        submap.add(loop1_data.first, loop1_data.second);
        submap.add(loop2_data.first, loop2_data.second);
      }
    }

    /* Identify "core" faces (bevel strips where ALL edges are in hs)
     * and "tube" faces (bevel strips with some non-hs edges)
     * Matches Python surround_net lines 1731-1742 */
    Set<BMFace *> fscore;
    Set<BMFace *> fstube;

    for (BMFace *face : bevel_strip_faces) {
      int face_len = face->len;
      int hs_count = 0;

      BMLoop *l_first = BM_FACE_FIRST_LOOP(face);
      BMLoop *l_iter = l_first;
      do {
        if (hs.contains(l_iter->e)) {
          hs_count++;
        }
        l_iter = l_iter->next;
      } while (l_iter != l_first);

      if (hs_count == face_len) {
        fscore.add(face);
      }
      else {
        fstube.add(face);
      }
    }

    /* Process core faces - create interior grids
     * Matches Python surround_net lines 1747-1759 */
    Vector<Vector<VertexRing>> fsvs;
    Vector<Vector<BMVert *>> fscen;

    for (BMFace *face : fscore) {
      /* Build outer ring from loop subdivisions
       * Matches Python lines 1748-1754: vs = [(p1.vert, submap[p1]), ...] */
      VertexRing vs;
      BMLoop *l_first = BM_FACE_FIRST_LOOP(face);
      BMLoop *l_iter = l_first;
      do {
        if (!submap.contains(l_iter)) {
          l_iter = l_iter->next;
          continue;
        }
        const Vector<BMVert *> &sub = submap.lookup(l_iter);
        vs.append({l_iter->v, sub});
        l_iter = l_iter->next;
      } while (l_iter != l_first);

      /* Create nested rings and connect with faces */
      Vector<VertexRing> vs2 = get_sub_slice(bm, vs, cut);
      Vector<BMVert *> cens = connect_inner_face(bm, vs2, cut);

      fsvs.append(vs2);
      fscen.append(cens);
    }

    /* Track faces that shouldn't be deleted (have 0 hs edges) */
    Set<BMFace *> nondelete;

    /* Process tube faces - create simple quad strips
     * Matches Python surround_net lines 1763-1797 */
    for (BMFace *face : fstube) {
      Vector<BMLoop *> ec;
      BMLoop *l_first = BM_FACE_FIRST_LOOP(face);
      BMLoop *l_iter = l_first;
      do {
        if (hs.contains(l_iter->e)) {
          ec.append(l_iter);
        }
        l_iter = l_iter->next;
      } while (l_iter != l_first);

      if (ec.size() == 0) {
        nondelete.add(face);
        continue;
      }
      else if (ec.size() == 1) {
        /* Single edge subdivision - create fan
         * Matches Python lines 1774-1782 */
        BMLoop *p1 = ec[0];
        if (submap.contains(p1)) {
          Vector<BMVert *> all_verts;
          BMLoop *loop = BM_FACE_FIRST_LOOP(face);
          do {
            all_verts.append(loop->v);
            loop = loop->next;
          } while (loop != BM_FACE_FIRST_LOOP(face));

          int pindex = -1;
          for (int i = 0; i < all_verts.size(); i++) {
            if (all_verts[i] == p1->v) {
              pindex = i;
              break;
            }
          }

          if (pindex >= 0) {
            const Vector<BMVert *> &sub1 = submap.lookup(p1);
            Vector<BMVert *> vs2;
            for (int i = 0; i <= pindex; i++) {
              vs2.append(all_verts[i]);
            }
            for (BMVert *sv : sub1) {
              vs2.append(sv);
            }
            for (int i = pindex + 1; i < all_verts.size(); i++) {
              vs2.append(all_verts[i]);
            }
            if (vs2.size() >= 3) {
              BM_face_create_verts(bm, vs2.data(), vs2.size(), nullptr, BM_CREATE_NOP, true);
            }
          }
        }
      }
      else if (ec.size() == 2) {
        /* Two opposite edges - create quad strip
         * Matches Python lines 1784-1797 */
        BMLoop *p1 = ec[0];
        BMLoop *p2 = ec[1];

        if (submap.contains(p1) && submap.contains(p2)) {
          const Vector<BMVert *> &sub1_raw = submap.lookup(p1);
          const Vector<BMVert *> &sub2_raw = submap.lookup(p2);

          /* sub1 = [p1.vert] + submap[p1] + [p1.link_loop_next.vert] */
          Vector<BMVert *> sub1;
          sub1.append(p1->v);
          for (BMVert *sv : sub1_raw) {
            sub1.append(sv);
          }
          sub1.append(p1->next->v);

          /* sub2 = [p2.vert] + submap[p2] + [p2.link_loop_next.vert] */
          Vector<BMVert *> sub2;
          sub2.append(p2->v);
          for (BMVert *sv : sub2_raw) {
            sub2.append(sv);
          }
          sub2.append(p2->next->v);

          /* sub2 = list(reversed(sub2)) */
          std::reverse(sub2.begin(), sub2.end());

          /* Create quads - Python lines 1790-1797:
           * v1 = sub1[i], v2 = sub1[i2], v3 = sub2[i2], v4 = sub2[i] */
          for (int i = 0; i < int(sub1.size()) - 1; i++) {
            int i2 = i + 1;
            BMVert *v1 = sub1[i];
            BMVert *v2 = sub1[i2];
            BMVert *v3 = sub2[i2];
            BMVert *v4 = sub2[i];
            BMVert *verts[4] = {v1, v2, v3, v4};
            BM_face_create_verts(bm, verts, 4, nullptr, BM_CREATE_NOP, true);
          }
        }
      }
    }

    /* Apply fillet bending and smoothing to core faces
     * Matches Python surround_net lines 1799-1802: fillet_poly() */
    for (int fi = 0; fi < fsvs.size(); fi++) {
      Vector<VertexRing> &vs2 = fsvs[fi];
      Vector<BMVert *> &vcen = fscen[fi];

      if (vs2.is_empty()) {
        continue;
      }

      /* Collect boundary points BEFORE bending */
      Vector<float3> bounds;
      const VertexRing &ps_outer = vs2[0];
      for (const auto &pair : ps_outer) {
        bounds.append(float3(pair.first->co));
        for (BMVert *sv : pair.second) {
          bounds.append(float3(sv->co));
        }
      }

      /* Apply Bezier bending to outer ring subdivisions
       * Matches Python fillet_poly lines 1290-1302 */
      for (int i = 0; i < ps_outer.size(); i++) {
        int i2 = (i + 1) % ps_outer.size();
        BMVert *p1 = ps_outer[i].first;
        const Vector<BMVert *> &sub = ps_outer[i].second;
        BMVert *p2 = ps_outer[i2].first;

        auto sub2 = get_bend_sub(p1, p2, snmap, cut);
        if (!sub2.has_value()) {
          continue;
        }
        if (sub.size() != sub2->size()) {
          continue;
        }
        for (int k = 0; k < sub.size(); k++) {
          copy_v3_v3(sub[k]->co, (*sub2)[k]);
        }
      }

      /* Collect boundary points AFTER bending */
      Vector<float3> bounds2;
      for (const auto &pair : ps_outer) {
        bounds2.append(float3(pair.first->co));
        for (BMVert *sv : pair.second) {
          bounds2.append(float3(sv->co));
        }
      }

      /* Collect interior points (inner rings + center)
       * Matches Python fillet_poly lines 1310-1317 */
      Vector<float3> points;
      Vector<BMVert *> point_verts;
      for (int ri = 1; ri < vs2.size(); ri++) {
        for (const auto &pair : vs2[ri]) {
          points.append(float3(pair.first->co));
          point_verts.append(pair.first);
          for (BMVert *sv : pair.second) {
            points.append(float3(sv->co));
            point_verts.append(sv);
          }
        }
      }
      for (BMVert *cv : vcen) {
        points.append(float3(cv->co));
        point_verts.append(cv);
      }

      /* Apply RBF interpolation
       * Matches Python fillet_poly lines 1319-1343 */
      if (!points.is_empty() && !bounds.is_empty()) {
        Vector<float3> points2;
        if (fillet_engine == 2) {
          points2 = move_forward_gaussian(points, bounds, bounds2, 1.0f);
        }
        else {
          points2 = move_forward_wendland(points, bounds, bounds2, 1.0f);
        }

        float dislimit = offset * 2.0f;
        for (int i = 0; i < point_verts.size(); i++) {
          float3 a1 = points2[i];
          float3 old_co(point_verts[i]->co);
          if (math::distance(a1, old_co) < dislimit) {
            copy_v3_v3(point_verts[i]->co, a1);
          }
        }
      }
    }

    /* Apply Bezier bending to edges in hs (for tube faces)
     * Matches Python surround_net lines 1805-1820 */
    for (BMEdge *edge : hs) {
      if (edge->l == nullptr) {
        continue;
      }
      BMLoop *p1 = edge->l;
      BMVert *v2 = BM_edge_other_vert(edge, p1->v);

      if (!submap.contains(p1)) {
        continue;
      }
      const Vector<BMVert *> &sub1 = submap.lookup(p1);
      if (sub1.is_empty()) {
        continue;
      }

      auto co = get_bend_sub(p1->v, v2, snmap, cut);
      if (!co.has_value()) {
        continue;
      }
      if (co->size() != sub1.size()) {
        continue;
      }
      for (int k = 0; k < sub1.size(); k++) {
        copy_v3_v3(sub1[k]->co, (*co)[k]);
      }
    }

    /* Delete original bevel strip faces (except nondelete)
     * Matches Python surround_net lines 1822-1824 */
    Vector<BMFace *> fdel;
    for (BMFace *face : bevel_strip_faces) {
      if (!nondelete.contains(face)) {
        fdel.append(face);
      }
    }
    for (BMFace *face : fdel) {
      BM_face_kill(bm, face);
    }
  }

  /* Convert back to Mesh */
  Mesh *result = BKE_mesh_from_bmesh_for_eval_nomain(bm, nullptr, &mesh);
  BM_mesh_free(bm);

  return result;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Node Definition
 * \{ */

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Mesh")
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Mesh to bevel");

  b.add_input<decl::Bool>("Selection")
      .default_value(true)
      .hide_value()
      .field_on_all()
      .description("Edges to bevel");

  b.add_input<decl::Float>("Amount")
      .default_value(0.1f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Bevel offset amount");

  b.add_input<decl::Float>("Threshold")
      .default_value(0.01f)
      .min(0.0f)
      .max(1.0f)
      .description("Collision detection threshold (smaller = more precise)");

  b.add_input<decl::Int>("Segments")
      .default_value(1)
      .min(1)
      .max(10)
      .description("Number of segments for rounded fillet (1 = sharp bevel)");

  b.add_input<decl::Int>("Fillet Engine")
      .default_value(1)
      .min(1)
      .max(2)
      .description("RBF interpolation type: 1 = Wendland (compact), 2 = Gaussian");

  b.add_output<decl::Geometry>("Mesh").propagate_all();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh");

  const float amount = params.get_input<float>("Amount");
  const float threshold = params.get_input<float>("Threshold");
  const int segments = params.get_input<int>("Segments");
  const int fillet_engine = params.get_input<int>("Fillet Engine");
  const Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");

  if (amount <= 0.0f) {
    params.set_output("Mesh", std::move(geometry_set));
    return;
  }

  geometry::foreach_real_geometry(geometry_set, [&](GeometrySet &geometry_set) {
    if (const Mesh *mesh = geometry_set.get_mesh()) {
      const bke::MeshFieldContext field_context{*mesh, bke::AttrDomain::Edge};
      const int edges_num = mesh->edges_num;

      fn::FieldEvaluator selection_evaluator{field_context, edges_num};
      selection_evaluator.add(selection_field);
      selection_evaluator.evaluate();
      const VArray<bool> selection_varray = selection_evaluator.get_evaluated<bool>(0);

      Array<bool> selection(edges_num);
      selection_varray.materialize(selection.as_mutable_span());

      Mesh *result = hard_bevel_mesh(*mesh, amount, threshold, segments, fillet_engine, selection);

      if (result) {
        geometry_set.replace_mesh(result);
      }
    }
  });

  params.set_output("Mesh", std::move(geometry_set));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeHardBevel", GEO_NODE_HARD_BEVEL);
  ntype.ui_name = "Hard Bevel";
  ntype.ui_description = "Bevel edges with straight skeleton collision detection";
  ntype.enum_name_legacy = "HARD_BEVEL";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

/** \} */

}  // namespace blender::nodes::node_geo_hard_bevel_cc
