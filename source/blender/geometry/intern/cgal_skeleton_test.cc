/* CGAL Straight Skeleton Integration Test */

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/create_offset_polygons_2.h>

#include <memory>
#include <vector>
#include <iostream>

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_2 = Kernel::Point_2;
using Polygon_2 = CGAL::Polygon_2<Kernel>;
using PolygonPtr = std::shared_ptr<Polygon_2>;
using PolygonPtrVector = std::vector<PolygonPtr>;

int main() {
  /* Create test polygon (star shape) */
  Polygon_2 poly;
  poly.push_back(Point_2(-1, -1));
  poly.push_back(Point_2(0, -12));
  poly.push_back(Point_2(1, -1));
  poly.push_back(Point_2(12, 0));
  poly.push_back(Point_2(1, 1));
  poly.push_back(Point_2(0, 12));
  poly.push_back(Point_2(-1, 1));
  poly.push_back(Point_2(-12, 0));

  /* Ensure CCW orientation */
  if (poly.is_clockwise_oriented()) {
    poly.reverse_orientation();
  }

  /* Create interior offset */
  double offset = 1.0;
  PolygonPtrVector result = CGAL::create_interior_skeleton_and_offset_polygons_2(offset, poly);

  std::cout << "Created " << result.size() << " offset polygons\n";

  for (size_t i = 0; i < result.size(); i++) {
    std::cout << "Polygon " << i << " has " << result[i]->size() << " vertices\n";
  }

  return 0;
}
