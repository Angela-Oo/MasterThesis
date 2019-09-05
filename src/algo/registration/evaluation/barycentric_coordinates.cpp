#include "barycentric_coordinates.h"

namespace Registration {
	

SurfaceMesh::Point pointOnPlane(Plane plane, SurfaceMesh::Point point)
{
	auto normal = plane.normal;
	Vector direction = point - plane.point;
	auto angle = CGAL::scalar_product(normal, direction);
	auto h = sin(angle) * std::sqrt(direction.squared_length());

	if (isnan(h))
		std::cout << "fuu h" << std::endl;
	auto point_on_plane = point - normal * h;
	return point_on_plane;
}


double area(Point point_a, Point point_b, Point point_c)
{
	auto c = point_b - point_a;
	auto b = point_c - point_a;
	assert(c.squared_length() != 0.f && b.squared_length() != 0.f);

	double angle = CGAL::scalar_product(c, b);
	double b_length = sqrt(b.squared_length());
	double hb = sin(angle) * b_length;
	double area = (hb * b_length) / 2.f;
	return area;
}

Point barycentricCoordinates(Point point_a, Point point_b, Point point_c, Point point_on_triangle)
{
	auto area_u = area(point_on_triangle, point_a, point_b);
	auto area_v = area(point_on_triangle, point_b, point_c);
	auto area_w = area(point_on_triangle, point_c, point_a);
	auto area_face = area(point_a, point_b, point_c);

	assert(area_face != 0.);
	double u = area_u / area_face;
	double v = area_v / area_face;
	double w = area_w / area_face;
	return Point(u, v, w);
}


}