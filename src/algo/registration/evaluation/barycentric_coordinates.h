#pragma once

#include "mesh/mesh_definition.h"


namespace Registration {


class Plane
{
public:
	Point point;
	Vector normal;
public:
	Plane(Point p, Vector n)
		: point(p)
		, normal(n)
	{}
};

SurfaceMesh::Point pointOnPlane(Plane plane, SurfaceMesh::Point point);

double area(Point point_a, Point point_b, Point point_c);

Point barycentricCoordinates(Point point_a, Point point_b, Point point_c, Point point_on_triangle);

}