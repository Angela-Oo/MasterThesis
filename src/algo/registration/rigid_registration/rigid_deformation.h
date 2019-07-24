#pragma once
#include "mLibCore.h"
#include "algo/surface_mesh/mesh_definition.h"

namespace Registration {

class RigidDeformation
{
public:
	Point _g; // center position
	ml::vec3d _r; // rotation matrix in angle axis
	ml::vec3d _t; // translation vector	
public:
	double * r() { return (&_r)->getData(); }
	double * t() { return (&_t)->getData(); }
public:
	Matrix rotation() const;
	Vector translation() const;
public:
	Point getDeformedPosition() const;
	Point deformPosition(Point point) const;
	Point deformPoint(const Point & point) const; // todo remove
	Vector deformNormal(const Vector & normal) const;;
public:
	RigidDeformation invertDeformation() const;
public:
	RigidDeformation();
	RigidDeformation(ml::vec3d r, ml::vec3d t, Point g = CGAL::ORIGIN);
	RigidDeformation(Matrix r, Vector t, Point g = CGAL::ORIGIN);
};

Point calculateGlobalCenter(const SurfaceMesh & mesh);

}