#pragma once
#include "algo/surface_mesh/mesh_definition.h"

class RigidDeformation
{
private:
	ml::vec3d _r; // rotation matrix in angle axis
	ml::vec3d _t; // translation vector	
public:
	double * r() { return (&_r)->getData(); }
	double * t() { return (&_t)->getData(); }
public:
	Matrix rotation() const;
	Vector translation() const;
public:
	Point deformPoint(const Point & point) const;
	Vector deformNormal(const Vector & normal) const;
public:
	RigidDeformation invertDeformation();
public:
	RigidDeformation();
	RigidDeformation(ml::vec3d r, ml::vec3d t);
};

