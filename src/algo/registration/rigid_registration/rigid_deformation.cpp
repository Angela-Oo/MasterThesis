#include "stdafx.h"
#include "rigid_deformation.h"
#include "ceres/rotation.h"

Matrix RigidDeformation::rotation() const
{
	ml::mat3d r;
	ceres::AngleAxisToRotationMatrix(_r.array, r.getData());
	auto r_t = r.getTranspose(); // why??
	Matrix m(r_t(0, 0), r_t(0, 1), r_t(0, 2), r_t(1, 0), r_t(1, 1), r_t(1, 2), r_t(2, 0), r_t(2, 1), r_t(2, 2));
	return m;
}

Vector RigidDeformation::translation() const
{
	return Vector(_t[0], _t[1], _t[2]);
}

Point RigidDeformation::deformPoint(const Point & point) const
{
	Vector rotated_point = rotation()(point - CGAL::ORIGIN);
	Vector moved_position = translation();
	return CGAL::ORIGIN + rotated_point + moved_position;
}

Vector RigidDeformation::deformNormal(const Vector & normal) const
{
	ml::mat3d r;
	ceres::AngleAxisToRotationMatrix(_r.array, r.getData());
	Matrix rotation(r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2), r(2, 0), r(2, 1), r(2, 2));
	Vector rotated_normal = rotation(normal);
	return rotated_normal;
}

RigidDeformation::RigidDeformation()
	: _r(ml::vec3f::origin)
	, _t(ml::vec3f::origin)
{}

