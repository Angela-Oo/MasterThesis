#include "stdafx.h"
#include "rigid_deformation.h"
#include "algo/registration/util/ceres_math.h"
#include "ceres/rotation.h"

namespace Registration {

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

Point RigidDeformation::position() const
{
	return _g;
}

Point RigidDeformation::getDeformedPosition() const
{
	Vector v = _g - CGAL::ORIGIN;
	return CGAL::ORIGIN + v + translation();
}

Point RigidDeformation::deformPosition(Point point) const
{
	auto edge = point - _g;
	Vector rotated_point = rotation()(edge);
	Vector moved_position = (_g - CGAL::ORIGIN) + translation();
	return CGAL::ORIGIN + moved_position + rotated_point;
}

Point RigidDeformation::deformPoint(const Point & point) const
{
	return deformPosition(point);
}

Vector RigidDeformation::deformNormal(const Vector & normal) const
{
	ml::mat3d r;
	ceres::AngleAxisToRotationMatrix(_r.array, r.getData());
	Matrix rotation(r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2), r(2, 0), r(2, 1), r(2, 2));
	Vector rotated_normal = rotation(normal);
	return rotated_normal;
}

RigidDeformation RigidDeformation::invertDeformation() const
{
	return RigidDeformation(-_r, -_t, getDeformedPosition());
}



RigidDeformation::RigidDeformation()
	: RigidDeformation(CGAL::ORIGIN)
{}

RigidDeformation::RigidDeformation(Point position)
	: _r(ml::vec3f::origin)
	, _t(ml::vec3f::origin)
	, _g(position)
{
}

RigidDeformation::RigidDeformation(ml::vec3d r, ml::vec3d t, Point g)
	: _r(r)
	, _t(t)
	, _g(g)
{ }

RigidDeformation::RigidDeformation(Matrix r, Vector t, Point g)
	: _g(g)
{
	double rotation[3];
	double rotation_matrix[9];
	Matrix3x3_toT(r, rotation_matrix);
	ceres::RotationMatrixToAngleAxis(rotation_matrix, rotation);
	_r = T_to_vec3f(rotation);

	_t = ml::vec3f(t.x(), t.y(), t.z());

}


RigidDeformation operator+(const RigidDeformation & lhs, const RigidDeformation & rhs)
{
	auto lhs_t = Matrix(CGAL::TRANSLATION, lhs.translation());
	auto rhs_t = Matrix(CGAL::TRANSLATION, rhs.translation());

	auto lhs_transformation = lhs.rotation() * lhs_t;
	auto rhs_transformation = rhs.rotation() * rhs_t;
	Matrix r = rhs_transformation * lhs_transformation;
	
	Vector t (r.m(0, 3), r.m(1, 3), r.m(2, 3));
	return RigidDeformation(r, t, lhs.position());

	// works only if the rotation axis is equal
	//auto r = lhs._r + rhs._r;
	//auto t = lhs._t + rhs._t;
	//return RigidDeformation(r, t, lhs.position());
}


Point calculateGlobalCenter(const SurfaceMesh & mesh)
{
	Vector global_position(0., 0., 0.);
	for (auto & v : mesh.vertices()) {
		global_position += mesh.point(v) - CGAL::ORIGIN;
	}
	global_position /= mesh.number_of_vertices();
	return CGAL::ORIGIN + global_position;
}

}