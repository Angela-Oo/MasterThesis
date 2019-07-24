#include "stdafx.h"

#include "ed_deformation.h"

namespace Registration {
namespace ED {

Matrix Deformation::rotation() const
{
	Matrix m(_r(0, 0), _r(0, 1), _r(0, 2), _r(1, 0), _r(1, 1), _r(1, 2), _r(2, 0), _r(2, 1), _r(2, 2));
	return m;
}

Vector Deformation::translation() const
{
	return Vector(_t[0], _t[1], _t[2]);
}

Point Deformation::position() const
{
	return _position;
}

Point Deformation::getDeformedPosition() const
{
	Vector v = _position - CGAL::ORIGIN;
	return CGAL::ORIGIN + v + translation();
}

Point Deformation::deformPosition(const Point & point) const
{
	auto edge = point - _position;
	Vector rotated_point = rotation()(edge);
	Vector moved_position = (_position - CGAL::ORIGIN) + translation();
	return CGAL::ORIGIN + moved_position + rotated_point;
}

Vector Deformation::deformNormal(const Vector & normal) const
{
	return rotation()(normal);
}

Deformation Deformation::invertDeformation() const
{
	return Deformation(_position, _r.getInverse(), -_t, _w);
}

Deformation::Deformation(const Point & position, const ml::mat3d & r, const ml::vec3d & t, double w)
	: _position(position)
	, _r(r)
	, _t(t)
	, _w(w)
{}

Deformation::Deformation()
	: Deformation(CGAL::ORIGIN, ml::mat3d::identity(), ml::vec3f::origin)
{}

Deformation::Deformation(const Point & position)
	: Deformation(position, ml::mat3d::identity(), ml::vec3f::origin)
{}

Deformation::Deformation(const RigidDeformation & rigid_deformation)
{
	auto r = rigid_deformation.rotation();
	//double x = r.m(0,1);
	ml::mat3d rotation(r.m(0, 0), r.m(0, 1), r.m(0, 2), r.m(1, 0), r.m(1, 1), r.m(1, 2), r.m(2, 0), r.m(2, 1), r.m(2, 2));
	_r = rotation;
	_t = rigid_deformation._t;
	_position = rigid_deformation._g;
}

Deformation::Deformation(const Deformation & other)
	: _position(other._position)
	, _r(other._r)
	, _t(other._t)
	, _w(other._w)
{}

Deformation::Deformation(const Deformation & deformation, bool inverse)
	: Deformation(deformation)
{
	if (inverse) {
		_r = deformation._r.getInverse();
		_t = -deformation._t;
	}
}


}
}