#include "stdafx.h"

#include "ed_deformation.h"

namespace Registration {

Matrix EDDeformation::rotation() const
{
	Matrix m(_r(0, 0), _r(0, 1), _r(0, 2), _r(1, 0), _r(1, 1), _r(1, 2), _r(2, 0), _r(2, 1), _r(2, 2));
	return m;
}

Vector EDDeformation::translation() const
{
	return Vector(_t[0], _t[1], _t[2]);
}

Point EDDeformation::position() const
{
	return _position;
}

Point EDDeformation::getDeformedPosition() const
{
	Vector v = _position - CGAL::ORIGIN;
	return CGAL::ORIGIN + v + translation();
}

Point EDDeformation::deformPosition(const Point & point) const
{
	auto edge = point - _position;
	Vector rotated_point = rotation()(edge);
	Vector moved_position = (_position - CGAL::ORIGIN) + translation();
	return CGAL::ORIGIN + moved_position + rotated_point;
}

Vector EDDeformation::deformNormal(const Vector & normal) const
{
	return rotation()(normal);
}

EDDeformation EDDeformation::invertDeformation() const
{
	return EDDeformation(getDeformedPosition(), _r.getInverse(), -_t, _w);
}

EDDeformation::EDDeformation(const Point & position, const ml::mat3d & r, const ml::vec3d & t, double w)
	: _position(position)
	, _r(r)
	, _t(t)
	, _w(w)
{}

EDDeformation::EDDeformation()
	: EDDeformation(CGAL::ORIGIN, ml::mat3d::identity(), ml::vec3f::origin)
{}

EDDeformation::EDDeformation(const Point & position)
	: EDDeformation(position, ml::mat3d::identity(), ml::vec3f::origin)
{}

EDDeformation::EDDeformation(const RigidDeformation & rigid_deformation)
{
	auto r = rigid_deformation.rotation();
	//double x = r.m(0,1);
	ml::mat3d rotation(r.m(0, 0), r.m(0, 1), r.m(0, 2), r.m(1, 0), r.m(1, 1), r.m(1, 2), r.m(2, 0), r.m(2, 1), r.m(2, 2));
	_r = rotation;
	_t = rigid_deformation._t;
	_position = rigid_deformation._g;
}

EDDeformation::EDDeformation(const EDDeformation & other)
	: _position(other._position)
	, _r(other._r)
	, _t(other._t)
	, _w(other._w)
{}

EDDeformation::EDDeformation(const EDDeformation & deformation, bool inverse)
	: EDDeformation(deformation)
{
	if (inverse) {
		_r = deformation._r.getInverse();
		_t = -deformation._t;
	}
}


}