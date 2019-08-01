#include "arap_deformation.h"
#include "mLibCore.h"
#include <ceres/rotation.h>


namespace Registration {

Matrix ARAPDeformation::rotation() const
{
	ml::mat3d r;
	ceres::AngleAxisToRotationMatrix(_d.array, r.getData());
	auto r_t =  r.getTranspose(); // why??
	Matrix m(r_t(0, 0), r_t(0, 1), r_t(0, 2), r_t(1, 0), r_t(1, 1), r_t(1, 2), r_t(2, 0), r_t(2, 1), r_t(2, 2));
	return m;
}

Vector ARAPDeformation::translation() const
{
	return Vector(_d[3], _d[4], _d[5]);
}

Point ARAPDeformation::position() const
{
	return _position;
}

Point ARAPDeformation::getDeformedPosition() const
{
	Vector v = _position - CGAL::ORIGIN;
	return CGAL::ORIGIN + v + translation();
}

Point ARAPDeformation::deformPosition(const Point & point) const
{
	auto edge = point - _position;
	Vector rotated_point = rotation()(edge);
	Vector moved_position = (_position - CGAL::ORIGIN) + translation();
	return CGAL::ORIGIN + moved_position + rotated_point;
}

Vector ARAPDeformation::deformNormal(const Vector & normal) const
{
	return rotation()(normal);
}

ARAPDeformation ARAPDeformation::invertDeformation() const
{
	return ARAPDeformation(getDeformedPosition(), -_d, _w);
}

ARAPDeformation::ARAPDeformation(const Point & position, const ml::vec6d & d,  double w)
	: _position(position)
	, _d(d)
	, _w(w)
{ }

ARAPDeformation::ARAPDeformation(const Point & position)
	: ARAPDeformation(position, ml::vec6d::origin)
{ }

ARAPDeformation::ARAPDeformation(const RigidDeformation & rigid_deformation)
{
	_d[0] = rigid_deformation._r[0];
	_d[1] = rigid_deformation._r[1];
	_d[2] = rigid_deformation._r[2];
	_d[3] = rigid_deformation._t[0];
	_d[4] = rigid_deformation._t[1];
	_d[5] = rigid_deformation._t[2];
	_position = rigid_deformation._g;
}

ARAPDeformation::ARAPDeformation()
	: ARAPDeformation(CGAL::ORIGIN)
{}

ARAPDeformation::ARAPDeformation(const ARAPDeformation & other)
	: _position(other._position)
	, _d(other._d)
	, _w(other._w)
{}

ARAPDeformation::ARAPDeformation(const ARAPDeformation & deformation, bool inverse)
	: ARAPDeformation(deformation)
{
	if (inverse) {
		_d = -deformation._d;
	}
}


}