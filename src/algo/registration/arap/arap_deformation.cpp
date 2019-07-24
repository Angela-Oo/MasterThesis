#include "stdafx.h"

#include "arap_deformation.h"
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

std::shared_ptr<IPositionDeformation> ARAPDeformation::invertDeformation() const
{
	return std::make_shared<ARAPDeformation>(_position, -_d, _w);
}

std::shared_ptr<IPositionDeformation> ARAPDeformation::clone() const
{
	return std::make_shared<ARAPDeformation>(*this);
}

ARAPDeformation::ARAPDeformation(const Point & position, const ml::vec6d & d,  double w)
	: _position(position)
	, _d(d)
	, _w(w)
{ }

ARAPDeformation::ARAPDeformation(const Point & position)
	: ARAPDeformation(position, ml::vec6d::origin)
{ }

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