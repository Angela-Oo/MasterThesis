#include "stdafx.h"

#include "arap_deformation.h"
#include <ceres/rotation.h>


namespace ARAP {

Matrix Deformation::rotation() const
{
	ml::mat3d r;
	ceres::AngleAxisToRotationMatrix(_d.array, r.getData());
	auto r_t =  r.getTranspose(); // why??
	Matrix m(r_t(0, 0), r_t(0, 1), r_t(0, 2), r_t(1, 0), r_t(1, 1), r_t(1, 2), r_t(2, 0), r_t(2, 1), r_t(2, 2));
	return m;
}

Vector Deformation::translation() const
{
	return Vector(_d[3], _d[4], _d[5]);
}

std::shared_ptr<IDeformation> Deformation::invertDeformation() const
{
	return std::make_shared<Deformation>(-_d, _w);
}

Deformation::Deformation(const ml::vec3d & r, const ml::vec3d & t, double w)
	: _w(w)
{
	_d[0] = r[0];
	_d[1] = r[1];
	_d[2] = r[2];
	_d[3] = t[0];
	_d[4] = t[1];
	_d[5] = t[2];
}

Deformation::Deformation(const ml::vec6d & d,  double w)
	: _d(d)
	, _w(w)
{ }

Deformation::Deformation()
	: Deformation(ml::vec3f::origin, ml::vec3f::origin)
{}

Deformation::Deformation(const Deformation & node, bool inverse)
	: _d(node._d)
	, _w(node._w)
{
	if (inverse) {
		_d = -node._d;
	}
}

}