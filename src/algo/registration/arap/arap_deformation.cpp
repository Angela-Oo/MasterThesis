#include "stdafx.h"

#include "arap_deformation.h"
#include <ceres/rotation.h>


namespace ARAP {

Matrix Deformation::rotation() const
{
	ml::mat3d r;
	ceres::AngleAxisToRotationMatrix(_r.array, r.getData());
	auto r_t =  r.getTranspose(); // why??
	Matrix m(r_t(0, 0), r_t(0, 1), r_t(0, 2), r_t(1, 0), r_t(1, 1), r_t(1, 2), r_t(2, 0), r_t(2, 1), r_t(2, 2));
	return m;
}

Vector Deformation::translation() const
{
	return Vector(_t[0], _t[1], _t[2]);
}

Deformation::Deformation(const ml::vec3d & r, const ml::vec3d & t)
	: _r(r)
	, _t(t)
	, _w(1.)
{}

Deformation::Deformation()
	: Deformation(ml::vec3f::origin, ml::vec3f::origin)
{}

Deformation::Deformation(const Deformation & node, bool inverse)
	: _r(node._r)
	, _t(node._t)
	, _w(node._w)
{
	if (inverse) {
		_r = -node._r;
		_t = -node._t;
	}
}

}