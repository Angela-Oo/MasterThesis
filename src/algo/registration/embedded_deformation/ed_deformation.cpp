#include "stdafx.h"

#include "ed_deformation.h"

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

Deformation::Deformation(const ml::mat3d & r, const ml::vec3d & t)
	: _r(r)
	, _t(t)
	, _w(1.)
{}

Deformation::Deformation()
	: Deformation(ml::mat3d::identity(), ml::vec3f::origin)
{}

Deformation::Deformation(const Deformation & node, bool inverse)
	: _r(node._r)
	, _t(node._t)
	, _w(node._w)
{
	if (inverse) {
		_r = node._r.getInverse();
		_t = -node._t;
	}
}

}