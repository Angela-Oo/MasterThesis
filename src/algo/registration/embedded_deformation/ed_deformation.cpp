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

std::shared_ptr<IDeformation> Deformation::invertDeformation() const
{	
	return std::make_shared<Deformation>(_r.getInverse(), -_t, _w);
}

std::shared_ptr<IDeformation> Deformation::clone() const
{
	return std::make_shared<Deformation>(*this);
}

Deformation::Deformation(const ml::mat3d & r, const ml::vec3d & t, double w)
	: _r(r)
	, _t(t)
	, _w(w)
{}

Deformation::Deformation()
	: Deformation(ml::mat3d::identity(), ml::vec3f::origin)
{}

Deformation::Deformation(const Deformation & other)
	: _r(other._r)
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


std::shared_ptr<Deformation> createDeformation()
{
	return std::make_shared<Deformation>();
}

}