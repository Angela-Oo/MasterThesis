#include "ed_deformation.h"

namespace Registration {

void EDDeformation::setDeformation(Matrix r, Vector t)
{
	_d[0] = r.m(0, 0);
	_d[1] = r.m(0, 1);
	_d[2] = r.m(0, 2);
	_d[3] = r.m(1, 0);
	_d[4] = r.m(1, 1);
	_d[5] = r.m(1, 2);
	_d[6] = r.m(2, 0);
	_d[7] = r.m(2, 1);
	_d[8] = r.m(2, 2);
	_d[9] = t[0];
	_d[10] = t[1];
	_d[11] = t[2];
}

Matrix EDDeformation::rotation() const
{
	Matrix m(_d[0], _d[1], _d[2], _d[3], _d[4], _d[5], _d[6], _d[7], _d[8]);
	return m;
}

Vector EDDeformation::translation() const
{
	return Vector(_d[9], _d[10], _d[11]);
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
	auto r = rotation().inverse();
	auto t = -translation();
	return EDDeformation(getDeformedPosition(), r, t, _w);
}

EDDeformation::EDDeformation(const Point & position, const Matrix & r, const Vector & t, double w)
	: _position(position)
	, _w(w)
{
	setDeformation(r, t);
}

EDDeformation::EDDeformation()
	: EDDeformation(CGAL::ORIGIN, Matrix(), Vector())
{}

EDDeformation::EDDeformation(const Point & position)
	: EDDeformation(position, Matrix(), Vector())
{}

EDDeformation::EDDeformation(const RigidDeformation & rigid_deformation)
{;
	setDeformation(rigid_deformation.rotation(), rigid_deformation.translation());
	_position = rigid_deformation._g;
}

EDDeformation::EDDeformation(const EDDeformation & other)
	: _position(other._position)
	, _d(other._d)
	, _w(other._w)
{}

EDDeformation::EDDeformation(const EDDeformation & deformation, bool inverse)
	: EDDeformation(deformation)
{
	if (inverse) {
		auto r = deformation.rotation().inverse();
		auto t = -deformation.translation();
		setDeformation(r, t);
	}
}

EDDeformation interpolateDeformations(Point position, std::vector<std::pair<EDDeformation, double>> deformation_weights_vector)
{
	// TODO
	//ml::vec6d d;
	//for (auto d_w : deformation_weights_vector)
	//{
	//	auto deformation = d_w.first;
	//	double weight = d_w.second;

	//	d += deformation.deformation() * weight;
	//}
	return EDDeformation(position);// , d);
}



EDDeformation linearInterpolation(const EDDeformation & deformation0, const EDDeformation & deformation1, double t)
{
	Vector dir = deformation1.position() - deformation0.position();
	Point position = deformation0.position() + dir * t;

	//auto d = (deformation0.deformation() + deformation1.deformation()) * t;// TODO
	return EDDeformation(position);
}

EDDeformation mean(const EDDeformation & d0, const EDDeformation & d1, const EDDeformation & d2)
{
	Vector position = ((d0.position() - CGAL::ORIGIN) + (d1.position() - CGAL::ORIGIN) + (d2.position() - CGAL::ORIGIN)) / 3.;
	//auto d = (d0.deformation() + d1.deformation() + d2.deformation()) / 3.;
	return EDDeformation(CGAL::ORIGIN + position);// , d);
}

}