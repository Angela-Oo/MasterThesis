#include "arap_deformation.h"
#include "mLibCore.h"
#include <ceres/rotation.h>

#include "algo/registration/util/math.h"
#include "algo/registration/util/dual_quaternion.h"

namespace Registration {

Matrix ARAPDeformation::rotation() const
{
	ml::mat3d r;
	ceres::AngleAxisToRotationMatrix(_d.array, r.getData());
	auto r_t =  r.getTranspose(); // why??
	Matrix m = convertMatrix(r_t);
	//Matrix m(r_t(0, 0), r_t(0, 1), r_t(0, 2), r_t(1, 0), r_t(1, 1), r_t(1, 2), r_t(2, 0), r_t(2, 1), r_t(2, 2));
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

DualQuaternion ARAPDeformation::deformDLBPosition() const
{
	Matrix r = rotation();
	Vector t = translation();
	ml::mat3d rotation_m = convertMatrix(r);
	ml::vec3d translation = convertVector(t);

	ml::mat4d transformation(rotation_m, translation);	
	DualQuaternion dual(transformation);

	return dual;
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


ARAPDeformation interpolateDeformations(Point position, std::vector<std::pair<ARAPDeformation, double>> deformation_weights_vector)
{
	ml::vec6d d;
	//for (auto d_w : deformation_weights_vector)
	//{
	//	auto deformation = d_w.first;
	//	double weight = d_w.second;
	//	
	//	d += deformation.deformation() * weight;
	//}
	// set rotation to zero
	d[0] = 0.;
	d[1] = 0.;
	d[2] = 0.;
	for (auto d_w : deformation_weights_vector)
	{
		auto deformation = d_w.first.deformation();
		double weight = d_w.second;	
		d += (deformation * weight);
	}

	Vector deformed_point(0., 0., 0.);
	for (auto d_w : deformation_weights_vector)
	{
		auto deformation = d_w.first;
		double weight = d_w.second;
		Vector transformed_point = deformation.deformPosition(position) - CGAL::ORIGIN;
		transformed_point *= weight;
		deformed_point += transformed_point;
	}
	Vector translation = deformed_point - (position - CGAL::ORIGIN);
	d[3] = translation.x();
	d[4] = translation.y();
	d[5] = translation.z();
	return ARAPDeformation(position, d);
}


ARAPDeformation linearInterpolation(const ARAPDeformation & deformation0, const ARAPDeformation & deformation1, double t)
{
	Vector dir = deformation1.position() - deformation0.position();
	Point position = deformation0.position() + (dir * t);

	auto d = (deformation0.deformation() + deformation1.deformation()) * t;
	//auto d = deformation0.deformation() + ((deformation1.deformation() - deformation0.deformation()) * t);
	return ARAPDeformation(position , d);
}


ARAPDeformation mean(const ARAPDeformation & d0, const ARAPDeformation & d1, const ARAPDeformation & d2)
{
	Vector position = ((d0.position() - CGAL::ORIGIN) + (d1.position() - CGAL::ORIGIN) + (d2.position() - CGAL::ORIGIN)) / 3.;	
	auto d = (d0.deformation() + d1.deformation() + d2.deformation()) / 3.;
	return ARAPDeformation(CGAL::ORIGIN + position, d);
}

}