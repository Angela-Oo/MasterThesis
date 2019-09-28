#pragma once

#include "algo/registration/deformation_graph/i_deformation.h"
#include "mLibCore.h"
#include "algo/registration/util/dual_quaternion.h"

namespace Registration {

class EDDeformation
{
private:
	std::array<double, 12> _d; // translation and rotation matrix
	double _w; // weight
	Point _position;
public:
	double * d() { return _d.data(); }
	double * w() { return &_w; }
private:
	void setDeformation(Matrix r, Vector t);
public:
	Matrix rotation() const;
	Vector translation() const;
	Point position() const;
	double weight() const { return _w; };
public:
	Point getDeformedPosition() const;
	Point deformPosition(const Point & point) const;
	DualQuaternion deformDLBPosition() const { return DualQuaternion(); };
	Vector deformNormal(const Vector & normal) const;
public:
	EDDeformation invertDeformation() const;
public:
	EDDeformation(const Point & position);
	EDDeformation(const Point & position, const Matrix & r, const Vector & t, double w = 1.);
	EDDeformation(const RigidDeformation & rigid_deformation);
	EDDeformation();
	EDDeformation(const EDDeformation& other);
	EDDeformation(const EDDeformation& node, bool inverse);
};

EDDeformation interpolateDeformations(Point position, std::vector<std::pair<EDDeformation, double>> deformation_weights_vector);

EDDeformation linearInterpolation(const EDDeformation & deformation0, const EDDeformation & deformation1, double t);

EDDeformation mean(const EDDeformation & d0, const EDDeformation & d1, const EDDeformation & d2);

}