#pragma once

#include "algo/registration/deformation_graph/i_deformation.h"
#include "mLibCore.h"

namespace Registration {

class EDDeformation
{
private:
	ml::mat3d _r; // rotation matrix
	ml::vec3d _t; // translation vector	
	double _w; // weight
	Point _position;
public:
	double * d() { return (&_r)->getData(); } // todo
	double * r() { return (&_r)->getData(); }
	double * t() { return (&_t)->getData(); }
	double * w() { return &_w; }
public:
	Matrix rotation() const;
	Vector translation() const;
	Point position() const;
	double weight() const { return _w; };
public:
	Point getDeformedPosition() const;
	Point deformPosition(const Point & point) const;
	Vector deformNormal(const Vector & normal) const;
public:
	EDDeformation invertDeformation() const;
public:
	EDDeformation(const Point & position);
	EDDeformation(const Point & position, const ml::mat3d & r, const ml::vec3d & t, double w = 1.);
	EDDeformation(const RigidDeformation & rigid_deformation);
	EDDeformation();
	EDDeformation(const EDDeformation& other);
	EDDeformation(const EDDeformation& node, bool inverse);
};

EDDeformation interpolateDeformations(Point position, std::vector<std::pair<EDDeformation, double>> deformation_weights_vector);

EDDeformation linearInterpolation(const EDDeformation & deformation0, const EDDeformation & deformation1, double t);

EDDeformation mean(const EDDeformation & d0, const EDDeformation & d1, const EDDeformation & d2);

}