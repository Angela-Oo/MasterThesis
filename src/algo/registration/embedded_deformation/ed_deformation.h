#pragma once
#include "mLibInclude.h"
#include "algo/registration/deformation_graph/i_deformation.h"

namespace Registration {
namespace ED {

class Deformation : public IPositionDeformation
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
	Matrix rotation() const override;
	Vector translation() const override;
	Point position() const override;
	double weight() const { return _w; };
public:
	Point getDeformedPosition() const override;
	Point deformPosition(const Point & point) const override;
	Vector deformNormal(const Vector & normal) const override;
public:
	Deformation invertDeformation() const;
public:
	Deformation(const Point & position);
	Deformation(const Point & position, const ml::mat3d & r, const ml::vec3d & t, double w = 1.);
	Deformation(const RigidDeformation & rigid_deformation);
	Deformation();
	Deformation(const Deformation& other);
	Deformation(const Deformation& node, bool inverse);
};


}
}