#pragma once
#include "mLibInclude.h"
#include "algo/registration/deformation_graph/i_deformation.h"
#include "algo/registration/rigid_registration/rigid_deformation.h"

namespace Registration {

class ARAPDeformation
{
private:
	ml::vec6d _d; // rotation (3d) and translation (3d)
	double _w; // weight
	Point _position;
public:
	double * d() { return (&_d)->getData(); }
	double * w() { return &_w; }
public:
	Matrix rotation() const;
	Vector translation() const;
	Point position() const;
public:
	Point getDeformedPosition() const;
	Point deformPosition(const Point & point) const;
	Vector deformNormal(const Vector & normal) const;
public:
	double weight() const { return _w; };
	ARAPDeformation invertDeformation() const;
public:
	//ARAPDeformation(const Point & position, const ml::vec3d & r, const ml::vec3d & t, double w = 1.);
	ARAPDeformation(const Point & position, const ml::vec6d & d, double w = 1.);
	ARAPDeformation(const Point & position);
	ARAPDeformation(const RigidDeformation & rigid_deformation);
	ARAPDeformation();
	ARAPDeformation(const ARAPDeformation& other);
	ARAPDeformation(const ARAPDeformation& deformation, bool inverse);
};


//std::shared_ptr<ARAPDeformation> createDeformation();

}
