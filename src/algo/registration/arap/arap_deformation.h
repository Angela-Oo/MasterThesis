#pragma once
#include "mLibInclude.h"
#include "algo/registration/deformation_graph/i_deformation.h"

namespace Registration {

class Deformation : public IPositionDeformation
{
private:
	ml::vec6d _d; // rotation (3d) and translation (3d)
	//ml::vec3d _r; // rotation matrix
	//ml::vec3d _t; // translation vector	
	double _w; // weight
public:
	double * d() override { return (&_d)->getData(); }
	double * r() override { return (&_d)->getData(); }
	double * t() override { return &((&_d)->getData())[3]; }
	double * w() override { return &_w; }
public:
	Matrix rotation() const override;
	Vector translation() const override;
	double weight() const override { return _w; };
	std::shared_ptr<IPositionDeformation> invertDeformation() const override;
	std::shared_ptr<IPositionDeformation> clone() const override;
public:
	Deformation(const ml::vec3d & r, const ml::vec3d & t, double w = 1.);
	Deformation(const ml::vec6d & d, double w = 1.);
	Deformation();
	Deformation(const Deformation& other);
	Deformation(const Deformation& deformation, bool inverse);
};


std::shared_ptr<Deformation> createDeformation();

}
