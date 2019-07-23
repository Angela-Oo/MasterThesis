#pragma once
#include "mLibInclude.h"
#include "algo/registration/deformation_graph/i_deformation.h"

namespace Registration {
namespace ED {

class Deformation : public IDeformation
{
private:
	ml::mat3d _r; // rotation matrix
	ml::vec3d _t; // translation vector	
	double _w; // weight
public:
	double * d() override { return (&_r)->getData(); } // todo
	double * r() override { return (&_r)->getData(); }
	double * t() override { return (&_t)->getData(); }
	double * w() override { return &_w; }
public:
	Matrix rotation() const override;
	Vector translation() const override;
	double weight() const override { return _w; };
	std::shared_ptr<IDeformation> invertDeformation() const override;
	std::shared_ptr<IDeformation> clone() const override;
public:
	Deformation(const ml::mat3d & r, const ml::vec3d & t, double w = 1.);
	Deformation();
	Deformation(const Deformation& other);
	Deformation(const Deformation& node, bool inverse);
};

std::shared_ptr<Deformation> createDeformation();

}
}