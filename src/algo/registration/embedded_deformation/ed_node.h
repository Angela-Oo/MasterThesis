#pragma once
#include "mLibInclude.h"
#include "algo/registration/deformation_graph/i_node.h"

namespace ED {

class Deformation : public INode
{
private:
	ml::mat3d _r; // rotation matrix
	ml::vec3d _t; // translation vector	
	double _w; // weight
public:
	double * r() override { return (&_r)->getData(); }
	double * t() override { return (&_t)->getData(); }
	double * w() override { return &_w; }
public:
	Matrix rotation() const override;
	Vector translation() const override;
	double weight() const override { return _w; };
public:
	Deformation(const ml::mat3d & r, const ml::vec3d & t);
	Deformation();
	Deformation(const Deformation& node, bool inverse);
};


}
