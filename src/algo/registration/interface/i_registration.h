#pragma once

#include "algo/registration/rigid_registration/rigid_deformation.h"
#include "mesh/mesh_definition.h"
#include <vector>

namespace Registration {


class IRegistration
{
public:
	virtual bool finished() = 0;
	virtual bool solveIteration() = 0;
	virtual size_t currentIteration() = 0;
	virtual bool solve() = 0;
public:
	virtual const SurfaceMesh & getSource() = 0;
	virtual const SurfaceMesh & getTarget() = 0;
	virtual SurfaceMesh getDeformedPoints() = 0;
	virtual SurfaceMesh getInverseDeformedPoints() = 0;
public:
	virtual ~IRegistration() = default;
};


class INonRigidRegistration : public IRegistration
{
public:
	virtual void setRigidDeformation(const RigidDeformation & rigid_deformation) = 0;
	virtual std::vector<Point> getFixedPostions() { return std::vector<Point>(); }
	virtual SurfaceMesh getDeformationGraphMesh() = 0;
	virtual bool shouldBeSavedAsImage() = 0;
public:
	virtual ~INonRigidRegistration() = default;
};



}