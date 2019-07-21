#pragma once

#include "algo/registration/i_registration.h"
#include "algo/surface_mesh/mesh_definition.h"
#include "algo/registration/deformation_graph/deformation_graph.h"

namespace Registration {

class RigidBeforeNonRigidRegistration : public INonRigidRegistration
{
private:	
	std::unique_ptr<IRigidRegistration> _rigid_registration;
	std::unique_ptr<INonRigidRegistration> _non_rigid_registration;
	bool _finished_rigid_registration;
private:
	RegistrationOptions _registration_options;
public:
	bool finished() override;
	bool solveIteration() override;
	size_t currentIteration() override;
	bool solve() override;
public:
	const SurfaceMesh & getSource() override;
	const SurfaceMesh & getTarget() override;
	SurfaceMesh getDeformedPoints() override;
	SurfaceMesh getInverseDeformedPoints() override;	
public:
	SurfaceMesh getDeformationGraphMesh() override;
	const DG::DeformationGraph & getDeformationGraph() override;
	void setRigidDeformation(const RigidDeformation & rigid_deformation) override;
	bool shouldBeSavedAsImage() override;
public:
	// without icp
	RigidBeforeNonRigidRegistration(std::unique_ptr<IRigidRegistration> rigid_registration,
									std::unique_ptr<INonRigidRegistration> non_rigid_registration);
};

}