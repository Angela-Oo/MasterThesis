#pragma once

#include "algo/registration/i_registration.h"
#include "algo/surface_mesh/mesh_definition.h"
#include "algo/registration/deformation_graph/deformation_graph.h"

namespace Registration {

class RigidBeforeNonRigidRegistration : public IRegistration
{
private:	
	std::unique_ptr<IRegistration> _rigid_registration;
	std::unique_ptr<IRegistration> _non_rigid_registration;
	std::function<std::unique_ptr<IRegistration>(const DG::DeformationGraph & deformation_graph)> _create_non_rigid_registration;
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
	SurfaceMesh getDeformationGraphMesh() override;
public:
	const DG::DeformationGraph & getDeformationGraph() override;
public:
	// without icp
	RigidBeforeNonRigidRegistration(std::unique_ptr<IRegistration> _rigid_registration,
									std::function<std::unique_ptr<IRegistration>(const DG::DeformationGraph &)> create_non_rigid_registration);
};

}