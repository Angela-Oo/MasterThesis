#pragma once


#include "algo/file_writer.h"
#include <ceres/ceres.h>
#include "algo/surface_mesh/mesh_definition.h"
#include "rigid_before_non_rigid_registration.h"
#include "algo/registration/rigid_registration/rigid_factory.h"
#include "algo/registration/arap/arap_factory.h"

namespace Registration {

class RigidBeforeNonRigidRegistrationFactory
{
	RigidFactory _rigid_factory;
	ARAPFactory _arap_factory;
	ceres::Solver::Options _ceres_options;
	RegistrationOptions _options;
	std::shared_ptr<FileWriter> _logger;
	std::vector<vertex_descriptor> _fixed_positions;
public:
	std::unique_ptr<RigidBeforeNonRigidRegistration> operator()(const SurfaceMesh & source,
																const SurfaceMesh & target);
	std::unique_ptr<RigidBeforeNonRigidRegistration> operator()(const SurfaceMesh & source,
																const SurfaceMesh & target,
																const DeformationGraph & deformation_graph);
	SurfaceMesh deformationGraphMesh(const DeformationGraph & deformation_graph);
	SurfaceMesh deformedMesh(const SurfaceMesh & mesh, const DeformationGraph & deformation_graph);
	SurfaceMesh inverseDeformedMesh(const SurfaceMesh & mesh, const DeformationGraph & deformation_graph);
	void setFixedPositions(std::vector<vertex_descriptor> fixed_positions);
	std::string registrationType();
	void logConfiguration();
public:
	RigidBeforeNonRigidRegistrationFactory(const RegistrationOptions & options,
										   const ceres::Solver::Options & ceres_options,
										   std::shared_ptr<FileWriter> logger);
};

}