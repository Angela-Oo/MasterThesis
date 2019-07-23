#pragma once


#include "algo/file_writer.h"
#include <ceres/ceres.h>
#include "algo/surface_mesh/mesh_definition.h"
#include "rigid_registration.h"

namespace Registration {

class RigidFactory
{
	ceres::Solver::Options _ceres_options;
	RegistrationOptions _options;
	std::shared_ptr<FileWriter> _logger;
	std::vector<vertex_descriptor> _fixed_positions;
public:
	std::unique_ptr<RigidRegistration> operator()(const SurfaceMesh & source,
												  const SurfaceMesh & target);
	std::unique_ptr<RigidRegistration> operator()(const SurfaceMesh & source,
												  const SurfaceMesh & target,
												  const RigidDeformation & deformation);
	SurfaceMesh deformationGraphMesh(const RigidDeformation & deformation);
	SurfaceMesh deformedMesh(const SurfaceMesh & mesh, const RigidDeformation & deformation);
	std::string registrationType();
	void logConfiguration();
public:
	RigidFactory(const RegistrationOptions & options,
				 const ceres::Solver::Options & ceres_options,
				 std::shared_ptr<FileWriter> logger);
};


}