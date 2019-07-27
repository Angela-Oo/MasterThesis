#pragma once


#include "algo/file_writer.h"
#include <ceres/ceres.h>
#include "algo/surface_mesh/mesh_definition.h"
#include "rigid_registration.h"

namespace Registration {

class RigidFactory
{
public:
	using Registration = RigidRegistration;
	using DeformMesh = RigidDeformedMesh;
private:
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
	std::unique_ptr<RigidRegistration> operator()(const SurfaceMesh & source,
												  const SurfaceMesh & target,
												  const SurfaceMesh & previous_mesh, // used for non rigid registration
												  const RigidDeformation & deformation);
	std::string registrationType();
public:
	RigidFactory(const RegistrationOptions & options,
				 const ceres::Solver::Options & ceres_options,
				 std::shared_ptr<FileWriter> logger);
};


}