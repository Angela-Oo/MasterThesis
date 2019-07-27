#include "rigid_factory.h"
#include "algo/registration/util/log_option.h"

namespace Registration {

std::unique_ptr<RigidRegistration> RigidFactory::operator()(const SurfaceMesh & source,
														    const SurfaceMesh & target)
{
	return std::make_unique<RigidRegistration>(source, target, _ceres_options, _options, _logger);
}

std::unique_ptr<RigidRegistration> RigidFactory::operator()(const SurfaceMesh & source,
														    const SurfaceMesh & target,
														    const RigidDeformation & deformation)
{
	return std::make_unique<RigidRegistration>(source, target, deformation, _ceres_options, _options, _logger);
}

std::unique_ptr<RigidRegistration> RigidFactory::operator()(const SurfaceMesh & source,
															const SurfaceMesh & target,
															const SurfaceMesh & previous_mesh,
															const RigidDeformation & deformation)
{
	return std::make_unique<RigidRegistration>(source, target, previous_mesh, deformation, _ceres_options, _options, _logger);
}


SurfaceMesh RigidFactory::deformationGraphMesh(const RigidDeformation & deformation)
{
	return SurfaceMesh();
}

SurfaceMesh RigidFactory::deformedMesh(const SurfaceMesh & mesh, const RigidDeformation & deformation)
{
	RigidDeformedMesh deformed(mesh, deformation);
	return deformed.deformPoints();
}


std::string RigidFactory::registrationType()
{
	return "rigid";
}

void RigidFactory::logConfiguration()
{
	logRegistrationOptions(_logger, _options);
	logCeresOptions(_logger, _ceres_options);
}

RigidFactory::RigidFactory(const RegistrationOptions & options,
						   const ceres::Solver::Options & ceres_options,
						   std::shared_ptr<FileWriter> logger)
	: _options(options)
	, _ceres_options(ceres_options)
	, _logger(logger)
{}


}