#include "rigid_before_non_rigid_registration_factory.h"
#include "algo/registration/helper/log_option.h"

namespace Registration {


std::unique_ptr<RigidBeforeNonRigidRegistration> 
RigidBeforeNonRigidRegistrationFactory::operator()(const SurfaceMesh & source, const SurfaceMesh & target)
{
	return std::make_unique <RigidBeforeNonRigidRegistration>(_rigid_factory(source, target),
															  _arap_factory(source, target));
}

std::unique_ptr<RigidBeforeNonRigidRegistration>
RigidBeforeNonRigidRegistrationFactory::operator()(const SurfaceMesh & source,
												   const SurfaceMesh & target,
												   const RigidBeforeNonRigidDeformation & deformation)
{
	return std::make_unique<RigidBeforeNonRigidRegistration>(_rigid_factory(source, target),
															 _arap_factory(source, target, deformation.non_rigid_deformation));
}

void RigidBeforeNonRigidRegistrationFactory::setFixedPositions(std::vector<vertex_descriptor> fixed_positions)
{
	_fixed_positions = fixed_positions;
}

SurfaceMesh RigidBeforeNonRigidRegistrationFactory::deformationGraphMesh(const RigidBeforeNonRigidDeformation & deformation)
{
	if (deformation.is_rigid_deformation) {
		return _arap_factory.deformationGraphMesh(deformation.non_rigid_deformation);
	}
	else {
		return SurfaceMesh();
	}
}

SurfaceMesh RigidBeforeNonRigidRegistrationFactory::deformedMesh(const SurfaceMesh & mesh, const RigidBeforeNonRigidDeformation & deformation)
{
	if (deformation.is_rigid_deformation) {
		return _arap_factory.deformedMesh(mesh, deformation.non_rigid_deformation);
	}
	else {
		return _rigid_factory.deformedMesh(mesh, deformation.rigid_deformation);
	}
}

SurfaceMesh RigidBeforeNonRigidRegistrationFactory::inverseDeformedMesh(const SurfaceMesh & mesh, const RigidBeforeNonRigidDeformation & deformation)
{
	auto inverse_deformation = invertDeformationGraph(deformation.non_rigid_deformation);
	DeformedMesh deformed(mesh, inverse_deformation, _options.dg_options.number_of_interpolation_neighbors);
	return deformed.deformPoints();
}

std::string RigidBeforeNonRigidRegistrationFactory::registrationType()
{
	return "rigid and non rigid registration";
}

void RigidBeforeNonRigidRegistrationFactory::logConfiguration()
{
	logRegistrationOptions(_logger, _options);
	logCeresOptions(_logger, _ceres_options);
}

RigidBeforeNonRigidRegistrationFactory::RigidBeforeNonRigidRegistrationFactory(const RegistrationOptions & options,
																			   const ceres::Solver::Options & ceres_options,
																			   std::shared_ptr<FileWriter> logger)
	: _options(options)
	, _ceres_options(ceres_options)
	, _logger(logger)
	, _rigid_factory(options, ceres_options, logger)
	, _arap_factory(options, ceres_options, logger)
{

}


}