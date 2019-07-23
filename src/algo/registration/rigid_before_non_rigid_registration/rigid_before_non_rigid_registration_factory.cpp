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
												   const DeformationGraph & deformation_graph)
{
	return std::make_unique<RigidBeforeNonRigidRegistration>(_rigid_factory(source, target),
															 _arap_factory(source, target, deformation_graph));
}

void RigidBeforeNonRigidRegistrationFactory::setFixedPositions(std::vector<vertex_descriptor> fixed_positions)
{
	_fixed_positions = fixed_positions;
}

SurfaceMesh RigidBeforeNonRigidRegistrationFactory::deformationGraphMesh(const DeformationGraph & deformation)
{
	return deformationGraphToSurfaceMesh(deformation, _options.evaluate_residuals);
}

SurfaceMesh RigidBeforeNonRigidRegistrationFactory::deformedMesh(const SurfaceMesh & mesh, const DeformationGraph & deformation)
{
	DeformedMesh deformed(mesh, deformation, _options.dg_options.number_of_interpolation_neighbors);
	return deformed.deformPoints();
}

SurfaceMesh RigidBeforeNonRigidRegistrationFactory::inverseDeformedMesh(const SurfaceMesh & mesh, const DeformationGraph & deformation_graph)
{
	auto inverse_deformation = invertDeformationGraph(deformation_graph);
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