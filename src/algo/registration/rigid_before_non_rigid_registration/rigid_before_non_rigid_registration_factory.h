#pragma once


#include "algo/file_writer.h"
#include <ceres/ceres.h>
#include "algo/surface_mesh/mesh_definition.h"
#include "rigid_before_non_rigid_registration.h"
#include "algo/registration/rigid_registration/rigid_factory.h"
#include "algo/registration/helper/log_option.h"

namespace Registration {

template<typename NonRigidFactory>
class RigidBeforeNonRigidRegistrationFactory
{
	RigidFactory _rigid_factory;
	NonRigidFactory _non_rigid_factory;
	ceres::Solver::Options _ceres_options;
	RegistrationOptions _options;
	std::shared_ptr<FileWriter> _logger;
	std::vector<vertex_descriptor> _fixed_positions;
public:
	std::unique_ptr<RigidBeforeNonRigidRegistration> operator()(const SurfaceMesh & source,
																const SurfaceMesh & target);
	std::unique_ptr<RigidBeforeNonRigidRegistration> operator()(const SurfaceMesh & source,
																const SurfaceMesh & target,
																const RigidBeforeNonRigidDeformation & deformation_graph);
	SurfaceMesh deformationGraphMesh(const RigidBeforeNonRigidDeformation & deformation_graph);
	SurfaceMesh deformedMesh(const SurfaceMesh & mesh, const RigidBeforeNonRigidDeformation & deformation_graph);	
	void setFixedPositions(std::vector<vertex_descriptor> fixed_positions);
	std::string registrationType();
	void logConfiguration();
public:
	RigidBeforeNonRigidRegistrationFactory(const RegistrationOptions & options,
										   const ceres::Solver::Options & ceres_options,
										   std::shared_ptr<FileWriter> logger);
};



template<typename NonRigidFactory>
std::unique_ptr<RigidBeforeNonRigidRegistration>
RigidBeforeNonRigidRegistrationFactory<NonRigidFactory>::operator()(const SurfaceMesh & source, const SurfaceMesh & target)
{
	return std::make_unique <RigidBeforeNonRigidRegistration>(_rigid_factory(source, target),
															  _non_rigid_factory(source, target));
}

template<typename NonRigidFactory>
std::unique_ptr<RigidBeforeNonRigidRegistration>
RigidBeforeNonRigidRegistrationFactory<NonRigidFactory>::operator()(const SurfaceMesh & source,
												   const SurfaceMesh & target,
												   const RigidBeforeNonRigidDeformation & deformation)
{
	return std::make_unique<RigidBeforeNonRigidRegistration>(_rigid_factory(source, target),
															 _non_rigid_factory(source, target, deformation.non_rigid_deformation));
}

template<typename NonRigidFactory>
void RigidBeforeNonRigidRegistrationFactory<NonRigidFactory>::setFixedPositions(std::vector<vertex_descriptor> fixed_positions)
{
	_fixed_positions = fixed_positions;
}

template<typename NonRigidFactory>
SurfaceMesh RigidBeforeNonRigidRegistrationFactory<NonRigidFactory>::deformationGraphMesh(const RigidBeforeNonRigidDeformation & deformation)
{
	if (deformation.is_rigid_deformation) {
		return _non_rigid_factory.deformationGraphMesh(deformation.non_rigid_deformation);
	}
	else {
		return SurfaceMesh();
	}
}

template<typename NonRigidFactory>
SurfaceMesh RigidBeforeNonRigidRegistrationFactory<NonRigidFactory>::deformedMesh(const SurfaceMesh & mesh, const RigidBeforeNonRigidDeformation & deformation)
{
	if (deformation.is_rigid_deformation) {
		return _non_rigid_factory.deformedMesh(mesh, deformation.non_rigid_deformation);
	}
	else {
		return _rigid_factory.deformedMesh(mesh, deformation.rigid_deformation);
	}
}

template<typename NonRigidFactory>
std::string RigidBeforeNonRigidRegistrationFactory<NonRigidFactory>::registrationType()
{
	return "rigid and non rigid registration";
}

template<typename NonRigidFactory>
void RigidBeforeNonRigidRegistrationFactory<NonRigidFactory>::logConfiguration()
{
	logRegistrationOptions(_logger, _options);
	logCeresOptions(_logger, _ceres_options);
}

template<typename NonRigidFactory>
RigidBeforeNonRigidRegistrationFactory<NonRigidFactory>::RigidBeforeNonRigidRegistrationFactory(const RegistrationOptions & options,
																			   const ceres::Solver::Options & ceres_options,
																			   std::shared_ptr<FileWriter> logger)
	: _options(options)
	, _ceres_options(ceres_options)
	, _logger(logger)
	, _rigid_factory(options, ceres_options, logger)
	, _non_rigid_factory(options, ceres_options, logger)
{ }

}