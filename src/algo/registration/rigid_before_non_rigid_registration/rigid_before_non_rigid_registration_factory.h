#pragma once


#include "algo/file_writer.h"
#include <ceres/ceres.h>
#include "algo/surface_mesh/mesh_definition.h"
#include "rigid_before_non_rigid_registration.h"
#include "algo/registration/rigid_registration/rigid_factory.h"
#include "algo/registration/arap/arap_factory.h"
#include "algo/registration/helper/log_option.h"

namespace Registration {

template<typename NonRigidRegistration, typename Factory>
class RigidBeforeNonRigidRegistrationFactory
{
public:
	using Deformation = typename NonRigidRegistration::Deformation;
private:
	RigidFactory _rigid_factory;
	Factory _non_rigid_factory;
	ceres::Solver::Options _ceres_options;
	RegistrationOptions _options;
	std::shared_ptr<FileWriter> _logger;
	std::vector<vertex_descriptor> _fixed_positions;
public:
	std::unique_ptr<RigidBeforeNonRigidRegistration<NonRigidRegistration>> operator()(const SurfaceMesh & source,
																					  const SurfaceMesh & target);
	std::unique_ptr<RigidBeforeNonRigidRegistration<NonRigidRegistration>> operator()(const SurfaceMesh & source,
																					  const SurfaceMesh & target,
																					  const RigidBeforeNonRigidDeformation<Deformation> & deformation_graph);
	std::unique_ptr<RigidBeforeNonRigidRegistration<NonRigidRegistration>> operator()(const SurfaceMesh & source,
																					  const SurfaceMesh & target,
																					  const SurfaceMesh & previous_mesh, // used for non rigid registration
																					  const RigidBeforeNonRigidDeformation<Deformation> & deformation_graph);
	SurfaceMesh deformationGraphMesh(const RigidBeforeNonRigidDeformation<Deformation> & deformation_graph);
	SurfaceMesh deformedMesh(const SurfaceMesh & mesh, const RigidBeforeNonRigidDeformation<Deformation> & deformation_graph);
	void setFixedPositions(std::vector<vertex_descriptor> fixed_positions);
	std::string registrationType();
	void logConfiguration();
public:
	RigidBeforeNonRigidRegistrationFactory(const RegistrationOptions & options,
										   const ceres::Solver::Options & ceres_options,
										   std::shared_ptr<FileWriter> logger);
};




template<typename NonRigidRegistration, typename Factory>
std::unique_ptr<RigidBeforeNonRigidRegistration<NonRigidRegistration>>
RigidBeforeNonRigidRegistrationFactory<NonRigidRegistration, Factory>::operator()(const SurfaceMesh & source, const SurfaceMesh & target)
{
	return std::make_unique <RigidBeforeNonRigidRegistration<NonRigidRegistration>>(_rigid_factory(source, target),
																					_non_rigid_factory(source, target));
}

template<typename NonRigidRegistration, typename Factory>
std::unique_ptr<RigidBeforeNonRigidRegistration<NonRigidRegistration>>
RigidBeforeNonRigidRegistrationFactory<NonRigidRegistration, Factory>::operator()(const SurfaceMesh & source,
																				  const SurfaceMesh & target,
																				  const RigidBeforeNonRigidDeformation<Deformation> & deformation)
{
	// new calc rigid deformation 
	//return std::make_unique<RigidBeforeNonRigidRegistration<NonRigidRegistration>>(_rigid_factory(source, target),
	//																			   _non_rigid_factory(source, target, deformation.non_rigid_deformation));

	// use previous rigid deformation for init rigid deformation
	return std::make_unique<RigidBeforeNonRigidRegistration<NonRigidRegistration>>(_rigid_factory(source, target, deformation.rigid_deformation),
																				   _non_rigid_factory(source, target, deformation.non_rigid_deformation));
}

template<typename NonRigidRegistration, typename Factory>
std::unique_ptr<RigidBeforeNonRigidRegistration<NonRigidRegistration>>
RigidBeforeNonRigidRegistrationFactory<NonRigidRegistration, Factory>::operator()(const SurfaceMesh & source,
																				  const SurfaceMesh & target,
																				  const SurfaceMesh & previous_mesh,
																				  const RigidBeforeNonRigidDeformation<Deformation> & deformation)
{
	return std::make_unique<RigidBeforeNonRigidRegistration<NonRigidRegistration>>(_rigid_factory(previous_mesh, target, deformation.rigid_deformation),
																				   _non_rigid_factory(source, target, deformation.non_rigid_deformation));
}

template<typename NonRigidRegistration, typename Factory>
void RigidBeforeNonRigidRegistrationFactory<NonRigidRegistration, Factory>::setFixedPositions(std::vector<vertex_descriptor> fixed_positions)
{
	_fixed_positions = fixed_positions;
}

template<typename NonRigidRegistration, typename Factory>
SurfaceMesh RigidBeforeNonRigidRegistrationFactory<NonRigidRegistration, Factory>::deformationGraphMesh(const RigidBeforeNonRigidDeformation<Deformation> & deformation)
{
	if (deformation.is_rigid_deformation) {
		return _non_rigid_factory.deformationGraphMesh(deformation.non_rigid_deformation);
	}
	else {
		return SurfaceMesh();
	}
}

template<typename NonRigidRegistration, typename Factory>
SurfaceMesh RigidBeforeNonRigidRegistrationFactory<NonRigidRegistration, Factory>::deformedMesh(const SurfaceMesh & mesh, const RigidBeforeNonRigidDeformation<Deformation> & deformation)
{
	if (deformation.is_rigid_deformation) {
		return _non_rigid_factory.deformedMesh(mesh, deformation.non_rigid_deformation);
	}
	else {
		return _rigid_factory.deformedMesh(mesh, deformation.rigid_deformation);
	}
}

template<typename NonRigidRegistration, typename Factory>
std::string RigidBeforeNonRigidRegistrationFactory<NonRigidRegistration, Factory>::registrationType()
{
	return "rigid and non rigid registration";
}

template<typename NonRigidRegistration, typename Factory>
void RigidBeforeNonRigidRegistrationFactory<NonRigidRegistration, Factory>::logConfiguration()
{
	logRegistrationOptions(_logger, _options);
	logCeresOptions(_logger, _ceres_options);
}

template<typename NonRigidRegistration, typename Factory>
RigidBeforeNonRigidRegistrationFactory<NonRigidRegistration, Factory>::RigidBeforeNonRigidRegistrationFactory(const RegistrationOptions & options,
																											  const ceres::Solver::Options & ceres_options,
																											  std::shared_ptr<FileWriter> logger)
	: _options(options)
	, _ceres_options(ceres_options)
	, _logger(logger)
	, _rigid_factory(options, ceres_options, logger)
	, _non_rigid_factory(options, ceres_options, logger)
{ }



}