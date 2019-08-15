#pragma once


#include "rigid_before_non_rigid_registration.h"
#include "rigid_before_non_rigid_deformation.h"
#include "util/file_writer.h"
#include "mesh/mesh_definition.h"
#include "algo/registration/rigid_registration/rigid_factory.h"
#include "algo/registration/arap/arap_factory.h"
#include "algo/registration/util/log_option.h"
#include <ceres/ceres.h>

namespace Registration {

template<typename Factory>
class RigidBeforeNonRigidRegistrationFactory
{
public:	
	using Registration = typename RigidBeforeNonRigidRegistration<typename Factory::Registration>;
private:
	using NonRigidRegistration = typename Factory::Registration;
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
																					  const typename Registration::Deformation & deformation_graph);
	std::unique_ptr<RigidBeforeNonRigidRegistration<NonRigidRegistration>> operator()(const SurfaceMesh & source,
																					  const SurfaceMesh & target,
																					  const SurfaceMesh & previous_mesh, // used for non rigid registration
																					  const typename Registration::Deformation & deformation_graph);
	void setFixedPositions(std::vector<vertex_descriptor> fixed_positions);
	std::string registrationType();
public:
	RigidBeforeNonRigidRegistrationFactory(const RegistrationOptions & options,
										   const ceres::Solver::Options & ceres_options,
										   std::shared_ptr<FileWriter> logger);
};


template<typename Factory>
std::unique_ptr<RigidBeforeNonRigidRegistration<typename Factory::Registration>>
RigidBeforeNonRigidRegistrationFactory<Factory>::operator()(const SurfaceMesh & source, const SurfaceMesh & target)
{
	return std::make_unique <RigidBeforeNonRigidRegistration<NonRigidRegistration>>(_rigid_factory(source, target),
																					_non_rigid_factory(source, target));
}

template<typename Factory>
std::unique_ptr<RigidBeforeNonRigidRegistration<typename Factory::Registration>>
RigidBeforeNonRigidRegistrationFactory<Factory>::operator()(const SurfaceMesh & source,
															const SurfaceMesh & target,
															const typename RigidBeforeNonRigidRegistration<typename Factory::Registration>::Deformation & deformation)
{
	// new calc rigid deformation 
	//return std::make_unique<RigidBeforeNonRigidRegistration<NonRigidRegistration>>(_rigid_factory(source, target),
	//																			   _non_rigid_factory(source, target, deformation.non_rigid_deformation));

	// use previous rigid deformation for init rigid deformation
	return std::make_unique<RigidBeforeNonRigidRegistration<NonRigidRegistration>>(_rigid_factory(source, target, deformation.rigid_deformation),
																				   _non_rigid_factory(source, target, deformation.non_rigid_deformation));
}

template<typename Factory>
std::unique_ptr<RigidBeforeNonRigidRegistration<typename Factory::Registration>>
RigidBeforeNonRigidRegistrationFactory<Factory>::operator()(const SurfaceMesh & source,
															const SurfaceMesh & target,
															const SurfaceMesh & previous_mesh,
															const typename RigidBeforeNonRigidRegistration<typename Factory::Registration>::Deformation & deformation)
{
	if (_options.sequence_options.init_rigid_deformation_with_non_rigid_globale_deformation) {
		return std::make_unique<RigidBeforeNonRigidRegistration<NonRigidRegistration>>(_rigid_factory(source, target, previous_mesh, deformation.non_rigid_deformation.getRigidDeformation()),
																					   _non_rigid_factory(source, target, deformation.non_rigid_deformation));
	}
	else {
		return std::make_unique<RigidBeforeNonRigidRegistration<NonRigidRegistration>>(_rigid_factory(source, target, previous_mesh, deformation.rigid_deformation),
																					   _non_rigid_factory(source, target, deformation.non_rigid_deformation));
	}
}

template<typename Factory>
void RigidBeforeNonRigidRegistrationFactory<Factory>::setFixedPositions(std::vector<vertex_descriptor> fixed_positions)
{
	_fixed_positions = fixed_positions;
}

template<typename Factory>
std::string RigidBeforeNonRigidRegistrationFactory<Factory>::registrationType()
{
	return "rigid and non rigid registration";
}

template<typename Factory>
RigidBeforeNonRigidRegistrationFactory<Factory>::RigidBeforeNonRigidRegistrationFactory(const RegistrationOptions & options,
																						const ceres::Solver::Options & ceres_options,
																						std::shared_ptr<FileWriter> logger)
	: _options(options)
	, _ceres_options(ceres_options)
	, _logger(logger)
	, _rigid_factory(options, ceres_options, logger)
	, _non_rigid_factory(options, ceres_options, logger)
{ }



}