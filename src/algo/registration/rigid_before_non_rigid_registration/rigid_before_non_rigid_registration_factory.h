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

template<typename NonRigidRegistration>
class RigidBeforeNonRigidRegistrationFactory
{
public:	
	using Registration = typename RigidBeforeNonRigidRegistration<typename NonRigidRegistration>;
private:

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


template<typename NonRigidRegistration>
std::unique_ptr<RigidBeforeNonRigidRegistration<NonRigidRegistration>>
RigidBeforeNonRigidRegistrationFactory<NonRigidRegistration>::operator()(const SurfaceMesh & source, const SurfaceMesh & target)
{
	auto rigid_registration = std::make_unique<RigidRegistration>(source, target, _ceres_options, _options, _logger);
	auto non_rigid_registration = std::make_unique<NonRigidRegistration>(source, target, _ceres_options, _options, _logger);
	return std::make_unique<RigidBeforeNonRigidRegistration<NonRigidRegistration>>(std::move(rigid_registration), std::move(non_rigid_registration));
}

template<typename NonRigidRegistration>
std::unique_ptr<RigidBeforeNonRigidRegistration<NonRigidRegistration>>
RigidBeforeNonRigidRegistrationFactory<NonRigidRegistration>::operator()(const SurfaceMesh & source,
															const SurfaceMesh & target,
															const typename RigidBeforeNonRigidRegistration<typename NonRigidRegistration>::Deformation & deformation)
{
	// new calc rigid deformation 
	//return std::make_unique<RigidBeforeNonRigidRegistration<NonRigidRegistration>>(_rigid_factory(source, target),
	//																			   _non_rigid_factory(source, target, deformation.non_rigid_deformation));

	// use previous rigid deformation for init rigid deformation
	auto rigid_registration = std::make_unique<RigidRegistration>(source, target, deformation.rigid_deformation, _ceres_options, _options, _logger);
	auto non_rigid_registration = std::make_unique<NonRigidRegistration>(source, target, deformation.non_rigid_deformation, _ceres_options, _options, _logger);
	return std::make_unique<RigidBeforeNonRigidRegistration<NonRigidRegistration>>(std::move(rigid_registration), std::move(non_rigid_registration));
}

template<typename NonRigidRegistration>
std::unique_ptr<RigidBeforeNonRigidRegistration<NonRigidRegistration>>
RigidBeforeNonRigidRegistrationFactory<NonRigidRegistration>::operator()(const SurfaceMesh & source,
															const SurfaceMesh & target,
															const SurfaceMesh & previous_mesh,
															const typename RigidBeforeNonRigidRegistration<typename NonRigidRegistration>::Deformation & deformation)
{
	std::unique_ptr<RigidRegistration> rigid_registration;
	if (_options.sequence_options.init_rigid_deformation_with_non_rigid_globale_deformation) {
		rigid_registration = std::make_unique<RigidRegistration>(source, target, previous_mesh, deformation.non_rigid_deformation.getRigidDeformation(), _ceres_options, _options, _logger);
	}
	else {
		rigid_registration = std::make_unique<RigidRegistration>(source, target, previous_mesh, deformation.rigid_deformation, _ceres_options, _options, _logger);
	}
	auto non_rigid_registration = std::make_unique<NonRigidRegistration>(source, target, deformation.non_rigid_deformation, _ceres_options, _options, _logger);
	return std::make_unique<RigidBeforeNonRigidRegistration<NonRigidRegistration>>(std::move(rigid_registration), std::move(non_rigid_registration));
}

template<typename NonRigidRegistration>
void RigidBeforeNonRigidRegistrationFactory<NonRigidRegistration>::setFixedPositions(std::vector<vertex_descriptor> fixed_positions)
{
	_fixed_positions = fixed_positions;
}

template<typename NonRigidRegistration>
std::string RigidBeforeNonRigidRegistrationFactory<NonRigidRegistration>::registrationType()
{
	return "rigid and non rigid registration";
}

template<typename NonRigidRegistration>
RigidBeforeNonRigidRegistrationFactory<NonRigidRegistration>::RigidBeforeNonRigidRegistrationFactory(const RegistrationOptions & options,
																						const ceres::Solver::Options & ceres_options,
																						std::shared_ptr<FileWriter> logger)
	: _options(options)
	, _ceres_options(ceres_options)
	, _logger(logger)
	//, _rigid_factory(options, ceres_options, logger)
	//, _non_rigid_factory(options, ceres_options, logger)
{ }



}