#include "stdafx.h"

#include "registration.h"
#include "ceres_option.h"
#include "algo/registration/rigid_registration/rigid_registration.h"
#include "algo/registration/embedded_deformation/ed.h"
#include "algo/registration/arap/arap.h"

std::unique_ptr<IRegistration> createRegistration(const SurfaceMesh & source, 
												  const SurfaceMesh & target, 												  
												  RegistrationType registration_type,
												  const ceres::Solver::Options & options,
												  RegistrationOptions registration_options,
												  std::shared_ptr<FileWriter> logger,
												  std::vector<vertex_descriptor> fixed_positions)
{
	if (registration_type == RegistrationType::ED)
		return std::make_unique<ED::EmbeddedDeformation>(source, target, options, registration_options.dg_options.edge_length, registration_options.evaluate_residuals, logger);
	else if (registration_type == RegistrationType::ED_WithoutICP)
		return std::make_unique<ED::EmbeddedDeformation>(source, target, fixed_positions, options, registration_options.evaluate_residuals, logger);
	else if (registration_type == RegistrationType::ARAP)
		return std::make_unique<ARAP::AsRigidAsPossible>(source, target, options, registration_options, logger);
	else if (registration_type == RegistrationType::ARAP_WithoutICP)
		return std::make_unique<ARAP::AsRigidAsPossible>(source, target, fixed_positions, options, registration_options, logger);
	else
		return std::make_unique<RigidRegistration>(source, target, options, logger);
}


std::unique_ptr<IRegistration> createRegistration(const SurfaceMesh & source,
												  const SurfaceMesh & target,
												  RegistrationType registration_type,
												  DG::DeformationGraph deformation_graph,
												  const ceres::Solver::Options & options,
												  RegistrationOptions registration_options,
												  std::shared_ptr<FileWriter> logger)
{
	if (registration_type == RegistrationType::ED)
		return std::make_unique<ED::EmbeddedDeformation>(source, target, deformation_graph, options, registration_options.evaluate_residuals, logger);
	else if (registration_type == RegistrationType::ARAP)
		return std::make_unique<ARAP::AsRigidAsPossible>(source, target, deformation_graph, options, registration_options, logger);
	//else if (registration_type == RegistrationType::Rigid)
	//	return std::make_unique<RigidRegistration>(source, target, options, logger);
	else
		return nullptr;
}

