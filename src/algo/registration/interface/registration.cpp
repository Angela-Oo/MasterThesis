#include "stdafx.h"

#include "registration.h"
#include "algo/registration/options/ceres_option.h"
#include "algo/registration/rigid_registration/rigid_registration.h"
#include "algo/registration/embedded_deformation/ed.h"
#include "algo/registration/arap/arap.h"
#include "algo/registration/rigid_before_non_rigid_registration/rigid_before_non_rigid_registration.h"
#include "algo/registration/arap/arap_factory.h"
#include "algo/registration/embedded_deformation/ed_factory.h"
#include "algo/registration/rigid_registration/rigid_factory.h"
#include "algo/registration/rigid_before_non_rigid_registration/rigid_before_non_rigid_registration_factory.h"

#include "algo/registration/sequence_registration/sequence_registration.h"
#include "algo/registration/sequence_registration/refine_deformation_graph_registration_factory.h"
#include "algo/registration/sequence_registration/refine_deformation_graph_registration.h"

namespace Registration {

std::unique_ptr<ISequenceRegistration> createSequenceRegistration(RegistrationType type,
																  RegistrationOptions & options,
																  ceres::Solver::Options & ceres_options,
																  std::shared_ptr<FileWriter> logger,
																  std::shared_ptr<IMeshReader> mesh_sequence)
{
	logOptions(logger, options, ceres_options);
	if (type == RegistrationType::ARAP_AllFrames) {
		//using SequenceARAPFactory = RigidBeforeNonRigidRegistrationFactory<ARAPFactory>;
		using SequenceARAPFactory = RigidBeforeNonRigidRegistrationFactory<RefineDeformationGraphRegistrationFactory<ARAPFactory>>;
		SequenceARAPFactory factory(options, ceres_options, logger);
		return std::make_unique<SequenceRegistration<SequenceARAPFactory>>(mesh_sequence, factory, options, logger);
	}
	else if (type == RegistrationType::ED_AllFrames) {
		using SequenceEDFactory = RigidBeforeNonRigidRegistrationFactory<EmbeddedDeformationFactory>;
		SequenceEDFactory factory(options, ceres_options, logger);
		return std::make_unique<SequenceRegistration<SequenceEDFactory>>(mesh_sequence, factory, options, logger);
	}
	else if (type == RegistrationType::Rigid_AllFrames) {
		Registration::RigidFactory factory(options, ceres_options, logger);
		return std::make_unique<SequenceRegistration<Registration::RigidFactory>>(mesh_sequence, factory, options, logger);
	}
	else {
		throw("Registration type makes no sense in this configuration");
		return nullptr;
	}
}

std::unique_ptr<IRegistration> createRegistration(RegistrationType type,
												  RegistrationOptions & options,
												  ceres::Solver::Options & ceres_options,
												  std::shared_ptr<FileWriter> logger,
												  const SurfaceMesh & source,
												  const SurfaceMesh & target) 
{
	logOptions(logger, options, ceres_options);
	if (type == RegistrationType::ARAP) {
		RigidBeforeNonRigidRegistrationFactory<ARAPFactory> factory(options, ceres_options, logger);
		return factory(source, target);
	}
	else if (type == RegistrationType::ED) {
		RigidBeforeNonRigidRegistrationFactory<EmbeddedDeformationFactory> factory(options, ceres_options, logger);
		return factory(source, target);
	}
	else if (type == RegistrationType::ARAP_Without_RIGID) {
		ARAPFactory factory(options, ceres_options, logger);
		return factory(source, target);
	}
	else if (type == RegistrationType::ED_Without_RIGID) {
		EmbeddedDeformationFactory factory(options, ceres_options, logger);
		return factory(source, target);
	}
	else if (type == RegistrationType::Rigid) {
		RigidFactory factory(options, ceres_options, logger);
		return factory(source, target);
	}
	else {
		throw("Registration type makes no sense in this configuration");
		return nullptr;
	}
}



std::unique_ptr<INonRigidRegistration> createRegistrationNoICP(RegistrationType type,
															   RegistrationOptions & options,
															   ceres::Solver::Options & ceres_options,
															   std::shared_ptr<FileWriter> logger, 
															   const SurfaceMesh & source,
															   const SurfaceMesh & target,
															   std::vector<vertex_descriptor> fixed_positions)
{
	logOptions(logger, options, ceres_options);
	if (fixed_positions.empty())
		std::cout << "fixed position are not set" << std::endl;
	if (type == RegistrationType::ED_WithoutICP) {
		return ED::createEmbeddedDeformation(source, target, fixed_positions, ceres_options, options, logger);
	}
	else if (type == RegistrationType::ARAP_WithoutICP) {
		return Registration::createAsRigidAsPossible(source, target, fixed_positions, ceres_options, options, logger);
	}
	else {
		throw("Registration type is not non rigid without icp");
		return nullptr;
	}
}

}