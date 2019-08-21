#include "registration.h"
#include "algo/registration/util/log_option.h"
#include "algo/registration/options/ceres_option.h"
#include "algo/registration/arap/arap.h"
#include "algo/registration/embedded_deformation/ed.h"
#include "algo/registration/rigid_registration/rigid_registration.h"
#include "algo/registration/rigid_before_non_rigid_registration/rigid_before_non_rigid_registration.h"
#include "algo/registration/sequence_registration/sequence_registration.h"
#include "algo/registration/deformation_graph_refinement/refinement_registration.h"
#include "algo/registration/deformation_graph_adaptive_rigidity/adaptive_rigidity_registration.h"

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
		return std::make_unique<SequenceRegistration<RigidBeforeNonRigidRegistration<RefineDeformationGraphRegistration<AsRigidAsPossible>>>>(mesh_sequence, options, logger);
	}
	else if (type == RegistrationType::ED_AllFrames) {
		return std::make_unique<SequenceRegistration<RigidBeforeNonRigidRegistration<EmbeddedDeformation>>>(mesh_sequence, options, logger);
	}
	else if (type == RegistrationType::Rigid_AllFrames) {
		return std::make_unique<SequenceRegistration<RigidRegistration>>(mesh_sequence, options, logger);
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
		//return std::make_unique<RigidBeforeNonRigidRegistration<RefineDeformationGraphRegistration<AsRigidAsPossible>>>(source, target, ceres_options, options, logger);
		//return std::make_unique<RefineDeformationGraphRegistration<AsRigidAsPossible>>(source, target, ceres_options, options, logger);
		return std::make_unique<AdaptiveRigidityRegistration<AsRigidAsPossible>>(source, target, ceres_options, options, logger);
	}
	else if (type == RegistrationType::ED) {
		return std::make_unique<RigidBeforeNonRigidRegistration<RefineDeformationGraphRegistration<EmbeddedDeformation>>>(source, target, ceres_options, options, logger);
		//return std::make_unique<RigidBeforeNonRigidRegistration<EmbeddedDeformation>>(source, target, ceres_options, options, logger);
	}
	else if (type == RegistrationType::ARAP_Without_RIGID) {
		return std::make_unique<AsRigidAsPossible>(source, target, ceres_options, options, logger);
	}
	else if (type == RegistrationType::ED_Without_RIGID) {
		return std::make_unique<EmbeddedDeformation>(source, target, ceres_options, options, logger);
	}
	else if (type == RegistrationType::Rigid) {
		return std::make_unique<RigidRegistration>(source, target, ceres_options, options, logger);
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
		return createEmbeddedDeformation(source, target, fixed_positions, ceres_options, options, logger);
	}
	else if (type == RegistrationType::ARAP_WithoutICP) {
		return createAsRigidAsPossible(source, target, fixed_positions, ceres_options, options, logger);
	}
	else {
		throw("Registration type is not non rigid without icp");
		return nullptr;
	}
}

}