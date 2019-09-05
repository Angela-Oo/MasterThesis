#include "registration.h"

#include "algo/registration/arap/arap.h"
#include "algo/registration/embedded_deformation/ed.h"
#include "algo/registration/rigid_registration/rigid_registration.h"
#include "algo/registration/rigid_before_non_rigid_registration/rigid_before_non_rigid_registration.h"
#include "algo/registration/sequence_registration/sequence_registration.h"
#include "algo/registration/deformation_graph_refinement/refinement_registration.h"
#include "algo/registration/deformation_graph_adaptive_rigidity/adaptive_rigidity_registration.h"
#include "algo/registration/evaluation/evaluate_registration.h"

namespace Registration {

using RefineARAPRegistration = EvaluateRegistration<RigidBeforeNonRigidRegistration<RefineDeformationGraphRegistration<AsRigidAsPossible>>>;
using AdaptiveARAPRegistration = EvaluateRegistration<RigidBeforeNonRigidRegistration<AdaptiveRigidityRegistration<AsRigidAsPossible>>>;
using ARAPRegistration = EvaluateRegistration<RigidBeforeNonRigidRegistration<AsRigidAsPossible>>;

using RefineARAPNonRigidRegistration = EvaluateRegistration<RefineDeformationGraphRegistration<AsRigidAsPossible>>;
using AdaptiveARAPNonRigidRegistration = EvaluateRegistration<AdaptiveRigidityRegistration<AsRigidAsPossible>>;
using ARAPNonRigidRegistration = EvaluateRegistration<AsRigidAsPossible>;

using RefineEDRegistration = EvaluateRegistration<RigidBeforeNonRigidRegistration<RefineDeformationGraphRegistration<EmbeddedDeformation>>>;
using AdaptiveEDRegistration = EvaluateRegistration<RigidBeforeNonRigidRegistration<AdaptiveRigidityRegistration<EmbeddedDeformation>>>;
using EDRegistration = EvaluateRegistration<RigidBeforeNonRigidRegistration<EmbeddedDeformation>>;

using RefineEDNonRigidRegistration = EvaluateRegistration<RefineDeformationGraphRegistration<EmbeddedDeformation>>;
using AdaptiveEDNonRigidRegistration = EvaluateRegistration<AdaptiveRigidityRegistration<EmbeddedDeformation>>;
using EDNonRigidRegistration = EvaluateRegistration<EmbeddedDeformation>;

std::unique_ptr<ISequenceRegistration> createSequenceRegistration(RegistrationOptions & options,
																  std::shared_ptr<FileWriter> logger,
																  std::shared_ptr<IMeshReader> mesh_sequence)
{
	if (!options.sequence_options.enable) {
		std::cout << "sequence should be enabled" << std::endl;
		throw("Registration type makes no sense in this configuration");
	}
	if (options.type == RegistrationType::ARAP) {
		if (options.rigid_and_non_rigid_registration) {
			if (options.refinement.enable) {
				return std::make_unique<SequenceRegistration<RefineARAPRegistration>>(mesh_sequence, options, logger);
			}
			else if (options.adaptive_rigidity.enable && options.adaptive_rigidity.adaptive_rigidity == AdaptiveRigidity::REDUCE_RIGIDITY) {
				return std::make_unique<SequenceRegistration<AdaptiveARAPRegistration>>(mesh_sequence, options, logger);
			}
			else {
				return std::make_unique<SequenceRegistration<ARAPRegistration>>(mesh_sequence, options, logger);
			}
		}
		else {
			if (options.refinement.enable) {
				return std::make_unique<SequenceRegistration<RefineARAPNonRigidRegistration>>(mesh_sequence, options, logger);
			}
			else if (options.adaptive_rigidity.enable && options.adaptive_rigidity.adaptive_rigidity == AdaptiveRigidity::REDUCE_RIGIDITY) {
				return std::make_unique<SequenceRegistration<AdaptiveARAPNonRigidRegistration>>(mesh_sequence, options, logger);
			}
			else {
				return std::make_unique<SequenceRegistration<ARAPNonRigidRegistration>>(mesh_sequence, options, logger);
			}
		}
	}
	else if (options.type == RegistrationType::ED) {
		if (options.rigid_and_non_rigid_registration) {
			if (options.refinement.enable) {
				return std::make_unique<SequenceRegistration<RefineEDRegistration>>(mesh_sequence, options, logger);
			}
			else if (options.adaptive_rigidity.enable && options.adaptive_rigidity.adaptive_rigidity == AdaptiveRigidity::REDUCE_RIGIDITY) {
				return std::make_unique<SequenceRegistration<AdaptiveEDRegistration>>(mesh_sequence, options, logger);
			}
			else {
				return std::make_unique<SequenceRegistration<EDRegistration>>(mesh_sequence, options, logger);
			}
		}
		else {
			if (options.refinement.enable) {
				return std::make_unique<SequenceRegistration<RefineEDNonRigidRegistration>>(mesh_sequence, options, logger);
			}
			else if (options.adaptive_rigidity.enable && options.adaptive_rigidity.adaptive_rigidity == AdaptiveRigidity::REDUCE_RIGIDITY) {
				return std::make_unique<SequenceRegistration<AdaptiveEDNonRigidRegistration>>(mesh_sequence, options, logger);
			}
			else {
				return std::make_unique<SequenceRegistration<EDNonRigidRegistration>>(mesh_sequence, options, logger);
			}
		}
	}
	else if (options.type == RegistrationType::Rigid) {
		return std::make_unique<SequenceRegistration<RigidRegistration>>(mesh_sequence, options, logger);
	}
	else {
		throw("Registration type makes no sense in this configuration");
	}
}





std::unique_ptr<IRegistration> createRegistration(RegistrationOptions & options,
												  std::shared_ptr<FileWriter> logger,
												  const SurfaceMesh & source,
												  const SurfaceMesh & target)
{
	if (options.type == RegistrationType::ARAP) {
		if (options.rigid_and_non_rigid_registration) {
			if (options.refinement.enable) {
				return std::make_unique<RefineARAPRegistration>(source, target, options, logger);
			}
			else if (options.adaptive_rigidity.enable && options.adaptive_rigidity.adaptive_rigidity == AdaptiveRigidity::REDUCE_RIGIDITY) {
				return std::make_unique<AdaptiveARAPRegistration>(source, target, options, logger);
			}
			else {
				return std::make_unique<ARAPRegistration>(source, target, options, logger);
			}
		}
		else {
			if (options.refinement.enable) {
				return std::make_unique<RefineARAPNonRigidRegistration>(source, target, options, logger);
			}
			else if (options.adaptive_rigidity.enable && options.adaptive_rigidity.adaptive_rigidity == AdaptiveRigidity::REDUCE_RIGIDITY) {
				return std::make_unique<AdaptiveARAPNonRigidRegistration>(source, target, options, logger);
			}
			else {
				return std::make_unique<ARAPNonRigidRegistration>(source, target, options, logger);
			}
		}
	}
	else if (options.type == RegistrationType::ED) {
		if (options.rigid_and_non_rigid_registration) {
			if (options.refinement.enable) {
				return std::make_unique<RefineEDRegistration>(source, target, options, logger);
			}
			else if (options.adaptive_rigidity.enable && options.adaptive_rigidity.adaptive_rigidity == AdaptiveRigidity::REDUCE_RIGIDITY) {
				return std::make_unique<AdaptiveEDRegistration>(source, target, options, logger);
			}
			else {
				return std::make_unique<EDRegistration>(source, target, options, logger);
			}
		}
		else {
			if (options.refinement.enable) {
				return std::make_unique<RefineEDNonRigidRegistration>(source, target, options, logger);
			}
			else if (options.adaptive_rigidity.enable && options.adaptive_rigidity.adaptive_rigidity == AdaptiveRigidity::REDUCE_RIGIDITY) {
				return std::make_unique<AdaptiveEDNonRigidRegistration>(source, target, options, logger);
			}
			else {
				return std::make_unique<EDNonRigidRegistration>(source, target, options, logger);
			}
		}
	}
	else if (options.type == RegistrationType::Rigid) {
		return std::make_unique<RigidRegistration>(source, target, options, logger);
	}
	std::cout << " registration typ not known" << std::endl;
	throw("Registration type makes no sense in this configuration");
	return nullptr;
}



std::unique_ptr<INonRigidRegistration> createRegistrationNoICP(RegistrationOptions & options,
															   std::shared_ptr<FileWriter> logger,
															   const SurfaceMesh & source,
															   const SurfaceMesh & target,
															   std::vector<vertex_descriptor> fixed_positions)
{
	if (fixed_positions.empty())
		std::cout << "fixed position are not set" << std::endl;
	if (options.type == RegistrationType::ED_WithoutICP) {
		return createEmbeddedDeformation(source, target, fixed_positions, options, logger);
	}
	else if (options.type == RegistrationType::ARAP_WithoutICP) {
		return createAsRigidAsPossible(source, target, fixed_positions, options, logger);
	}
	else {
		throw("Registration type is not non rigid without icp");
		return nullptr;
	}
}

}