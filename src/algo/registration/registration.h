#pragma once

#include "algo/file_writer.h"
#include "algo/mesh_simplification/mesh_simplification.h"
#include <ceres/ceres.h>
#include "input_reader/mesh_reader.h"

#include "i_registration.h"
#include "algo/registration/sequence_registration/i_sequence_registration.h"

typedef ml::TriMeshf Mesh;
namespace Registration {

enum class RegistrationType
{
	ARAP,
	ARAP_Without_RIGID,
	ARAP_WithoutICP,
	ED,
	ED_Without_RIGID,
	ED_WithoutICP,
	Rigid,
	ARAP_AllFrames,
	ED_AllFrames,
	Rigid_AllFrames
};


std::unique_ptr<ISequenceRegistration> createSequenceRegistration(RegistrationType type,
																  RegistrationOptions & options,
																  ceres::Solver::Options & ceres_options,
																  std::shared_ptr<FileWriter> logger,
																  std::shared_ptr<IMeshReader> mesh_sequence);

std::unique_ptr<IRegistration> createRegistration(RegistrationType type,
												  RegistrationOptions & options,
												  ceres::Solver::Options & ceres_options,
												  std::shared_ptr<FileWriter> logger,
												  const SurfaceMesh & source,
												  const SurfaceMesh & target);


std::unique_ptr<INonRigidRegistration> createRegistrationNoICP(RegistrationType type,
															   RegistrationOptions & options,
															   ceres::Solver::Options & ceres_options,
															   std::shared_ptr<FileWriter> logger,
															   const SurfaceMesh & source,
															   const SurfaceMesh & target,
															   std::vector<vertex_descriptor> fixed_positions);


}