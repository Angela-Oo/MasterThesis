#pragma once

#include "i_registration.h"
#include "i_sequence_registration.h"
#include "registration_type.h"
#include "algo/file_writer.h"
#include "input_reader/mesh_reader.h"
#include <ceres/ceres.h>

typedef ml::TriMeshf Mesh;
namespace Registration {


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