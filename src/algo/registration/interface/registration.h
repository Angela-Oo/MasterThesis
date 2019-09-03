#pragma once

#include "i_registration.h"
#include "registration_options.h"
#include "i_sequence_registration.h"
#include "registration_type.h"
#include "util/file_writer.h"
#include "input_reader/mesh_reader.h"
#include "util/ceres_include.h"

typedef ml::TriMeshf Mesh;
namespace Registration {


std::unique_ptr<ISequenceRegistration> createSequenceRegistration(RegistrationOptions & options,
																  std::shared_ptr<FileWriter> logger,
																  std::shared_ptr<IMeshReader> mesh_sequence);


std::unique_ptr<IRegistration> createRegistration(RegistrationOptions & options,
												  std::shared_ptr<FileWriter> logger,
												  const SurfaceMesh & source,
												  const SurfaceMesh & target);


std::unique_ptr<INonRigidRegistration> createRegistrationNoICP(RegistrationOptions & options,
															   std::shared_ptr<FileWriter> logger,
															   const SurfaceMesh & source,
															   const SurfaceMesh & target,
															   std::vector<vertex_descriptor> fixed_positions);


}