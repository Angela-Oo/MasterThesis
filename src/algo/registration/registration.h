#pragma once

#include "algo/file_writer.h"
#include "algo/mesh_simplification/mesh_simplification.h"
#include <ceres/ceres.h>
#include "input_reader/mesh_reader.h"

#include "i_registration.h"

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


class RegistrationFactory
{
	RegistrationType _registration_type;
	ceres::Solver::Options _ceres_options;
	RegistrationOptions _options;
	std::shared_ptr<FileWriter> _logger;
	std::vector<vertex_descriptor> _fixed_positions;
private:
	std::unique_ptr<INonRigidRegistration> buildWithoutICP(const SurfaceMesh & source,
														   const SurfaceMesh & target);
	std::unique_ptr<INonRigidRegistration> buildWithoutRigid(const SurfaceMesh & source,
															 const SurfaceMesh & target);
	std::unique_ptr<INonRigidRegistration> buildWithoutRigid(const SurfaceMesh & source,
															 const SurfaceMesh & target,
															 const DeformationGraph & deformation_graph);
	std::unique_ptr<INonRigidRegistration> buildWithRigid(const SurfaceMesh & source,
														  const SurfaceMesh & target);
	std::unique_ptr<INonRigidRegistration> buildWithRigid(const SurfaceMesh & source,
														  const SurfaceMesh & target,
														  const DeformationGraph & deformation_graph);
public:
	void setRegistrationType(RegistrationType type);
	void setCeresOption(const ceres::Solver::Options options);
	void setRegistrationOption(const RegistrationOptions & options);
	void setLogger(std::shared_ptr<FileWriter> logger);
	void setFixedPositions(std::vector<vertex_descriptor> fixed_positions);
public:
	std::unique_ptr<INonRigidRegistration> buildNonRigidRegistration(const SurfaceMesh & source,
																	 const SurfaceMesh & target);
	std::unique_ptr<INonRigidRegistration> buildNonRigidRegistration(const SurfaceMesh & source,
																	 const SurfaceMesh & target,
																	 const DeformationGraph & deformation_graph);
	std::unique_ptr<IRigidRegistration> buildRigidRegistration(const SurfaceMesh & source,
															   const SurfaceMesh & target);
	std::unique_ptr<IRegistration> buildRegistration(const SurfaceMesh & source,
													 const SurfaceMesh & target);
	void logConfiguration();
public:
	RegistrationFactory();
};


}