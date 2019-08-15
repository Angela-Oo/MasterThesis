#pragma once

#include "arap.h"
#include "util/file_writer.h"
#include "mesh/mesh_definition.h"

#include <ceres/ceres.h>

namespace Registration {

class ARAPFactory
{
public:
	using Registration = AsRigidAsPossible;
private:
	ceres::Solver::Options _ceres_options;
	RegistrationOptions _options;
	std::shared_ptr<FileWriter> _logger;
	std::vector<vertex_descriptor> _fixed_positions;
public:
	std::unique_ptr<AsRigidAsPossible> operator()(const SurfaceMesh & source,
												  const SurfaceMesh & target);
	std::unique_ptr<AsRigidAsPossible> operator()(const SurfaceMesh & source,
												  const SurfaceMesh & target,
												  const AsRigidAsPossible::Deformation & deformation_graph);
	std::unique_ptr<AsRigidAsPossible> operator()(const SurfaceMesh & source,
												  const SurfaceMesh & target,
												  const SurfaceMesh & previous_mesh, // used for non rigid registration
												  const AsRigidAsPossible::Deformation & deformation_graph);
	void setFixedPositions(std::vector<vertex_descriptor> fixed_positions);
	std::string registrationType();
public:
	ARAPFactory(const RegistrationOptions & options,
				const ceres::Solver::Options & ceres_options,
				std::shared_ptr<FileWriter> logger);
};

}