#pragma once

#include "ed.h"
#include "util/file_writer.h"
#include "mesh/mesh_definition.h"
#include "algo/registration/deformation_graph/deformation_graph_deform_mesh.h"
#include <ceres/ceres.h>

namespace Registration {
using namespace ED;

class EmbeddedDeformationFactory
{
public:
	using Registration = EmbeddedDeformation;
	using PositionDeformation = EDDeformation;
	using DeformMesh = DeformationGraphDeformMesh<typename EmbeddedDeformation::Deformation>;
private:
	ceres::Solver::Options _ceres_options;
	RegistrationOptions _options;
	std::shared_ptr<FileWriter> _logger;
	std::vector<vertex_descriptor> _fixed_positions;
public:
	std::unique_ptr<EmbeddedDeformation> operator()(const SurfaceMesh & source,
													const SurfaceMesh & target);
	std::unique_ptr<EmbeddedDeformation> operator()(const SurfaceMesh & source,
													const SurfaceMesh & target,
													const EmbeddedDeformation::Deformation & deformation_graph);
	std::unique_ptr<EmbeddedDeformation> operator()(const SurfaceMesh & source,
													const SurfaceMesh & target,
													const SurfaceMesh & previous_mesh, // used for non rigid registration
													const EmbeddedDeformation::Deformation & deformation_graph);
	void setFixedPositions(std::vector<vertex_descriptor> fixed_positions);
	std::string registrationType();
public:
	EmbeddedDeformationFactory(const RegistrationOptions & options,
							   const ceres::Solver::Options & ceres_options,
							   std::shared_ptr<FileWriter> logger);
};

}