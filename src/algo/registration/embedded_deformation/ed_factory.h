#pragma once


#include "algo/file_writer.h"
#include <ceres/ceres.h>
#include "algo/surface_mesh/mesh_definition.h"
#include "ed.h"

namespace Registration {
using namespace ED;

class EmbeddedDeformationFactory
{
	ceres::Solver::Options _ceres_options;
	RegistrationOptions _options;
	std::shared_ptr<FileWriter> _logger;
	std::vector<vertex_descriptor> _fixed_positions;
public:
	std::unique_ptr<EmbeddedDeformation> operator()(const SurfaceMesh & source,
													const SurfaceMesh & target);
	std::unique_ptr<EmbeddedDeformation> operator()(const SurfaceMesh & source,
													const SurfaceMesh & target,
													const DeformationGraph & deformation_graph);
	SurfaceMesh deformationGraphMesh(const DeformationGraph & deformation_graph);
	SurfaceMesh deformedMesh(const SurfaceMesh & mesh, const DeformationGraph & deformation_graph);
	SurfaceMesh inverseDeformedMesh(const SurfaceMesh & mesh, const DeformationGraph & deformation_graph);
	void setFixedPositions(std::vector<vertex_descriptor> fixed_positions);
	std::string registrationType();
	void logConfiguration();
public:
	EmbeddedDeformationFactory(const RegistrationOptions & options,
							   const ceres::Solver::Options & ceres_options,
							   std::shared_ptr<FileWriter> logger);
};

}