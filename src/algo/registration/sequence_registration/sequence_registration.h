#pragma once

#include "algo/file_writer.h"
#include "algo/surface_mesh/mesh_definition.h"
#include "algo/mesh_simplification/mesh_simplification.h"
#include "algo/registration/deformation_graph/deformation_graph.h"
#include "algo/registration/i_registration.h"
#include "algo/registration/registration.h"
#include "algo/ceres_iteration_logger.h"
#include <ceres/ceres.h>
#include <memory>

class SequenceRegistration
{
private:
	RegistrationType _registration_type;
	std::vector<SurfaceMesh> _meshes;
	std::vector<SurfaceMesh> _deformed_meshes;
	std::vector<DG::DeformationGraph> _deformation_graphs;
	size_t _current;
	std::unique_ptr<IRegistration> _registration;
	bool _evaluate_residuals;
	std::shared_ptr<FileWriter> _logger;
	std::unique_ptr<CeresLogger> _ceres_logger;
public:
	bool solve();
	bool finished();
	size_t getCurrent();
	SurfaceMesh getMesh(size_t frame);
	SurfaceMesh getDeformedMesh(size_t frame);
	SurfaceMesh getDeformationGraphMesh(size_t frame);
public:
	SequenceRegistration();
	SequenceRegistration(const std::vector<SurfaceMesh> & meshes,
						 RegistrationType registration_type, 
						 std::shared_ptr<FileWriter> logger, 
						 double deformation_graph_edge_length = 0.05);
};

