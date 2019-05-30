#pragma once

#include "algo/file_writer.h"
#include "algo/surface_mesh/mesh_definition.h"
#include "algo/mesh_simplification/mesh_simplification.h"
#include "algo/registration/deformation_graph/deformation_graph.h"
#include "algo/registration/i_registration.h"
#include "algo/registration/registration.h"
#include <ceres/ceres.h>

class SequenceRegistration
{
private:
	RegistrationType _registration_type;
	std::vector<SurfaceMesh> _meshes;
	std::vector<SurfaceMesh> _deformed_meshes;
	std::vector<DG::DeformationGraph> _deformation_graphs;
	size_t _current;
	unsigned int _number_of_deformation_nodes;
	std::unique_ptr<IRegistration> _registration;
	std::shared_ptr<FileWriter> _logger;
public:
	bool solve();
	bool finished();
	size_t getCurrent();
	SurfaceMesh getMesh(int frame);
	SurfaceMesh getDeformedMesh(int frame);
	SurfaceMesh getDeformationGraphMesh(int frame);
public:
	SequenceRegistration();
	SequenceRegistration(const std::vector<SurfaceMesh> & meshes,
						 RegistrationType registration_type, 
						 std::shared_ptr<FileWriter> logger, 
						 unsigned int number_of_deformation_nodes = 1000);
};

