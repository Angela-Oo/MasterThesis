#pragma once

#include "mesh/i_mesh_sequence.h"
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
	std::shared_ptr<IMeshReader> _mesh_sequence;
	std::vector<SurfaceMesh> _deformed_meshes;
	std::vector<DG::DeformationGraph> _deformation_graphs;
	size_t _current;
	std::unique_ptr<INonRigidRegistration> _registration;
	RegistrationFactory _registration_factory;
	RegistrationOptions _registration_options;
	std::unique_ptr<CeresLogger> _ceres_logger;
	bool _finished;
public:
	bool solve();
	void nextFrame();
	bool finished();
	size_t getCurrent();
	std::pair<bool, std::string> saveCurrentFrameAsImage();
	SurfaceMesh getMesh(size_t frame);
	SurfaceMesh getDeformedMesh(size_t frame);
	SurfaceMesh getInverseDeformedMesh(size_t frame);
	SurfaceMesh getDeformationGraphMesh(size_t frame);
public:
	SequenceRegistration();
	SequenceRegistration(std::shared_ptr<IMeshReader> mesh_sequence,
						 RegistrationType registration_type, 
						 std::shared_ptr<FileWriter> logger, 
						 RegistrationOptions registration_options);
};

