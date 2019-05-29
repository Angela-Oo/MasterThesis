#pragma once

#include "algo/file_writer.h"
#include "algo/mesh_simplification/mesh_simplification.h"
#include <ceres/ceres.h>
#include "input_reader/mesh_reader.h"

#include "i_registration.h"

typedef ml::TriMeshf Mesh;


enum class RegistrationType
{
	ASAP,
	ASAP_WithoutICP,
	ED,
	ED_WithoutICP,
	Rigid,
	AllFrames
};

//class Registration
//{
//	std::unique_ptr<IRegistration> _registration;
//	const int _number_of_deformation_graph_nodes = 2000;
//	std::shared_ptr<FileWriter> _logger;
//public:
//	const Mesh & getSource();
//	const Mesh & getTarget();
//	Mesh getDeformedPoints();
//	Mesh getInverseDeformedPoints();
//	Registration(RegistrationType type,
//				 std::shared_ptr<IMeshReader> mesh_reader, 
//				 int source_frame, int target_frame,
//				 std::shared_ptr<FileWriter> logger = nullptr);
//	bool solve();
//};

