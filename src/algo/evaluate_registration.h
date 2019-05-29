#pragma once

#include "../mLibInclude.h"
#include "mLibFLANN.h"
#include "ext-openmesh/triMesh.h"
#include "mesh_knn.h"

typedef ml::TriMeshf Mesh;
std::vector<ml::vec3f> evaluate_error(const Mesh & result, const Mesh & reference_mesh);

std::vector<float> evaluate_distance_error(const Mesh& points_a, const std::vector<ml::vec3f>& points_b);


class ErrorEvaluation
{
	Mesh _reference_mesh;
	ml::OpenMeshTriMesh::Mesh _reference_open_mesh;
	std::unique_ptr<OpenMeshKNN> _knn;
public:
	std::vector<ml::vec3f> evaluate_error(const Mesh & mesh);
public:
	ErrorEvaluation(const Mesh & reference_mesh);
};