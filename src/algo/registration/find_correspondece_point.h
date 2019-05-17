#pragma once

#include "stdafx.h"
#include "algo/mesh_knn.h"
typedef ml::TriMeshf Mesh;

class FindCorrespondecePoint
{
	Mesh _mesh;
	TriMeshKNN _nn_search;
public:
	std::pair<bool, const ml::vec3f &> correspondingPoint(ml::vec3f point, ml::vec3f normal);
	FindCorrespondecePoint(Mesh mesh);
};