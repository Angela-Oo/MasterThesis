#pragma once

#include "stdafx.h"
#include "algo/mesh_knn.h"

typedef ml::TriMeshf Mesh;
class TriMeshKNN;

class FindCorrespondecePoint
{
	Mesh _mesh;
	TriMeshKNN _nn_search;
	double _max_allowed_distance;
	double _max_normal_angle_deviation;
public:
	std::pair<bool, ml::vec3f> correspondingPoint(ml::vec3f point, ml::vec3f normal);
	FindCorrespondecePoint(Mesh mesh, double max_allowed_distance = 0.05, double max_normal_angle_deviation = 30.);
};