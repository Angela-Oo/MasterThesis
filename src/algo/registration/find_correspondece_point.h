#pragma once

#include "stdafx.h"
#include "algo/mesh_knn.h"
#include <deque>

typedef ml::TriMeshf Mesh;
class TriMeshKNN;

double angle_between_to_vectors_in_rad(ml::vec3f vector_a, ml::vec3f vector_b);

class FindCorrespondecePoint
{
	Mesh _mesh;
	TriMeshKNN _nn_search;
	double _max_normal_angle_deviation;
	double _median_distance;
	double _k;	
public:
	double median();
	std::pair<bool, ml::vec3f> correspondingPointAngle(ml::vec3f point, ml::vec3f normal);
	std::pair<bool, ml::vec3f> correspondingPointDistance(ml::vec3f point);
	std::pair<bool, ml::vec3f> correspondingPoint(ml::vec3f point, ml::vec3f normal);
	FindCorrespondecePoint(Mesh mesh, double max_allowed_distance = 0.05, double max_normal_angle_deviation = 30.);
};