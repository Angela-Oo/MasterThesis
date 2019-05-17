#include "stdafx.h"
#include "find_correspondece_point.h"



std::pair<bool, const ml::vec3f &> FindCorrespondecePoint::correspondingPoint(ml::vec3f point, ml::vec3f normal)
{
	unsigned int i = _nn_search.nearest_index(point);
	auto & vertex = _mesh.getVertices()[i];
	bool valid = true;
	//if (abs(ml::vec3f::dot(normal, vertex.normal)) > 0.1)
	//	valid = false;
	//if (dist(point, vertex.position) > 0.1)
	//	valid = false;
	return std::make_pair(valid, vertex.position);
}

FindCorrespondecePoint::FindCorrespondecePoint(Mesh mesh)
	: _mesh(mesh)
	, _nn_search(mesh)
{
}