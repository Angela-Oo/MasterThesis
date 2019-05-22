#include "stdafx.h"
#include "find_correspondece_point.h"


std::pair<bool, ml::vec3f> FindCorrespondecePoint::correspondingPoint(ml::vec3f point, ml::vec3f normal)
{
	unsigned int i = _nn_search.nearest_index(point);
	auto vertex = _mesh.getVertices()[i];
	bool valid = true;
	auto dot_product = ml::vec3f::dot(normal.getNormalized(), vertex.normal.getNormalized());
	auto angle = acos(dot_product);
	//if (dot_product < 0.5)
	if(abs(angle) > ml::math::degreesToRadians(30.))
		valid = false;
	if (dist(point, vertex.position) > 0.01)
		valid = false;
	return std::make_pair(valid, vertex.position);
}

FindCorrespondecePoint::FindCorrespondecePoint(Mesh mesh)
	: _mesh(mesh)
	, _nn_search(mesh)
{
}