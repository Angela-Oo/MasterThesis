#pragma once

#include "algo/surface_mesh/mesh_definition.h" // todo move to mesh folder
#include <vector>

class IMeshReader
{
public:
	virtual SurfaceMesh& getMesh(unsigned int frame) = 0;
	virtual std::vector<vertex_descriptor> getFixedPositions(unsigned int frame) = 0;
	virtual unsigned int frame() = 0;
	virtual size_t size() = 0;
};