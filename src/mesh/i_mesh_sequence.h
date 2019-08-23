#pragma once

#include "mesh/mesh_definition.h"
#include <vector>

class IMeshReader
{
public:
	virtual const SurfaceMesh & getMesh(size_t frame) = 0;
	virtual std::vector<vertex_descriptor> getFixedPositions(size_t frame) = 0;
	//virtual unsigned int frame() = 0;
	virtual size_t size() = 0;
};