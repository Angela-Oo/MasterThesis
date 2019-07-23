#pragma once

#include "algo/surface_mesh/mesh_definition.h"

class ISequenceRegistration
{
public:
	virtual std::pair<bool, std::string> saveCurrentFrameAsImage() = 0;
	virtual SurfaceMesh getMesh(size_t frame) = 0;
	virtual SurfaceMesh getDeformedMesh(size_t frame) = 0;
	virtual SurfaceMesh getInverseDeformedMesh(size_t frame) = 0;
	virtual SurfaceMesh getDeformationGraphMesh(size_t frame) = 0;
	virtual size_t getCurrent() = 0;
	virtual void nextFrame() = 0;
	virtual bool finished() = 0;
	virtual bool solve() = 0;
	virtual ~ISequenceRegistration() {}
};
