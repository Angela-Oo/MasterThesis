#pragma once

#include "i_reader.h"
#include "mesh_reader.h"

class HierarchicalDeformationGraphReader : public IMeshReader
{
private:
	std::shared_ptr<MeshReader> _reader;
	SurfaceMesh _mesh;
	std::vector<SurfaceMesh> _hierarchical_deformation_graph;
	double _radius;
public:
	const SurfaceMesh & getMesh(size_t frame) override;
	std::vector<vertex_descriptor> getFixedPositions(size_t frame) override {
		return std::vector<vertex_descriptor>();
	};
	bool processFrame();
	size_t size() override;
	bool processAllFrames();
public:
	HierarchicalDeformationGraphReader(std::shared_ptr<MeshReader> reader);
};
