#pragma once

#include "i_reader.h"
#include "mesh_reader.h"
#include "algo/hierarchical_mesh/hierarchical_mesh.h"

class HierarchicalDeformationGraphReader : public IMeshReader
{
private:
	std::shared_ptr<MeshReader> _reader;
	SurfaceMesh _mesh;
	HierarchicalMesh _hierarchical_mesh;
	std::vector<SurfaceMesh> _meshes;
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
