#include "hierarchical_deformation_graph_reader.h"
#include "algo/mesh_simplification/hierarchical_mesh.h"


const SurfaceMesh & HierarchicalDeformationGraphReader::getMesh(size_t frame)
{
	assert(frame < _hierarchical_deformation_graph.size());
	return _hierarchical_deformation_graph[frame];
}

bool HierarchicalDeformationGraphReader::processFrame()
{
	if (_hierarchical_deformation_graph.empty()) {
		_hierarchical_deformation_graph.push_back(createReducedMesh(_mesh, _radius));
		return true;
	}
	else if (_hierarchical_deformation_graph.size() < 2) {
		double radius = _radius * pow(4, _hierarchical_deformation_graph.size());
		_hierarchical_deformation_graph.push_back(createHierarchicalMesh(_hierarchical_deformation_graph.back(), radius));
		return true;
	}
	else {
		return false;
	}
}

bool HierarchicalDeformationGraphReader::processAllFrames()
{
	bool successfull = true;
	while (successfull) {
		successfull = processFrame();
	}
	return true;
}

size_t HierarchicalDeformationGraphReader::size()
{
	return _hierarchical_deformation_graph.size();
}


HierarchicalDeformationGraphReader::HierarchicalDeformationGraphReader(std::shared_ptr<MeshReader> reader)
	: _reader(reader)
	, _radius(0.05)
{
	_reader->processFrame();
	_mesh = _reader->getMesh(0);
	processAllFrames();
}




