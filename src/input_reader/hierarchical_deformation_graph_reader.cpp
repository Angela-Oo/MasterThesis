#include "hierarchical_deformation_graph_reader.h"
#include "algo/triangulation/hierarchical_mesh.h"
#include "algo/triangulation/generate_hierarchical_mesh.h"


const SurfaceMesh & HierarchicalDeformationGraphReader::getMesh(size_t frame)
{
	assert(frame < _hierarchical_deformation_graph.size());
	return _hierarchical_mesh._meshes[frame];
}

bool HierarchicalDeformationGraphReader::processFrame()
{
	return false;
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
	return _hierarchical_mesh._meshes.size();
}

HierarchicalDeformationGraphReader::HierarchicalDeformationGraphReader(std::shared_ptr<MeshReader> reader)
	: _reader(reader)
	, _radius(0.05)
{
	_reader->processFrame();
	_reader->processFrame();
	_mesh = _reader->getMesh(0);

	_hierarchical_mesh = generateHierarchicalMesh(_mesh, _radius, 4);
}




