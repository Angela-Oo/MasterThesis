#include "hierarchical_deformation_graph_reader.h"
#include "algo/triangulation/hierarchical_mesh.h"
#include "algo/triangulation/generate_hierarchical_mesh.h"


const SurfaceMesh & HierarchicalDeformationGraphReader::getMesh(size_t frame)
{
	assert(frame < _hierarchical_deformation_graph.size());
	return _meshes[frame];
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
	return _meshes.size();
}

HierarchicalDeformationGraphReader::HierarchicalDeformationGraphReader(std::shared_ptr<MeshReader> reader)
	: _reader(reader)
	, _radius(0.05)
{
	_reader->processFrame();
	_reader->processFrame();
	_mesh = _reader->getMesh(0);

	_hierarchical_mesh = generateHierarchicalMesh(_mesh, _radius, 4);

	_meshes.insert(_meshes.end(), _hierarchical_mesh._meshes.begin(), _hierarchical_mesh._meshes.end());

	auto marked_edge_mesh = _meshes.back();
	auto color = marked_edge_mesh.property_map<edge_descriptor, ml::vec4f>("e:color").first;

	auto edge = *marked_edge_mesh.edges_begin();
	color[edge] = ml::vec4f(1., 0., 0., 1.);

	_meshes.push_back(marked_edge_mesh);

	_hierarchical_mesh.refineEdge(edge);
}




