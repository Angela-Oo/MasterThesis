#include "hierarchical_deformation_graph_reader.h"
#include "algo/triangulation/hierarchical_mesh.h"
#include "algo/triangulation/generate_hierarchical_mesh.h"


const SurfaceMesh & HierarchicalDeformationGraphReader::getMesh(size_t frame)
{
	assert(frame < _meshes.size());
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

	_meshes.insert(_meshes.end(), _hierarchical_mesh._meshes.rbegin(), _hierarchical_mesh._meshes.rend());

	int i = 0;
	for (auto edge : _hierarchical_mesh._mesh.edges()) {
		auto marked_edge_mesh = _hierarchical_mesh._mesh;
		auto color = marked_edge_mesh.property_map<edge_descriptor, ml::vec4f>("e:color").first;
		color[edge] = ml::vec4f(1., 0., 0., 1.);

		
		_hierarchical_mesh.refineEdge(edge);
		_meshes.push_back(marked_edge_mesh);
		i++;
		if (i > 5)
			break;
	}
		
	_hierarchical_mesh.triangulate();
	_meshes.push_back(_hierarchical_mesh._mesh);
}




