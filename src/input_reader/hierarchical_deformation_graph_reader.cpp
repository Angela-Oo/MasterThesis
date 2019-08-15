#include "hierarchical_deformation_graph_reader.h"
#include "algo/hierarchical_mesh/hierarchical_mesh.h"
#include "algo/hierarchical_mesh/generate_hierarchical_mesh.h"


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
	_mesh = _reader->getMesh(0);

	_hierarchical_mesh = generateHierarchicalMesh(_mesh, _radius, 4);
	_meshes.insert(_meshes.end(), _hierarchical_mesh._meshes.rbegin(), _hierarchical_mesh._meshes.rend());
	auto mesh_to_refine = _hierarchical_mesh.getInitMesh();

	std::vector<edge_descriptor> edges;
	int i = 0;
	for (auto edge : mesh_to_refine.edges()) {
		auto color = mesh_to_refine.property_map<edge_descriptor, ml::vec4f>("e:color").first;
		color[edge] = ml::vec4f(1., 0., 0., 1.);
		
		edges.push_back(edge);
		i++;
		if (i > 5)
			break;
	}
	HierarchicalMeshRefinement mesh_refinement(_hierarchical_mesh);
	mesh_refinement.refine(edges, mesh_to_refine);

	_meshes.push_back(mesh_to_refine);
}




