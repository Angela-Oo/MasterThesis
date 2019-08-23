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

HierarchicalDeformationGraphReader::HierarchicalDeformationGraphReader(std::shared_ptr<MeshReader> reader,
																	   unsigned int number_of_interpolation_neighbors)
	: _reader(reader)
	, _radius(0.05)
{
	_reader->processFrame();
	_mesh = _reader->getMesh(0);

	_hierarchical_mesh = generateHierarchicalMesh(_mesh, _radius, number_of_interpolation_neighbors);
	for (unsigned int i = 0; i < _hierarchical_mesh.size(); ++i)
		_meshes.emplace_back(_hierarchical_mesh.getMesh(i));
	auto mesh_to_refine = _hierarchical_mesh.getInitMesh();

	auto color = mesh_to_refine.property_map<edge_descriptor, ml::vec4f>("e:color").first;
	
	auto edge = *mesh_to_refine.edges().begin();
	color[edge] = ml::vec4f(1., 0., 0., 1.);
	_meshes.push_back(mesh_to_refine);

	HierarchicalMeshRefinement mesh_refinement(_hierarchical_mesh);
	auto vertices = mesh_refinement.refine({ edge }, mesh_to_refine);

	_meshes.push_back(mesh_to_refine);

	std::vector<edge_descriptor> edges;
	for (auto v : vertices) {
		auto edge = mesh_to_refine.edge(mesh_to_refine.halfedge(v));
		color[edge] = ml::vec4f(0., 1., 0., 1.);
		edges.push_back(edge);
	}	
	_meshes.push_back(mesh_to_refine);

	mesh_refinement.refine(edges, mesh_to_refine);
	_meshes.push_back(mesh_to_refine);
}




