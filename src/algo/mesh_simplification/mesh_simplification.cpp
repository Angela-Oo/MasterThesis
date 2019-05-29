#include "stdafx.h"

#include "mesh_simplification.h"
#include "mLibCGAL.h"
//
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
//// Stop-condition policy
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
//
//typedef CGAL::Simple_cartesian<double>                                   Kernel;
//typedef CGAL::Surface_mesh<Kernel::Point_3> SurfaceMesh;
//typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor    vertex_descriptor;
////typedef boost::graph_traits<SurfaceMesh>::vertex_iterator        vertex_iterator;
//typedef SurfaceMesh::Face_index        face_descriptor;

//
//class INode
//{
//public:
//	virtual Kernel::Point_3 deformed() = 0;
//};
//
//class NodeTest : public INode
//{
//public:
//	Kernel::Point_3 x;
//	Kernel::Point_3 deformed() {
//		return x;
//	}
//	NodeTest(double f)
//		: x(f, 0., 0.) 
//	{}
//};
//
//SurfaceMesh convertToDeformationGraphMesh(const Mesh& triMesh) {
//	SurfaceMesh mesh;
//
//	auto property_map = mesh.add_property_map<vertex_descriptor, std::shared_ptr<INode>>("v:node", nullptr);
//	
//	std::vector<SurfaceMesh::Vertex_index> vertex_handles;
//	vertex_handles.reserve(triMesh.getVertices().size());
//
//	for (const auto& v : triMesh.getVertices()) {
//		auto vertex_handle = mesh.add_vertex(SurfaceMesh::Point(v.position.x, v.position.y, v.position.z));
//		vertex_handles.push_back(vertex_handle);
//		property_map.first[vertex_handle] = std::make_shared<NodeTest>(2.);
//	}
//
//	for (const auto& f : triMesh.getIndices()) {
//		mesh.add_face(vertex_handles[f.x], vertex_handles[f.y], vertex_handles[f.z]);
//	}
//	auto get = mesh.property_map<vertex_descriptor, std::shared_ptr<INode>>("v:node");
//	for (auto x : get.first)
//	{
//		std::cout << " " << x->deformed()[0] << std::endl;
//	}
//	return mesh;
//}
//
SurfaceMesh convertToCGALMesh(const Mesh& triMesh) {
	SurfaceMesh mesh;

	std::vector<SurfaceMesh::Vertex_index> vertex_handles;
	vertex_handles.reserve(triMesh.getVertices().size());
	auto normals = mesh.add_property_map<vertex_descriptor, Direction>("v:normal", Direction(0., 0., 1.)).first;
	auto colors = mesh.add_property_map<vertex_descriptor, CGAL::Color>("v:color", CGAL::Color(1., 1., 1.)).first;
	for (const auto& v : triMesh.getVertices()) {
		auto vertex_handle = mesh.add_vertex(SurfaceMesh::Point(v.position.x, v.position.y, v.position.z));
		normals[vertex_handle] = Direction(v.normal.x, v.normal.y, v.normal.z);
		//colors[vertex_handle] = Color(v.normal.x, v.normal.y, v.normal.z);
		vertex_handles.push_back(vertex_handle);
	}	

	for (const auto& f : triMesh.getIndices()) {
		mesh.add_face(vertex_handles[f.x], vertex_handles[f.y], vertex_handles[f.z]);
	}
	return mesh;
}

Mesh convertToTriMesh(const SurfaceMesh& mesh) {

	ml::MeshDataf mData;
	mData.m_Vertices.resize(mesh.number_of_vertices());
	std::map<vertex_descriptor, int> cgal_vertex_handle_to_mesh_vertex_handle;

	int i = 0;
	for (auto vertex_it = mesh.vertices_begin(); vertex_it != mesh.vertices_end(); ++vertex_it, ++i) {
		const auto & point = mesh.point(*vertex_it);
		mData.m_Vertices[i] = ml::vec3f(point.x(), point.y(), point.z());
		cgal_vertex_handle_to_mesh_vertex_handle[*vertex_it] = i;
	}

	mData.m_FaceIndicesVertices.resize(mesh.number_of_faces());

	i = 0;
	for (auto face_it = mesh.faces_begin(); face_it != mesh.faces_end(); ++face_it, ++i) {		
		auto vertices_around_face = mesh.vertices_around_face(mesh.halfedge(*face_it));

		auto vertex_it = vertices_around_face.begin();
		mData.m_FaceIndicesVertices[i][0] = cgal_vertex_handle_to_mesh_vertex_handle[*vertex_it];
		vertex_it++;
		mData.m_FaceIndicesVertices[i][1] = cgal_vertex_handle_to_mesh_vertex_handle[*vertex_it];
		vertex_it++;
		mData.m_FaceIndicesVertices[i][2] = cgal_vertex_handle_to_mesh_vertex_handle[*vertex_it];
	}
	auto trimesh = Mesh(mData);
	trimesh.computeNormals();
	return trimesh;
}

Mesh createReducedMesh(const Mesh & mesh, int number_of_vertices)
{
	auto surface_mesh = convertToCGALMesh(mesh);

	//CGAL::Surface_mesh_simplification::Count_stop_predicate_vertices<SurfaceMesh> stop(number_of_vertices);

	// This is a stop predicate (defines when the algorithm terminates).
	// In this example, the simplification stops when the number of undirected edges
	// left in the surface mesh drops below the specified number (1000)
	CGAL::Surface_mesh_simplification::Count_stop_predicate<SurfaceMesh> stop(number_of_vertices);

	//CGAL::Surface_mesh_simplification::
	int r = CGAL::Surface_mesh_simplification::edge_collapse(surface_mesh, stop);
	return convertToTriMesh(surface_mesh);
}



SurfaceMesh createReducedMesh(const SurfaceMesh & mesh, int number_of_vertices)
{
	SurfaceMesh surface_mesh = mesh;
	CGAL::Surface_mesh_simplification::Count_stop_predicate<SurfaceMesh> stop(number_of_vertices);
	int r = CGAL::Surface_mesh_simplification::edge_collapse(surface_mesh, stop);
	return surface_mesh;
}
