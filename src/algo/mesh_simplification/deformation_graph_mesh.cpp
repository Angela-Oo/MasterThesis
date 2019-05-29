#include "stdafx.h"

#include "deformation_graph_mesh.h"
#include "algo/deformation_graph/i_node.h"

#include "algo/deformation_graph/nearest_neighbor_search.h"


DeformationGraphMesh convertToDeformationGraphMesh(const Mesh& triMesh) {
	DeformationGraphMesh mesh;

	auto property_map = mesh.add_property_map<vertex_descriptor, std::shared_ptr<INode>>("v:node", nullptr);
	//auto property_map_edge = mesh.add_property_map<edge_descriptor, std::shared_ptr<IEdge>>("e:edge", nullptr);

	std::vector<vertex_descriptor> vertex_handles;
	vertex_handles.reserve(triMesh.getVertices().size());

	for (const auto& v : triMesh.getVertices()) {
		auto vertex_handle = mesh.add_vertex(Point(v.position.x, v.position.y, v.position.z));
		vertex_handles.push_back(vertex_handle);
		//property_map.first[vertex_handle] = std::make_shared<NodeTest>(2.);
	}

	for (const auto& f : triMesh.getIndices()) {
		mesh.add_face(vertex_handles[f.x], vertex_handles[f.y], vertex_handles[f.z]);
	}

	NearestNeighborSearch search(mesh);
	Point query(0.05, 0., 0.);
	Neighbor_search s = search.search(query, 5);

	for (Neighbor_search::iterator it = s.begin(); it != s.end(); ++it) {
		auto distance = std::sqrt(it->second);
		auto vertex_handle = it->first;
		auto point = mesh.point(vertex_handle);
		std::cout << vertex_handle << " p: " << point << " dist " << distance << std::endl;
	}
	return mesh;
}

//
//Mesh convertToTriMesh(DeformationGraphMesh& mesh) {
//
//	ml::MeshDataf mData;
//	mData.m_Vertices.resize(mesh.number_of_vertices());
//	std::map<vertex_descriptor, int> cgal_vertex_handle_to_mesh_vertex_handle;
//
//	int i = 0;
//	for (auto vertex_it = mesh.vertices_begin(); vertex_it != mesh.vertices_end(); ++vertex_it, ++i) {
//		const auto & point = mesh.point(*vertex_it);
//		mData.m_Vertices[i] = ml::vec3f(point.x(), point.y(), point.z());
//		cgal_vertex_handle_to_mesh_vertex_handle[*vertex_it] = i;
//	}
//
//	mData.m_FaceIndicesVertices.resize(mesh.number_of_faces());
//
//	i = 0;
//	for (auto face_it = mesh.faces_begin(); face_it != mesh.faces_end(); ++face_it, ++i) {
//		auto vertices_around_face = mesh.vertices_around_face(mesh.halfedge(*face_it));
//
//		auto vertex_it = vertices_around_face.begin();
//		mData.m_FaceIndicesVertices[i][0] = cgal_vertex_handle_to_mesh_vertex_handle[*vertex_it];
//		vertex_it++;
//		mData.m_FaceIndicesVertices[i][1] = cgal_vertex_handle_to_mesh_vertex_handle[*vertex_it];
//		vertex_it++;
//		mData.m_FaceIndicesVertices[i][2] = cgal_vertex_handle_to_mesh_vertex_handle[*vertex_it];
//	}
//	auto trimesh = Mesh(mData);
//	trimesh.computeNormals();
//	return trimesh;
//}

//Mesh createReducedMesh(const Mesh & mesh, int number_of_vertices)
//{
//	auto surface_mesh = convertToCGALMesh(mesh);
//
//	//CGAL::Surface_mesh_simplification::Count_stop_predicate_vertices<SurfaceMesh> stop(number_of_vertices);
//
//	// This is a stop predicate (defines when the algorithm terminates).
//	// In this example, the simplification stops when the number of undirected edges
//	// left in the surface mesh drops below the specified number (1000)
//	CGAL::Surface_mesh_simplification::Count_stop_predicate<SurfaceMesh> stop(number_of_vertices);
//
//	//CGAL::Surface_mesh_simplification::
//	int r = CGAL::Surface_mesh_simplification::edge_collapse(surface_mesh, stop);
//	return convertToTriMesh(surface_mesh);
//}
