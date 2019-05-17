#include "stdafx.h"

#include "mesh_simplification.h"
#include "mLibCGAL.h"

#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
// Stop-condition policy
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>

typedef CGAL::Simple_cartesian<double>                                   Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> SurfaceMesh;
typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor    vertex_descriptor;
//typedef boost::graph_traits<SurfaceMesh>::vertex_iterator        vertex_iterator;
typedef SurfaceMesh::Face_index        face_descriptor;

SurfaceMesh convertToCGALMesh(const Mesh& triMesh) {
	SurfaceMesh mesh;

	std::vector<SurfaceMesh::Vertex_index> vertex_handles;
	vertex_handles.reserve(triMesh.getVertices().size());

	for (const auto& v : triMesh.getVertices()) {
		auto vertex_handle = mesh.add_vertex(SurfaceMesh::Point(v.position.x, v.position.y, v.position.z));
		vertex_handles.push_back(vertex_handle);
	}

	for (const auto& f : triMesh.getIndices()) {
		mesh.add_face(vertex_handles[f.x], vertex_handles[f.y], vertex_handles[f.z]);
	}
	return mesh;
}

Mesh convertToTriMesh(SurfaceMesh& mesh) {

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
	return Mesh(mData);
}

Mesh createReducedMesh(const Mesh & mesh, int number_of_vertices)
{
	auto surface_mesh = convertToCGALMesh(mesh);

	CGAL::Surface_mesh_simplification::Count_stop_predicate_vertices<SurfaceMesh> stop(number_of_vertices);


	// This is a stop predicate (defines when the algorithm terminates).
	// In this example, the simplification stops when the number of undirected edges
	// left in the surface mesh drops below the specified number (1000)
	//CGAL::Surface_mesh_simplification::Count_stop_predicate<Mesh> stop(1000);

	//CGAL::Surface_mesh_simplification::
	int r = CGAL::Surface_mesh_simplification::edge_collapse(surface_mesh, stop);
	return convertToTriMesh(surface_mesh);
}
