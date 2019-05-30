#include "stdafx.h"

#include "mesh_convertion.h"

SurfaceMesh convertToCGALMesh(const Mesh& triMesh) {
	SurfaceMesh mesh;

	std::vector<SurfaceMesh::Vertex_index> vertex_handles;
	vertex_handles.reserve(triMesh.getVertices().size());
	auto normals = mesh.add_property_map<vertex_descriptor, Direction>("v:normal", Direction(0., 0., 1.)).first;
	auto colors = mesh.add_property_map<vertex_descriptor, ml::vec4f>("v:color", ml::vec4f(1., 1., 1., 1.)).first;
	auto edge_colors = mesh.add_property_map<edge_descriptor, ml::vec4f>("e:color", ml::vec4f(1., 1., 1., 1.)).first;
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
