#include "stdafx.h"

#include "mesh_convertion.h"
#include <CGAL/Polygon_mesh_processing/compute_normal.h>

SurfaceMesh convertToCGALMesh(const Mesh& triMesh, bool calculate_normals) {
	SurfaceMesh mesh;

	std::vector<SurfaceMesh::Vertex_index> vertex_handles;
	vertex_handles.reserve(triMesh.getVertices().size());
	auto normals = mesh.add_property_map<vertex_descriptor, Vector>("v:normal", Vector(0., 0., 0.)).first;
	auto colors = mesh.add_property_map<vertex_descriptor, ml::vec4f>("v:color", ml::vec4f(1., 1., 1., 1.)).first;
	auto edge_colors = mesh.add_property_map<edge_descriptor, ml::vec4f>("e:color", ml::vec4f(1., 1., 1., 1.)).first;

	for (const auto& v : triMesh.getVertices()) {
		auto vertex_handle = mesh.add_vertex(SurfaceMesh::Point(v.position.x, v.position.y, v.position.z));
		normals[vertex_handle] = Vector(v.normal.x, v.normal.y, v.normal.z);
		if (v.normal.length() <= 0. && calculate_normals == false)
		{
			std::cout << "no vertex normal given" << std::endl;
			calculate_normals = true;
		}
		//colors[vertex_handle] = Color(v.normal.x, v.normal.y, v.normal.z);
		vertex_handles.push_back(vertex_handle);
	}

	for (const auto& f : triMesh.getIndices()) {
		mesh.add_face(vertex_handles[f.x], vertex_handles[f.y], vertex_handles[f.z]);
	}

	if (calculate_normals) {
		CGAL::Polygon_mesh_processing::compute_vertex_normals(mesh, normals);
	}
	return mesh;
}

Mesh convertToTriMesh(const SurfaceMesh& mesh) {

	ml::MeshDataf mData;
	mData.m_Vertices.resize(mesh.number_of_vertices());
	
	std::map<vertex_descriptor, int> cgal_vertex_handle_to_mesh_vertex_handle;

	auto mesh_color_property_map = mesh.property_map<vertex_descriptor, ml::vec4f>("v:color");
	if(mesh_color_property_map.second)
		mData.m_Colors.resize(mesh.number_of_vertices());
	int i = 0;
	for (auto vertex_it = mesh.vertices_begin(); vertex_it != mesh.vertices_end(); ++vertex_it, ++i) {
		const auto & point = mesh.point(*vertex_it);
		mData.m_Vertices[i] = ml::vec3f(point.x(), point.y(), point.z());
		if (mesh_color_property_map.second) {
			mData.m_Colors[i] = mesh_color_property_map.first[*vertex_it];
		}
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

