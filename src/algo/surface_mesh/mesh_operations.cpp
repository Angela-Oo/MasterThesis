#pragma once

#include "mesh_operations.h"
#include <algorithm>
#include <CGAL/boost/graph/Euler_operations.h>


namespace Registration
{

halfedge_descriptor splitEdgeAtCenter(edge_descriptor e, SurfaceMesh & mesh)
{
	auto he = mesh.halfedge(e);
	Point source = mesh.point(mesh.source(he));
	Point target = mesh.point(mesh.target(he));
	Vector dir = target - source;
	Point center_point = source + (dir * 0.5);

	auto new_he = CGAL::Euler::split_edge(he, mesh);
	mesh.point(mesh.target(new_he)) = center_point;
	return new_he;
}

void splitEdge(edge_descriptor e, SurfaceMesh & mesh)
{
	auto new_he_f0 = splitEdgeAtCenter(e, mesh);

	auto split_f0_he0 = new_he_f0;
	auto split_f0_he1 = mesh.prev(mesh.prev(new_he_f0));

	auto new_he_f1 = mesh.opposite(new_he_f0);
	auto split_f1_he0 = mesh.prev(new_he_f1);
	auto split_f1_he1 = mesh.next(new_he_f1);

	CGAL::Euler::split_face(split_f0_he0, split_f0_he1, mesh);
	CGAL::Euler::split_face(split_f1_he0, split_f1_he1, mesh);
}

boost::optional<halfedge_descriptor> halfedgeCorrespondingToFace(face_descriptor f, vertex_descriptor v, const SurfaceMesh & mesh)
{
	for (auto he_around_v : CGAL::halfedges_around_target(v, mesh)) {
		auto face = mesh.face(he_around_v);
		if (face == f) {
			return he_around_v;
		}
	}
	return boost::optional<halfedge_descriptor>();
}

bool splitFaceAtEdge(face_descriptor f, vertex_descriptor v0, SurfaceMesh & mesh)
{
	auto he0 = halfedgeCorrespondingToFace(f, v0, mesh);

	if (he0) {
		auto he1 = mesh.prev(mesh.prev(*he0));
		CGAL::Euler::split_face(*he0, he1, mesh);
		return true;
	}
	return false;
}

bool splitFaceAtEdge(face_descriptor f, vertex_descriptor v0, vertex_descriptor v1, SurfaceMesh & mesh)
{
	auto he0 = halfedgeCorrespondingToFace(f, v0, mesh);
	auto he1 = halfedgeCorrespondingToFace(f, v1, mesh);

	if (he0 && he1) {
		CGAL::Euler::split_face(*he0, *he1, mesh);
		return true;
	}
	return false;
}


bool splitFaceAtEdge(face_descriptor f, vertex_descriptor v0, vertex_descriptor v1, vertex_descriptor v2, SurfaceMesh & mesh)
{
	auto he0 = halfedgeCorrespondingToFace(f, v0, mesh);
	auto he1 = halfedgeCorrespondingToFace(f, v1, mesh);
	auto he2 = halfedgeCorrespondingToFace(f, v2, mesh);

	if (he0 && he1 && he2) {
		CGAL::Euler::split_face(*he0, *he1, mesh);
		CGAL::Euler::split_face(*he1, *he2, mesh);
		CGAL::Euler::split_face(*he2, *he0, mesh);
		return true;
	}
	return false;
}



void splitFaces(const std::map<face_descriptor, std::vector<vertex_descriptor>> & face_vertices_to_split_map, SurfaceMesh & mesh)
{
	for (auto f : face_vertices_to_split_map) {
		if (f.second.size() == 2) {
			splitFaceAtEdge(f.first, f.second[0], f.second[1], mesh);
		}
		else if (f.second.size() == 3) {
			splitFaceAtEdge(f.first, f.second[0], f.second[1], f.second[2], mesh);
		}
		else if (f.second.size() == 1) {
			splitFaceAtEdge(f.first, f.second[0], mesh);
		}
	}
}


void splitFaceAtEdge(face_descriptor f, SurfaceMesh & mesh)
{
	auto he_0 = mesh.halfedge(f);
	auto he_1 = mesh.next(he_0);
	auto he_2 = mesh.next(he_1);

	auto new_he_0 = splitEdgeAtCenter(mesh.edge(he_0), mesh);
	auto new_he_1 = splitEdgeAtCenter(mesh.edge(he_1), mesh);
	auto new_he_2 = splitEdgeAtCenter(mesh.edge(he_2), mesh);

	CGAL::Euler::split_face(new_he_0, new_he_1, mesh);
	CGAL::Euler::split_face(new_he_1, new_he_2, mesh);
	CGAL::Euler::split_face(new_he_2, new_he_0, mesh);
}


vertex_descriptor splitFace(face_descriptor f, SurfaceMesh & mesh)
{
	auto he = CGAL::Euler::add_center_vertex(mesh.halfedge(f), mesh);
	return mesh.target(he);
}



void flipEdges(std::vector<edge_descriptor> & edges, SurfaceMesh & mesh)
{
	for (auto e : edges) {
		CGAL::Euler::flip_edge(mesh.halfedge(e), mesh);
	}
}




std::set<vertex_descriptor> oneRingNeighborhood(const SurfaceMesh & mesh, vertex_descriptor v)
{
	std::set<vertex_descriptor> neighbors;
	for (auto & n : mesh.vertices_around_target(mesh.halfedge(v))) {
		neighbors.insert(n);
	}
	return neighbors;
}

std::set<vertex_descriptor> twoRingNeighborhood(const SurfaceMesh & mesh, vertex_descriptor v)
{
	std::set<vertex_descriptor> neighbors = oneRingNeighborhood(mesh, v);

	for (auto n : neighbors) {
		std::set<vertex_descriptor> nn = oneRingNeighborhood(mesh, n);
		neighbors.insert(nn.begin(), nn.end());
	}
	neighbors.erase(v);
	return neighbors;
}

}