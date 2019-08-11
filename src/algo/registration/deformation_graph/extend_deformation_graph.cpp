#include "extend_deformation_graph.h"

#include "algo/nearest_neighbor_search/nearest_neighbor_search.h"
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/boost/graph/Euler_operations.h>
#include <vector>

void cutAreaOutOfMesh(SurfaceMesh & mesh, const SurfaceMesh & cut_area, double covered_if_distance_is_smaller_than)
{
	auto knn_search = NearestNeighborSearch(cut_area);

	//find overlapping regions
	auto distance = mesh.add_property_map<vertex_descriptor, double>("v:distance_to_target", 0.).first;
	auto marked_to_remove = mesh.add_property_map<vertex_descriptor, bool>("v:remove", false).first;
	for (auto v : mesh.vertices()) {
		auto neares_point = knn_search.search(mesh.point(v)).begin();
		distance[v] = std::sqrt(neares_point->second);
		if (distance[v] < covered_if_distance_is_smaller_than)
			marked_to_remove[v] = true;
	}

	for (auto f : mesh.faces()) {
		bool remove_face = true;
		auto h = mesh.halfedge(f);
		for (auto v : mesh.vertices_around_face(h))
		{
			if (!marked_to_remove[v])
				remove_face = false;
		}
		CGAL::Euler::make_hole(h, mesh);
	}
}

struct halfedge2edge
{
	halfedge2edge(const SurfaceMesh& m, std::vector<edge_descriptor>& edges)
		: m_mesh(m), m_edges(edges)
	{}
	void operator()(const halfedge_descriptor& h) const
	{
		m_edges.push_back(edge(h, m_mesh));
	}
	const SurfaceMesh& m_mesh;
	std::vector<edge_descriptor>& m_edges;
};

std::vector<halfedge_descriptor> allBorderHalfedgesAroundVertex(vertex_descriptor v, const SurfaceMesh & mesh)
{
	std::vector<halfedge_descriptor> border_halfedges_around_v;
	for (auto he : mesh.halfedges_around_target(mesh.halfedge(v)))
	{
		if (mesh.is_border(he))
			border_halfedges_around_v.push_back(he);
	}
	return border_halfedges_around_v;
}

std::vector<halfedge_descriptor> connectingEdgesOnBorder(vertex_descriptor v0, vertex_descriptor v1, SurfaceMesh & mesh)
{
	std::vector<halfedge_descriptor> connecting_edges;
	std::vector<halfedge_descriptor> border_halfedges_around_v0 = allBorderHalfedgesAroundVertex(v0, mesh);
	for (auto he : border_halfedges_around_v0)
	{
		if (mesh.source(he) == v1)
		{
			connecting_edges.push_back(he);
		}
	}
	return connecting_edges;
}

SurfaceMesh connectTwoMeshesAtBorder(SurfaceMesh & mesh_a, const SurfaceMesh & mesh_b, double merge_if_distance_to_vertex_is_smaller_than)
{
	SurfaceMesh combined_mesh = mesh_a;

	std::vector<edge_descriptor> border_merged_mesh;
	CGAL::Polygon_mesh_processing::border_halfedges(faces(combined_mesh), 
													combined_mesh,
													boost::make_function_output_iterator(halfedge2edge(combined_mesh, border_merged_mesh)));

	std::vector<edge_descriptor> border_mesh_b;
	CGAL::Polygon_mesh_processing::border_halfedges(faces(mesh_b), 
													mesh_b, 
													boost::make_function_output_iterator(halfedge2edge(mesh_b, border_mesh_b)));

	std::vector<vertex_descriptor> border_vertices_mesh_b;
	for (auto & e : border_mesh_b) {
		border_vertices_mesh_b.push_back(combined_mesh.vertex(e, 0));
	}

	auto knn_search = NearestNeighborSearch(mesh_b, border_vertices_mesh_b.begin(), border_vertices_mesh_b.end());
	
	std::map<vertex_descriptor, vertex_descriptor> corresponding_border_vertices;
	for (auto & e : border_merged_mesh) {
		auto vertex = combined_mesh.vertex(e, 0);
		auto p0 = combined_mesh.point(vertex);
		auto corresponding_vertex = knn_search.search(p0, 0).begin()->first;
		corresponding_border_vertices[vertex] = corresponding_vertex;
	}

	// border edge collaps
	for (auto & e : border_merged_mesh) {
		auto v0 = combined_mesh.vertex(e, 0);
		auto cv0 = corresponding_border_vertices[v0];

		auto v1 = combined_mesh.vertex(e, 1);
		auto cv1 = corresponding_border_vertices[v1];


	}

	return mesh_a;
}

SurfaceMesh mergeMeshIntoMesh(const SurfaceMesh & graph, const SurfaceMesh & mesh)
{
	double covered_if_distance_to_vertex_is_smaller_than = 0.05;
	auto knn_search = NearestNeighborSearch(graph);
	auto cliped_mesh = mesh;
	auto merged_mesh = graph;
	//find overlapping regions

	cutAreaOutOfMesh(cliped_mesh, graph, covered_if_distance_to_vertex_is_smaller_than);


	auto connected_mesh = connectTwoMeshesAtBorder(merged_mesh, cliped_mesh, covered_if_distance_to_vertex_is_smaller_than);
	return connected_mesh;
}