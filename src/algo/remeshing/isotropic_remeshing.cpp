#include "isotropic_remeshing.h"
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/border.h>

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



SurfaceMesh isotropicRemeshing(const SurfaceMesh & mesh, double target_edge_length)
{
	SurfaceMesh surface_mesh = mesh;

	std::vector<edge_descriptor> border;
	CGAL::Polygon_mesh_processing::border_halfedges(faces(surface_mesh), surface_mesh, boost::make_function_output_iterator(halfedge2edge(surface_mesh, border)));
	CGAL::Polygon_mesh_processing::split_long_edges(border, target_edge_length, surface_mesh);

	CGAL::Polygon_mesh_processing::isotropic_remeshing(faces(surface_mesh),
													   target_edge_length,
													   surface_mesh,
													   CGAL::Polygon_mesh_processing::parameters::number_of_iterations(10));// .relax_constraints(true));// .protect_constraints(true));

	surface_mesh.collect_garbage();
	auto normals = surface_mesh.property_map<vertex_descriptor, Vector>("v:normal").first;
	CGAL::Polygon_mesh_processing::compute_vertex_normals(surface_mesh, normals);
	
	std::cout << " count of surface mesh vertices " << surface_mesh.number_of_vertices() << std::endl;
	return surface_mesh;
}

