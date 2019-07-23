#include "deformation_graph.h"


namespace Registration {

PositionAndDeformation createGlobalDeformation(const SurfaceMesh & mesh, std::function<std::shared_ptr<IPositionDeformation>()> create_node)
{
	Vector global_position(0., 0., 0.);
	for (auto & v : mesh.vertices()) {
		global_position += mesh.point(v) - CGAL::ORIGIN;
	}
	global_position /= mesh.number_of_vertices();

	PositionAndDeformation global;
	global._point = CGAL::ORIGIN + global_position;
	global._deformation = create_node();
	return global;
}

}