#include "arap_fit_cost.h"
#include "arap_cost.h"
#include "mesh/mesh_definition.h"

namespace Registration {


void AsRigidAsPossibleFitCost::evaluateResiduals(ceres::Problem &problem, SurfaceMesh & mesh, CeresIterationLoggerGuard & logger)
{
	//auto fit_cost = mesh.property_map<edge_descriptor, double>("e:smooth_cost");
	//if (fit_cost.second) {
	//	auto max_and_mean_cost = ::evaluateResiduals(mesh, problem, _fit_residual_ids, fit_cost.first, _options.fit);
	//	logger.write(" max fit cost: " + std::to_string(max_and_mean_cost.first), false);
	//}
}

bool AsRigidAsPossibleFitCost::useVertex(vertex_descriptor & v, DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh)
{
	bool use_vertex = true;
	if (_options.ignore_deformation_graph_border_vertices)
		use_vertex = !deformed_mesh.getSourceMesh().is_border(v, true);

	if (deformed_mesh.nearestNodes(v).node_weight_vector.size() < _options.dg_options.number_of_interpolation_neighbors)
		use_vertex = false;
	return use_vertex;
}

bool AsRigidAsPossibleFitCost::addFitCostVertex(ceres::Problem & problem,
												vertex_descriptor & v,
												DeformationGraph<ARAPDeformation> & deformation_graph,
												DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
												VertexResidualIds &residual_ids)
{
	auto deformed_point = deformed_mesh.deformed_point(v);
	auto deformed_normal = deformed_mesh.deformed_normal(v);
	auto correspondent_point = _find_correspondence_point->correspondingPoint(deformed_point, deformed_normal);

	if (correspondent_point.first)
	{
		vertex_descriptor target_vertex = correspondent_point.second;
		auto target_point = _find_correspondence_point->getPoint(target_vertex);
		auto target_normal = _find_correspondence_point->getNormal(target_vertex);

		double point_to_point_weighting = _options.fit * 0.1;
		double point_to_plane_weighting = _options.fit * 0.9;

		auto residual_id_point_to_point = pointToPointCost(problem, point_to_point_weighting, _options.dg_options.number_of_interpolation_neighbors,
														   deformation_graph, deformed_mesh, v,
														   target_point);
		residual_ids[v].push_back(residual_id_point_to_point);

		assert(target_normal.squared_length() > 0.);
		auto residual_id_point_to_plane = pointToPlaneCost(problem, point_to_plane_weighting, _options.dg_options.number_of_interpolation_neighbors,
														   deformation_graph, deformed_mesh, v,
														   target_point, target_normal);
		residual_ids[v].push_back(residual_id_point_to_plane);

		return true;
	}
	return false;
}

AsRigidAsPossibleFitCost::VertexResidualIds
AsRigidAsPossibleFitCost::addFitCost(ceres::Problem &problem,
									 DeformationGraph<ARAPDeformation> & deformation_graph,
									 DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
									 std::unique_ptr<CeresIterationLoggerGuard>& logger)
{
	VertexResidualIds residual_ids;

	int i = 0;
	// use all vertices
	if (_subset_of_vertices_to_fit.empty()) {
		for (auto & v : deformed_mesh.vertices())
		{
			if (useVertex(v, deformed_mesh)) {
				if (addFitCostVertex(problem, v, deformation_graph, deformed_mesh, residual_ids)) {
					++i;
				}
			}
		}
	} // use fixed subset of vertices
	else {
		for (auto & v : _subset_of_vertices_to_fit)
		{
			if (useVertex(v, deformed_mesh)) {
				if (addFitCostVertex(problem, v, deformation_graph, deformed_mesh, residual_ids)) {
					++i;
				}
			}
		}
	}

	logger->write("used " + std::to_string(i) + " / " + std::to_string(deformed_mesh.number_of_vertices()) + " vertices ");
	//std::cout << " allowed distance " <<  _find_correspondence_point->median() << " ";
	return residual_ids;
}

AsRigidAsPossibleFitCost::AsRigidAsPossibleFitCost(SurfaceMesh & target,
												   std::vector<vertex_descriptor> & subset_of_vertices_to_fit,
												   RegistrationOptions options)
	: _subset_of_vertices_to_fit(subset_of_vertices_to_fit)
	, _options(options)
{
	_find_correspondence_point = std::make_unique<FindCorrespondingPoints>(target,
																		   _options.correspondence_max_distance,
																		   _options.correspondence_max_angle_deviation,
																		   10.);
}





}