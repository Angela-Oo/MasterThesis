#include "ed_fit_cost.h"
#include "mesh/mesh_definition.h"
#include "algo/registration/util/ceres_residual_evaluation.h"
#include "ed_point_to_plane_point_to_point_cost.h"

namespace Registration {


void EmbeddedDeformationFitCost::evaluateResiduals(ceres::Problem &problem, SurfaceMesh & mesh, CeresIterationLoggerGuard & logger)
{
	if (!mesh.property_map<vertex_descriptor, double>("v:fit_cost").second) {
		mesh.add_property_map<vertex_descriptor, double>("v:fit_cost", 0.);
	}
	auto fit_cost = mesh.property_map<vertex_descriptor, double>("v:fit_cost");
	if (fit_cost.second) {
		// TODO
		//auto max_and_mean_cost = ::evaluateResiduals(mesh, problem, _fit_residual_ids, fit_cost.first, _options.fit);
		
		auto residuals = ::evaluateResiduals(problem, _fit_residual_ids);
		assert(residuals.size() == residual_block_ids.size());
		double mean_cost = 0.;
		double max_cost = 0.0;
		for (auto & r : residuals) {
			auto cost = r * _options.fit;
			if (cost > max_cost)
				max_cost = cost;
			mean_cost += cost;
		}
		mean_cost /= residuals.size();
		
		logger.write(" max_fit_cost: " + std::to_string(max_cost) + " mean_fit_cost: " + std::to_string(mean_cost), false);
	}
}

bool EmbeddedDeformationFitCost::useVertex(vertex_descriptor & v, DeformedMesh<DeformationGraph<EDDeformation>> & deformed_mesh)
{
	bool use_vertex = true;
	if (_options.ignore_border_vertices)
		use_vertex = !deformed_mesh.getSourceMesh().is_border(v, true);

	return use_vertex;
}

bool EmbeddedDeformationFitCost::addFitCostVertex(ceres::Problem & problem,
												  vertex_descriptor & v,
												  DeformationGraph<EDDeformation> & deformation_graph,
												  DeformedMesh<DeformationGraph<EDDeformation>> & deformed_mesh)
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

		auto residual_id_point_to_point = pointToPointCost(problem, point_to_point_weighting,
														   deformation_graph, deformed_mesh, v,
														   target_point);
		_fit_residual_ids[v].push_back(residual_id_point_to_point);

		assert(target_normal.squared_length() > 0.);
		auto residual_id_point_to_plane = pointToPlaneCost(problem, point_to_plane_weighting,
														   deformation_graph, deformed_mesh, v,
														   target_point, target_normal);
		_fit_residual_ids[v].push_back(residual_id_point_to_plane);

		return true;
	}
	return false;
}

EmbeddedDeformationFitCost::VertexResidualIds
EmbeddedDeformationFitCost::addFitCost(ceres::Problem &problem,
									 DeformationGraph<EDDeformation> & deformation_graph,
									 DeformedMesh<DeformationGraph<EDDeformation>> & deformed_mesh,
									 std::unique_ptr<CeresIterationLoggerGuard>& logger)
{
	_fit_residual_ids.clear();		
	int i = 0;
	 // use fixed subset of vertices
	for (auto & v : _subset_of_vertices_to_fit)
	{
		if (useVertex(v, deformed_mesh)) {
			if (addFitCostVertex(problem, v, deformation_graph, deformed_mesh)) {
				++i;
			}
		}
	}	

	logger->write("used " + std::to_string(i) + " / " + std::to_string(deformed_mesh.number_of_vertices()) + " vertices ");
	//std::cout << " allowed distance " <<  _find_correspondence_point->median() << " ";
	return _fit_residual_ids;
}

EmbeddedDeformationFitCost::EmbeddedDeformationFitCost(SurfaceMesh & target,
												   std::vector<vertex_descriptor> & subset_of_vertices_to_fit,
												   RegistrationOptions options)
	: _subset_of_vertices_to_fit(subset_of_vertices_to_fit)
	, _options(options)
{
	_find_correspondence_point = std::make_unique<FindCorrespondingPoints>(target,
																		   _options.icp.correspondence_max_distance,
																		   _options.icp.correspondence_max_angle_deviation,
																		   10.);
}





}
