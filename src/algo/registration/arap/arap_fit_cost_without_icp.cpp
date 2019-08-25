#include "arap_fit_cost_without_icp.h"
#include "arap_cost.h"
#include "mesh/mesh_definition.h"

namespace Registration {



std::vector<Point> AsRigidAsPossibleFitCostWithoutICP::getFixedPostions()
{
	std::vector<Point> positions;
	for (auto & v : _fixed_positions) {
		positions.push_back(_target.point(v));
	}
	return positions;
}

AsRigidAsPossibleFitCostWithoutICP::ResidualIds
AsRigidAsPossibleFitCostWithoutICP::addFitCostVertex(ceres::Problem & problem,
													 DeformationGraph<ARAPDeformation> & deformation_graph,
													 DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
													 SurfaceMesh::Vertex_index & v)
{
	auto target_normals = _target.property_map<vertex_descriptor, Vector>("v:normal").first;

	ResidualIds residual_ids;
	double point_to_point_weighting = _options.fit * 0.1;
	double point_to_plane_weighting = _options.fit * 0.9;

	auto residual_id_point_to_point = pointToPointCost(problem, point_to_point_weighting,
													   deformation_graph, deformed_mesh, v,
													   _target.point(v));
	residual_ids.push_back(residual_id_point_to_point);

	assert(target_normal.squared_length() > 0.);
	auto residual_id_point_to_plane = pointToPlaneCost(problem, point_to_plane_weighting,
													   deformation_graph, deformed_mesh, v,
													   _target.point(v), target_normals[v]);
	residual_ids.push_back(residual_id_point_to_plane);
	return residual_ids;
}


AsRigidAsPossibleFitCostWithoutICP::VertexResidualIds
AsRigidAsPossibleFitCostWithoutICP::addFitCost(ceres::Problem &problem,
											   DeformationGraph<ARAPDeformation> & deformation_graph,
											   DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
											   std::unique_ptr<CeresIterationLoggerGuard>& logger)
{
	std::map<vertex_descriptor, ResidualIds> residual_ids;
	for (auto & v : deformation_graph._mesh.vertices())
	{
		if (_fixed_positions.empty() || (std::find(_fixed_positions.begin(), _fixed_positions.end(), v) != _fixed_positions.end()))
		{
			auto residuals = addFitCostVertex(problem, deformation_graph, deformed_mesh, v);
			residual_ids[v].insert(residual_ids[v].end(), residuals.begin(), residuals.end());
		}
	}
	//	std::cout << "used nodes " << i << " / " << mesh.number_of_vertices();
	return residual_ids;
}


AsRigidAsPossibleFitCostWithoutICP::AsRigidAsPossibleFitCostWithoutICP(SurfaceMesh & target,
																	   std::vector<vertex_descriptor> fixed_positions,
																	   std::vector<vertex_descriptor> & subset_of_vertices_to_fit,
																	   RegistrationOptions options)
	: _target(target)
	, _fixed_positions(fixed_positions)
	, _subset_of_vertices_to_fit(subset_of_vertices_to_fit)
	, _options(options)
{}


}