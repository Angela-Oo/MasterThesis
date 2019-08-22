#include "arap_cost.h"
#include "arap_cost_functions.h"
#include "mesh/mesh_definition.h"

namespace Registration {

const ceres::ResidualBlockId & pointToPointCost3NN(ceres::Problem & problem,
												   double loss_weighting,
												   DeformationGraph<ARAPDeformation> & deformation_graph,
												   DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
												   const vertex_descriptor &v,
												   const Point & target_point)
{
	auto n_w_vector = deformed_mesh.nearestNodes(v).node_weight_vector;
	auto & global = deformation_graph._global;	

	assert(n_w_vector.size() == 3);
	if (n_w_vector.size() < 3)
		std::cout << "help nearest node is smaller than expected" << std::endl;

	auto cost_function = FitStarPointToPointAngleAxisCostFunction::Create(target_point, deformed_mesh.point(v), global.position(),
																		  deformation_graph.getDeformation(n_w_vector[0].first).position(),
																		  deformation_graph.getDeformation(n_w_vector[1].first).position(),
																		  deformation_graph.getDeformation(n_w_vector[2].first).position(),
																		  n_w_vector[0].second, n_w_vector[1].second, n_w_vector[2].second);

	auto loss_function = new ceres::ScaledLoss(NULL, loss_weighting, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function,
									global.d(),
									deformation_graph.getDeformation(n_w_vector[0].first).d(),
									deformation_graph.getDeformation(n_w_vector[1].first).d(),
									deformation_graph.getDeformation(n_w_vector[2].first).d());
}



const ceres::ResidualBlockId & pointToPointCost4NN(ceres::Problem & problem,
												   double loss_weighting,
												   DeformationGraph<ARAPDeformation> & deformation_graph,
												   DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
												   const vertex_descriptor &v,
												   const Point & target_point)
{	
	auto n_w_vector = deformed_mesh.nearestNodes(v).node_weight_vector;
	auto & global = deformation_graph._global;

	assert(n_w_vector.size() == 4);
	if (n_w_vector.size() < 4)
		std::cout << "help nearest node is smaller than expected" << std::endl;

	auto cost_function = FitStarPointToPointAngleAxisCostFunction::Create(target_point, deformed_mesh.point(v), global.position(),
																		  deformation_graph.getDeformation(n_w_vector[0].first).position(),
																		  deformation_graph.getDeformation(n_w_vector[1].first).position(),
																		  deformation_graph.getDeformation(n_w_vector[2].first).position(),
																		  deformation_graph.getDeformation(n_w_vector[3].first).position(),
																		  n_w_vector[0].second, n_w_vector[1].second, n_w_vector[2].second, n_w_vector[3].second);

	auto loss_function = new ceres::ScaledLoss(NULL, loss_weighting, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function,
									global.d(),
									deformation_graph.getDeformation(n_w_vector[0].first).d(),
									deformation_graph.getDeformation(n_w_vector[1].first).d(),
									deformation_graph.getDeformation(n_w_vector[2].first).d(),
									deformation_graph.getDeformation(n_w_vector[3].first).d());
}


const ceres::ResidualBlockId & pointToPointCost(ceres::Problem & problem,
												double loss_weighting,
												double k_neighbors,
												DeformationGraph<ARAPDeformation> & deformation_graph,
												DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
												const vertex_descriptor &v,
												const Point & target_point)
{
	if (k_neighbors == 3)
		return pointToPointCost3NN(problem, loss_weighting, deformation_graph, deformed_mesh, v, target_point);
	else if (k_neighbors == 4)
		return pointToPointCost4NN(problem, loss_weighting, deformation_graph, deformed_mesh, v, target_point);
	else {
		std::string message = "no cost function for " + std::to_string(k_neighbors) + " interpolation neighbors";
		throw std::exception(message.c_str());
	}
}

// point to plane


const ceres::ResidualBlockId & pointToPlaneCost3NN(ceres::Problem & problem,
												   double loss_weighting,
												   DeformationGraph<ARAPDeformation> & deformation_graph,
												   DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
												   const vertex_descriptor &v,
												   const Point & target_point,
												   const Vector & target_normal)
{
	auto n_w_vector = deformed_mesh.nearestNodes(v).node_weight_vector;
	auto & global = deformation_graph._global;
	auto cost_function = FitStarPointToPlaneAngleAxisCostFunction::Create(target_point, target_normal,
																		  deformed_mesh.point(v), global.position(),
																		  deformation_graph.getDeformation(n_w_vector[0].first).position(),
																		  deformation_graph.getDeformation(n_w_vector[1].first).position(),
																		  deformation_graph.getDeformation(n_w_vector[2].first).position(),
																		  n_w_vector[0].second, n_w_vector[1].second, n_w_vector[2].second);

	auto loss_function = new ceres::ScaledLoss(NULL, loss_weighting, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function,
									global.d(),
									deformation_graph.getDeformation(n_w_vector[0].first).d(),
									deformation_graph.getDeformation(n_w_vector[1].first).d(),
									deformation_graph.getDeformation(n_w_vector[2].first).d());
}

const ceres::ResidualBlockId & pointToPlaneCost4NN(ceres::Problem & problem,
												   double loss_weighting,
												   DeformationGraph<ARAPDeformation> & deformation_graph,
												   DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
												   const vertex_descriptor &v,
												   const Point & target_point,
												   const Vector & target_normal)
{
	auto n_w_vector = deformed_mesh.nearestNodes(v).node_weight_vector;
	auto & global = deformation_graph._global;
	auto cost_function = FitStarPointToPlaneAngleAxisCostFunction::Create(target_point, target_normal,
																		  deformed_mesh.point(v), global.position(),
																		  deformation_graph.getDeformation(n_w_vector[0].first).position(),
																		  deformation_graph.getDeformation(n_w_vector[1].first).position(),
																		  deformation_graph.getDeformation(n_w_vector[2].first).position(),
																		  deformation_graph.getDeformation(n_w_vector[3].first).position(),
																		  n_w_vector[0].second, n_w_vector[1].second, n_w_vector[2].second, n_w_vector[3].second);

	auto loss_function = new ceres::ScaledLoss(NULL, loss_weighting, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function,
									global.d(),
									deformation_graph.getDeformation(n_w_vector[0].first).d(),
									deformation_graph.getDeformation(n_w_vector[1].first).d(),
									deformation_graph.getDeformation(n_w_vector[2].first).d(),
									deformation_graph.getDeformation(n_w_vector[3].first).d());
}

const ceres::ResidualBlockId & pointToPlaneCost(ceres::Problem & problem,
												double loss_weighting,
												double k_neighbors,
												DeformationGraph<ARAPDeformation> & deformation_graph,
												DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
												const vertex_descriptor &v,
												const Point & target_point,
												const Vector & target_normal)
{
	if (k_neighbors == 3)
		return pointToPlaneCost3NN(problem, loss_weighting, deformation_graph, deformed_mesh, v, target_point, target_normal);
	else if (k_neighbors == 4)
		return pointToPlaneCost4NN(problem, loss_weighting, deformation_graph, deformed_mesh, v, target_point, target_normal);
	else {
		std::string message = "no cost function for " + std::to_string(k_neighbors) + " interpolation neighbors";
		throw std::exception(message.c_str());
	}
}


//-----------------------------------------------------------------------------


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

std::map<vertex_descriptor, ResidualIds> AsRigidAsPossibleFitCost::addFitCost(ceres::Problem &problem, 
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






ResidualIds AsRigidAsPossibleFitCostWithoutICP::addFitCostVertex(ceres::Problem & problem,
														         DeformationGraph<ARAPDeformation> & deformation_graph, 
														         DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh, 
														         SurfaceMesh::Vertex_index & v)
{
	auto target_normals = _target.property_map<vertex_descriptor, Vector>("v:normal").first;

	ResidualIds residual_ids;
	double point_to_point_weighting = _options.fit * 0.1;
	double point_to_plane_weighting = _options.fit * 0.9;

	auto residual_id_point_to_point = pointToPointCost(problem, point_to_point_weighting, _options.dg_options.number_of_interpolation_neighbors,
													   deformation_graph, deformed_mesh, v,
													   _target.point(v));
	residual_ids.push_back(residual_id_point_to_point);

	assert(target_normal.squared_length() > 0.);
	auto residual_id_point_to_plane = pointToPlaneCost(problem, point_to_plane_weighting, _options.dg_options.number_of_interpolation_neighbors,
													   deformation_graph, deformed_mesh, v,
													   _target.point(v), target_normals[v]);
	residual_ids.push_back(residual_id_point_to_plane);
	return residual_ids;
}


std::map<vertex_descriptor, ResidualIds> AsRigidAsPossibleFitCostWithoutICP::addFitCost(ceres::Problem &problem,
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