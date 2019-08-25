#include "arap_cost.h"
#include "arap_cost_functions.h"
#include "mesh/mesh_definition.h"

namespace Registration {

ceres::ResidualBlockId pointToPointCost3NN(ceres::Problem & problem,
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



ceres::ResidualBlockId pointToPointCost4NN(ceres::Problem & problem,
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



ceres::ResidualBlockId pointToPointCost5NN(ceres::Problem & problem,
										   double loss_weighting,
										   DeformationGraph<ARAPDeformation> & deformation_graph,
										   DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
										   const vertex_descriptor &v,
										   const Point & target_point)
{
	auto n_w_vector = deformed_mesh.nearestNodes(v).node_weight_vector;
	auto & global = deformation_graph._global;

	assert(n_w_vector.size() == 5);
	if (n_w_vector.size() < 5)
		std::cout << "help nearest node is smaller than expected" << std::endl;

	auto cost_function = FitStarPointToPointAngleAxisCostFunction::Create(target_point, deformed_mesh.point(v), global.position(),
																		  deformation_graph.getDeformation(n_w_vector[0].first).position(),
																		  deformation_graph.getDeformation(n_w_vector[1].first).position(),
																		  deformation_graph.getDeformation(n_w_vector[2].first).position(),
																		  deformation_graph.getDeformation(n_w_vector[3].first).position(),
																		  deformation_graph.getDeformation(n_w_vector[4].first).position(),
																		  n_w_vector[0].second, n_w_vector[1].second, n_w_vector[2].second, n_w_vector[3].second, n_w_vector[4].second);

	auto loss_function = new ceres::ScaledLoss(NULL, loss_weighting, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function,
									global.d(),
									deformation_graph.getDeformation(n_w_vector[0].first).d(),
									deformation_graph.getDeformation(n_w_vector[1].first).d(),
									deformation_graph.getDeformation(n_w_vector[2].first).d(),
									deformation_graph.getDeformation(n_w_vector[3].first).d(),
									deformation_graph.getDeformation(n_w_vector[4].first).d());
}



ceres::ResidualBlockId pointToPointCost(ceres::Problem & problem,
										double loss_weighting,
										DeformationGraph<ARAPDeformation> & deformation_graph,
										DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
										const vertex_descriptor &v,
										const Point & target_point)
{
	if (true) {
		auto n_w_vector = deformed_mesh.nearestNodes(v).node_weight_vector;
		if(n_w_vector.size() == 3)
			return pointToPointCost3NN(problem, loss_weighting, deformation_graph, deformed_mesh, v, target_point);
		else if(n_w_vector.size() == 4)
			return pointToPointCost4NN(problem, loss_weighting, deformation_graph, deformed_mesh, v, target_point);
		else if (n_w_vector.size() == 5)
			return pointToPointCost5NN(problem, loss_weighting, deformation_graph, deformed_mesh, v, target_point);
		else
			return pointToPointCost5NN(problem, loss_weighting, deformation_graph, deformed_mesh, v, target_point);
	}
	else {
		unsigned int k_neighbors = deformation_graph.getNumberOfInterpolationNeighbors();
		if (k_neighbors == 3)
			return pointToPointCost3NN(problem, loss_weighting, deformation_graph, deformed_mesh, v, target_point);
		else if (k_neighbors == 4)
			return pointToPointCost4NN(problem, loss_weighting, deformation_graph, deformed_mesh, v, target_point);
		else {
			std::string message = "no cost function for " + std::to_string(k_neighbors) + " interpolation neighbors";
			throw std::exception(message.c_str());
		}
	}
}

// point to plane


ceres::ResidualBlockId pointToPlaneCost3NN(ceres::Problem & problem,
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

ceres::ResidualBlockId pointToPlaneCost4NN(ceres::Problem & problem,
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


ceres::ResidualBlockId pointToPlaneCost5NN(ceres::Problem & problem,
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
																		  deformation_graph.getDeformation(n_w_vector[4].first).position(),
																		  n_w_vector[0].second, n_w_vector[1].second, n_w_vector[2].second, n_w_vector[3].second, n_w_vector[4].second);

	auto loss_function = new ceres::ScaledLoss(NULL, loss_weighting, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function,
									global.d(),
									deformation_graph.getDeformation(n_w_vector[0].first).d(),
									deformation_graph.getDeformation(n_w_vector[1].first).d(),
									deformation_graph.getDeformation(n_w_vector[2].first).d(),
									deformation_graph.getDeformation(n_w_vector[3].first).d(),
									deformation_graph.getDeformation(n_w_vector[4].first).d());
}


ceres::ResidualBlockId pointToPlaneCost(ceres::Problem & problem,
										double loss_weighting,
										DeformationGraph<ARAPDeformation> & deformation_graph,
										DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
										const vertex_descriptor &v,
										const Point & target_point,
										const Vector & target_normal)
{
	if (true) {
		auto n_w_vector = deformed_mesh.nearestNodes(v).node_weight_vector;
		if (n_w_vector.size() == 3)
			return pointToPlaneCost3NN(problem, loss_weighting, deformation_graph, deformed_mesh, v, target_point, target_normal);
		else if (n_w_vector.size() == 4)
			return pointToPlaneCost4NN(problem, loss_weighting, deformation_graph, deformed_mesh, v, target_point, target_normal);
		else if (n_w_vector.size() == 5)
			return pointToPlaneCost5NN(problem, loss_weighting, deformation_graph, deformed_mesh, v, target_point, target_normal);
		else {
			std::cout << "number of nearest nodes " << n_w_vector.size() << std::endl;
			return pointToPlaneCost5NN(problem, loss_weighting, deformation_graph, deformed_mesh, v, target_point, target_normal);
		}
	}
	else {
		unsigned int k_neighbors = deformation_graph.getNumberOfInterpolationNeighbors();
		if (k_neighbors == 3)
			return pointToPlaneCost3NN(problem, loss_weighting, deformation_graph, deformed_mesh, v, target_point, target_normal);
		else if (k_neighbors == 4)
			return pointToPlaneCost4NN(problem, loss_weighting, deformation_graph, deformed_mesh, v, target_point, target_normal);
		else {
			std::string message = "no cost function for " + std::to_string(k_neighbors) + " interpolation neighbors";
			throw std::exception(message.c_str());
		}
	}
}




ceres::ResidualBlockId asRigidAsPossibleCost(ceres::Problem & problem,
											 double loss_weighting,
											 DeformationGraph<ARAPDeformation> & deformation_graph,
											 const vertex_descriptor &source,
											 const vertex_descriptor &target)
{
	auto deformations = deformation_graph._mesh.property_map<vertex_descriptor, ARAPDeformation>("v:node_deformation").first;

	ceres::CostFunction* cost_function = AsRigidAsPossibleCostFunction::Create(deformation_graph._mesh.point(source), deformation_graph._mesh.point(target));
	auto loss_function = new ceres::ScaledLoss(NULL, loss_weighting, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function,
									deformations[source].d(), deformations[target].d());
}

}