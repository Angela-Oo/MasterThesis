#include "stdafx.h"

#include "as_rigid_as_possible.h"
#include "as_rigid_as_possible_cost_function.h"
#include "non_rigid_registration_cost_function.h"
#include "algo/ceres_iteration_logger.h"
#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/connected_components.hpp"
#include "algo/mesh_simplification/mesh_simplification.h"




const Mesh & AsRigidAsPossible::getSource()
{
	return _src;
}

const Mesh & AsRigidAsPossible::getTarget()
{
	return _dst;
}

Mesh AsRigidAsPossible::getDeformedPoints()
{
	return _deformed_mesh->deformPoints();
}

Mesh AsRigidAsPossible::getInverseDeformedPoints()
{
	auto inverse_deformation = inverteDeformationGraph(_deformation_graph);
	ARAPDeformedMesh deformed(_dst, inverse_deformation);
	return deformed.deformPoints();
}


void AsRigidAsPossible::updateMeanCost()
{
	auto & g = _deformation_graph._graph;
	auto & nodes = boost::get(node_t(), g);
	auto & graph_edges = boost::get(edge_t(), g);

	double max_fit_cost = 0.0;
	double max_conf_cost = 0.0;
	double max_smooth_cost = 0.0;
	double mean_fit_cost = 0.;
	double mean_smooth_cost = 0.;
	for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first)
	{
		mean_fit_cost += nodes[*vp.first]._fit_cost;

		if (nodes[*vp.first]._fit_cost > max_fit_cost)
			max_fit_cost = nodes[*vp.first]._fit_cost;
		if (nodes[*vp.first]._conf_cost > max_conf_cost)
			max_conf_cost = nodes[*vp.first]._conf_cost;
	}
	mean_fit_cost /= boost::num_vertices(g);

	boost::graph_traits<ARAPGraph>::edge_iterator ei, ei_end;
	for (boost::tie(ei, ei_end) = boost::edges(g); ei != ei_end; ++ei)
	{
		mean_smooth_cost += graph_edges[*ei]._smooth_cost;

		if (graph_edges[*ei]._smooth_cost > max_smooth_cost)
			max_smooth_cost = graph_edges[*ei]._smooth_cost;
	}
	mean_smooth_cost /= boost::num_edges(g);

	_fit_mean_cost = mean_fit_cost;
	_smooth_mean_cost = mean_smooth_cost;

	_k_mean_cost = std::max(_fit_mean_cost, _smooth_mean_cost);
	_k_mean_cost *= 10.;

	std::cout << "max fit cost " << max_fit_cost << " max conf cost " << max_conf_cost << " max smooth cost " << max_smooth_cost << " used visualize cost " << _k_mean_cost << std::endl;
}

std::vector<Edge> AsRigidAsPossible::getDeformationGraph()
{
	std::vector<Edge> edges;
	auto & g = _deformation_graph._graph;
	auto & graph_edges = boost::get(edge_t(), g);
	auto & nodes = boost::get(node_t(), g);
	boost::graph_traits<ARAPGraph>::edge_iterator ei, ei_end;
	for (boost::tie(ei, ei_end) = boost::edges(g); ei != ei_end; ++ei)
	{
		Edge e;
		auto & edge = graph_edges[*ei];

		auto vertex_i = _deformation_graph.deformNode(boost::source(*ei, g));
		e.source_point = vertex_i.position;

		auto vertex_j = _deformation_graph.deformNode(boost::target(*ei, g));
		e.target_point = vertex_j.position;
		e.cost = edge._smooth_cost;
		if (_k_mean_cost > 0.)
			e.cost /= _k_mean_cost;
		e.cost = std::min(1., e.cost);
		edges.push_back(e);

		// test nearest point
		//Edge e;
		//auto & edge = graph_edges[*ei];

		//auto node = nodes[boost::source(*ei, g)];
		//auto vertex_i = _deformation_graph.deformNode(boost::source(*ei, g));
		//e.source_point = vertex_i.position;

		//e.target_point = node._nearest_point;// vertex_j.position;
		//if (dist(node._nearest_point, ml::vec3f::origin) < 0.0001)
		//	e.target_point = e.source_point;
		//
		//e.cost = (edge.residual().empty()) ? 0. : edge.residual()[0];
		//edges.push_back(e);
	}
	return edges;
}


Mesh AsRigidAsPossible::getDeformationGraphMesh()
{
	Mesh mesh;
	auto & g = _deformation_graph._graph;
	auto & nodes = boost::get(node_t(), g);

	for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first) {

		Mesh::Vertex vertex = _deformation_graph.deformNode(*vp.first);
		auto & node = nodes[*vp.first];
		double error = node._fit_cost;
		if (_k_mean_cost > 0.)
			error /= _k_mean_cost;
		error = std::min(1., error);
		vertex.color = errorToRGB(error, nodes[*vp.first].weight());		

		if (node.weight() < 0.7)
			vertex.color = ml::RGBColor::White.toVec4f();
		else if (!node._found_nearest_point)
			vertex.color = ml::RGBColor::Black.toVec4f();
		mesh.m_vertices.push_back(vertex);
	}
	return mesh;
}

DeformationGraph<ARAPGraph, ARAPNode> & AsRigidAsPossible::getARAPDeformationGraph()
{
	return _deformation_graph;
}

std::vector<ml::vec3f> AsRigidAsPossible::getFixedPostions()
{
	std::vector<ml::vec3f> positions;
	for (auto & i : _fixed_positions) {
		positions.push_back(_dst.getVertices()[i].position);
	}
	return positions;
}

ceres::ResidualBlockId AsRigidAsPossible::addPointToPointCostForNode(ceres::Problem &problem, ARAPNode & node, ml::vec3f target_position)
{
	float point_to_point_weighting = 0.1;
	double weight = a_fit * point_to_point_weighting;	
	auto & global_node = _deformation_graph._global_rigid_deformation;

	ceres::CostFunction* cost_function = FitStarPointToPointAngleAxisCostFunction::Create(target_position, node.g(), global_node.g());
	auto loss_function = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);

	return problem.AddResidualBlock(cost_function, loss_function, global_node.r(), global_node.t(), node.t(), node.w());
}

ceres::ResidualBlockId AsRigidAsPossible::addPointToPlaneCostForNode(ceres::Problem &problem, ARAPNode & node, ml::vec3f target_position)
{
	float point_to_plane_weighting = 0.9;
	double weight = a_fit * point_to_plane_weighting;
	auto & global_node = _deformation_graph._global_rigid_deformation;

	ceres::CostFunction* cost_function = FitStarPointToPlaneAngleAxisCostFunction::Create(target_position, node.g(), node.n(), global_node.g());
	auto loss_function = new ceres::ScaledLoss(NULL, point_to_plane_weighting * weight, ceres::TAKE_OWNERSHIP);

	return problem.AddResidualBlock(cost_function, loss_function, global_node.r(), global_node.t(), node.r(), node.t(), node.w());
}


ARAPVertexResidualIds AsRigidAsPossible::addFitCostWithoutICP(ceres::Problem &problem)
{
	ARAPVertexResidualIds residual_ids;
	auto & nodes = boost::get(node_t(), _deformation_graph._graph);

	for (auto vp = boost::vertices(_deformation_graph._graph); vp.first != vp.second; ++vp.first) {
		auto vertex_handle = *vp.first;
		auto& node = nodes[*vp.first];
		if (_fixed_positions.empty() || (std::find(_fixed_positions.begin(), _fixed_positions.end(), node.index()) != _fixed_positions.end()))
		{
			unsigned int i = node.index();

			auto residual_id_point_to_point = addPointToPointCostForNode(problem, node, _dst.getVertices()[i].position);
			residual_ids[vertex_handle].push_back(residual_id_point_to_point);

			auto residual_id_point_to_plane = addPointToPlaneCostForNode(problem, node, _dst.getVertices()[i].position);			
			residual_ids[vertex_handle].push_back(residual_id_point_to_plane);
		}
	}
	return residual_ids;
}


ARAPVertexResidualIds AsRigidAsPossible::addFitCost(ceres::Problem &problem)
{
	ARAPVertexResidualIds residual_ids;

	auto & nodes = boost::get(node_t(), _deformation_graph._graph);
	int i = 0;

	//std::cout << "median dist " << _find_correspondence_point->median() << std::endl;
	for (auto vp = boost::vertices(_deformation_graph._graph); vp.first != vp.second; ++vp.first) {
		auto vertex_handle = *vp.first;
		auto& node = nodes[vertex_handle];
		auto vertex = _deformation_graph.deformNode(vertex_handle);		

		auto correspondent_point = _find_correspondence_point->correspondingPoint(vertex.position, vertex.normal);
				
		if (correspondent_point.first) {
			auto target_position = correspondent_point.second;
			node._nearest_point = target_position;
			node._found_nearest_point = true;
			i++;

			residual_ids[vertex_handle].push_back(addPointToPointCostForNode(problem, node, target_position));
			residual_ids[vertex_handle].push_back(addPointToPlaneCostForNode(problem, node, target_position));
		}
		else {
			node._nearest_point = vertex.position;
			node._found_nearest_point = false;
		}
	}
	//std::cout << "used " << i << " of " << _deformation_graph._graph.m_vertices.size() << " deformation graph nodes" << std::endl;
	return residual_ids;
}

ARAPEdgeResidualIds AsRigidAsPossible::addAsRigidAsPossibleCost(ceres::Problem &problem)
{
	ARAPEdgeResidualIds residual_ids;

	auto & g = _deformation_graph._graph;
	auto & nodes = boost::get(node_t(), g);
	for (auto ep = boost::edges(g); ep.first != ep.second; ++ep.first) {	
		auto edge_index = (*ep.first);
		auto vi = boost::source(edge_index, g);
		auto vj = boost::target(edge_index, g);

		auto& src_i = nodes[vi];
		auto& src_j = nodes[vj];

		auto & e = boost::get(edge_t(), g)[edge_index];
		
		auto angle = [](ml::vec3f v1, ml::vec3f v2, ml::vec3f v3)
		{
			auto edge1 = (v3 - v1).getNormalized();
			auto edge2 = (v3 - v2).getNormalized();
			return acos(ml::vec3f::dot(edge1, edge2));
		};

		// todo!
		double alpha_ij = angle(src_i.deformedPosition(), src_j.deformedPosition(), nodes[e._triangle_vij].deformedPosition());
		double beta_ij = angle(src_j.deformedPosition(), src_i.deformedPosition(), nodes[e._triangle_vji].deformedPosition());
		double wij = 0.5 * (cos(alpha_ij) + atan(beta_ij));
		double wji = 0.5 * (cos(beta_ij) + atan(alpha_ij));
		double weight_ij = a_smooth * wij;
		double weight_ji = a_smooth * wji;

		ceres::CostFunction* cost_function = AsRigidAsPossibleCostFunction::Create(src_i.g(), src_j.g());
		auto loss_function = new ceres::ScaledLoss(NULL, weight_ij, ceres::TAKE_OWNERSHIP);
		auto residual_id_smooth_e1 = problem.AddResidualBlock(cost_function, loss_function, src_i.r(), src_i.t(), src_j.t());

		ceres::CostFunction* cost_function_j = AsRigidAsPossibleCostFunction::Create(src_j.g(), src_i.g());
		auto loss_function_j = new ceres::ScaledLoss(NULL, weight_ji, ceres::TAKE_OWNERSHIP);
		auto residual_id_smooth_e2 = problem.AddResidualBlock(cost_function_j, loss_function_j, src_j.r(), src_j.t(), src_i.t());

		residual_ids[edge_index].push_back(residual_id_smooth_e1);
		residual_ids[edge_index].push_back(residual_id_smooth_e2);
	}
	return residual_ids;
}

ARAPVertexResidualIds AsRigidAsPossible::addConfCost(ceres::Problem &problem)
{
	ARAPVertexResidualIds residual_ids;
	auto & g = _deformation_graph._graph;
	auto & nodes = boost::get(node_t(), g);

	for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first)
	{
		auto& src_i = nodes[*vp.first];
		ceres::CostFunction* cost_function = ConfCostFunction::Create();
		auto loss_function = new ceres::ScaledLoss(NULL, a_conf, ceres::TAKE_OWNERSHIP);
		auto residual_id = problem.AddResidualBlock(cost_function, loss_function, src_i.w());
		residual_ids[*vp.first].push_back(residual_id);
	}
	return residual_ids;
}

bool AsRigidAsPossible::solveIteration()
{

	if (!finished()) {
		_solve_iteration++;
		ceres::Solver::Summary summary;
		CeresIterationLoggerGuard logger(summary, _total_time_in_ms, _solve_iteration, _logger);

		ceres::Problem problem;

		ARAPVertexResidualIds fit_residual_ids;
		if (_with_icp)
			fit_residual_ids = addFitCost(problem);
		else			
			fit_residual_ids = addFitCostWithoutICP(problem);
		ARAPEdgeResidualIds arap_residual_ids = addAsRigidAsPossibleCost(problem);
		ARAPVertexResidualIds conf_residual_ids = addConfCost(problem);

		ceres::Solve(_options, &problem, &summary);

		// evaluate		
		evaluateResidual(problem, fit_residual_ids, arap_residual_ids, conf_residual_ids);

		_last_cost = _current_cost;
		_current_cost = summary.final_cost;

		auto scale_factor_tol = 0.001;// 0.00001;
		if (abs(_current_cost - _last_cost) < scale_factor_tol *(1 + _current_cost) &&
			(a_smooth > 0.1 && a_conf > 0.1))
		{
			a_smooth /= 2.;
			a_conf /= 2.;
			std::cout << std::endl <<"scale factor: smooth " << a_smooth << " conf " << a_conf << std::endl;
		}

		_total_time_in_ms += logger.get_time_in_ms();
	}
	updateMeanCost();
	return finished();
}


bool AsRigidAsPossible::solve()
{
	while (!finished()) {
		solveIteration();
	}
	return true;
}

bool AsRigidAsPossible::finished()
{
	auto tol = _options.function_tolerance;

	double error = abs(_last_cost - _current_cost);
	bool solved = error < (tol * _current_cost);
	return (_solve_iteration >= _max_iterations) || (solved && _solve_iteration > 2);
}

double AsRigidAsPossible::evaluateResidual(ceres::Problem & problem,
										   std::vector<ceres::ResidualBlockId> & residual_ids)
{
	ceres::Problem::EvaluateOptions evaluate_options;
	evaluate_options.residual_blocks = residual_ids;
	double total_cost = 0.0;
	std::vector<double> residuals;
	problem.Evaluate(evaluate_options, &total_cost, &residuals, nullptr, nullptr);
	return total_cost;
}

void AsRigidAsPossible::evaluateResidual(ceres::Problem & problem,
										 ARAPVertexResidualIds & fit_residual_block_ids,
										 ARAPEdgeResidualIds & arap_residual_block_ids,
										 ARAPVertexResidualIds & conf_residual_block_ids)
{
	auto & nodes = boost::get(node_t(), _deformation_graph._graph);
	auto & edges = boost::get(edge_t(), _deformation_graph._graph);
	for (auto & r : fit_residual_block_ids) {
		nodes[r.first]._fit_cost = evaluateResidual(problem, r.second);
	}
	for (auto & r : arap_residual_block_ids) {
		edges[r.first]._smooth_cost = evaluateResidual(problem, r.second);
	}
	for (auto & r : conf_residual_block_ids) {
		nodes[r.first]._conf_cost = evaluateResidual(problem, r.second);
	}
}


void AsRigidAsPossible::printCeresOptions()
{
	std::cout << "\nCeres Solver" << std::endl;
	std::cout << "Ceres preconditioner type: " << _options.preconditioner_type << std::endl;
	std::cout << "Ceres linear algebra type: " << _options.sparse_linear_algebra_library_type << std::endl;
	std::cout << "Ceres linear solver type: " << _options.linear_solver_type << std::endl;
}

AsRigidAsPossible::AsRigidAsPossible(const Mesh& src,
									 const Mesh& dst,
									 std::vector<int> fixed_positions,
									 ceres::Solver::Options option,
									 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)
	, _deformation_graph(src)
	, _fixed_positions(fixed_positions)
	, _logger(logger)
	, _with_icp(false)
	, a_smooth(10.)
	, a_fit(100.)
	, a_conf(100.)
{
	_find_correspondence_point = std::make_unique<FindCorrespondecePoint>(dst, _find_max_distance, _find_max_angle_deviation);
	_deformed_mesh = std::make_unique<ARAPDeformedMesh>(src, _deformation_graph);
	printCeresOptions();
}


AsRigidAsPossible::AsRigidAsPossible(const Mesh& src,
									 const Mesh& dst,
									 ceres::Solver::Options option,
									 unsigned int number_of_deformation_nodes,
									 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)	
	, _logger(logger)
{	
	_find_correspondence_point = std::make_unique<FindCorrespondecePoint>(dst, _find_max_distance, _find_max_angle_deviation);
	auto reduced_mesh = createReducedMesh(src, number_of_deformation_nodes);
	std::cout << "number of def nodes " << number_of_deformation_nodes << " true number " << reduced_mesh.m_vertices.size() << std::endl;
	_deformation_graph = ARAPDeformationGraph(reduced_mesh);
	_deformed_mesh = std::make_unique<ARAPDeformedMesh>(src, _deformation_graph);
	printCeresOptions();
}

AsRigidAsPossible::AsRigidAsPossible(const Mesh& src,
									 const Mesh& dst,
									 const ARAPDeformationGraph & deformation_graph,
									 ceres::Solver::Options option,
									 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)
	, _deformation_graph(deformation_graph)
	, _logger(logger)
{
	_find_correspondence_point = std::make_unique<FindCorrespondecePoint>(dst, _find_max_distance, _find_max_angle_deviation);
	_deformed_mesh = std::make_unique<ARAPDeformedMesh>(src, _deformation_graph);
	printCeresOptions();
}



