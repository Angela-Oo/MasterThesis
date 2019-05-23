#include "stdafx.h"

#include "embedded_deformation.h"
#include "embedded_deformation_cost_function.h"
#include "non_rigid_registration_cost_function.h"
#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/connected_components.hpp"
#include "algo/ceres_iteration_logger.h"
#include "algo/mesh_simplification/mesh_simplification.h"
#include "find_correspondece_point.h"

namespace ED {

const Mesh & EmbeddedDeformation::getSource()
{
	return _src;
}

const Mesh & EmbeddedDeformation::getTarget()
{
	return _dst;
}

Mesh EmbeddedDeformation::getDeformedPoints()
{
	return _deformed_mesh->deformPoints();
}

Mesh EmbeddedDeformation::getInverseDeformedPoints()
{
	auto inverse_deformation = inverteDeformationGraph(_deformation_graph);
	EmbeddedDeformedMesh deformed(_dst, inverse_deformation);
	return deformed.deformPoints();
}

std::vector<ml::vec3f> EmbeddedDeformation::getFixedPostions()
{
	std::vector<ml::vec3f> positions;
	for (auto & i : _fixed_positions) {
		positions.push_back(_dst.getVertices()[i].position);
	}
	return positions;
}


void EmbeddedDeformation::updateMeanCost()
{
	auto & g = _deformation_graph._graph;
	auto & nodes = boost::get(node_t(), g);
	auto & graph_edges = boost::get(edge_t(), g);

	double mean_fit_cost = 0.;
	double mean_smooth_cost = 0.;
	for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first)
	{
		mean_fit_cost += nodes[*vp.first]._fit_cost;
	}
	mean_fit_cost /= boost::num_vertices(g);
	boost::graph_traits<Graph>::edge_iterator ei, ei_end;
	for (boost::tie(ei, ei_end) = boost::edges(g); ei != ei_end; ++ei)
	{
		mean_smooth_cost += graph_edges[*ei]._smooth_cost;
	}
	mean_smooth_cost /= boost::num_edges(g);

	_k_mean_cost = std::max(mean_fit_cost, mean_smooth_cost);
	_k_mean_cost *= 10.;
}

std::vector<Edge> EmbeddedDeformation::getDeformationGraph()
{
	std::vector<Edge> edges;
	auto & g = _deformation_graph._graph;
	auto & graph_edges = boost::get(edge_t(), g);
	auto & nodes = boost::get(node_t(), g);
	boost::graph_traits<Graph>::edge_iterator ei, ei_end;
	for (boost::tie(ei, ei_end) = boost::edges(g); ei != ei_end; ++ei)
	{
		Edge e;
		auto & edge = graph_edges[*ei];
		auto vertex_i = _deformation_graph.deformNode(boost::source(*ei, g));
		e.source_point = vertex_i.position;
		auto vertex_j = _deformation_graph.deformNode(boost::target(*ei, g));
		e.target_point = vertex_j.position;
		e.cost = edge._smooth_cost;
		edges.push_back(e);
	}
	return edges;
}


Mesh EmbeddedDeformation::getDeformationGraphMesh()
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

EmbeddedDeformationGraph & EmbeddedDeformation::getEmbeddedDeformationGraph()
{
	return _deformation_graph;
}

double EmbeddedDeformation::evaluateResidual(ceres::Problem & problem,
											 std::vector<ceres::ResidualBlockId> & residual_ids)
{
	ceres::Problem::EvaluateOptions evaluate_options;
	evaluate_options.residual_blocks = residual_ids;
	double total_cost = 0.0;
	std::vector<double> residuals;
	problem.Evaluate(evaluate_options, &total_cost, &residuals, nullptr, nullptr);
	return total_cost;
}

void EmbeddedDeformation::evaluateResidual(ceres::Problem & problem,
										   VertexResidualIds & fit_residual_block_ids,
										   EdgeResidualIds & smooth_residual_block_ids,
										   VertexResidualIds & rotation_residual_block_ids,
										   VertexResidualIds & conf_residual_block_ids)
{
	auto & nodes = boost::get(node_t(), _deformation_graph._graph);
	auto & edges = boost::get(edge_t(), _deformation_graph._graph);
	for (auto & r : fit_residual_block_ids) {
		nodes[r.first]._fit_cost = evaluateResidual(problem, r.second);
	}
	for (auto & r : rotation_residual_block_ids) {
		nodes[r.first]._rotation_cost = evaluateResidual(problem, r.second);
	}
	for (auto & r : smooth_residual_block_ids) {
		edges[r.first]._smooth_cost = evaluateResidual(problem, r.second);
	}
	for (auto & r : conf_residual_block_ids) {
		nodes[r.first]._conf_cost = evaluateResidual(problem, r.second);
	}
}

ceres::ResidualBlockId EmbeddedDeformation::addPointToPointCostForNode(ceres::Problem &problem, Node & node, ml::vec3f & target_position)
{
	double weight = a_fit;
	auto & global_node = _deformation_graph._global_rigid_deformation;

	ceres::CostFunction* cost_function = FitStarPointToPointCostFunction::Create(target_position, node.g(), global_node.g());
	auto loss_function = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function, global_node.r(), global_node.t(), node.t(), node.w());
}

ceres::ResidualBlockId EmbeddedDeformation::addPointToPlaneCostForNode(ceres::Problem &problem, Node & node, ml::vec3f & target_position)
{
	double weight = a_fit;
	auto & global_node = _deformation_graph._global_rigid_deformation;

	ceres::CostFunction* cost_function = FitStarPointToPlaneCostFunction::Create(target_position, node.g(), node.n(), global_node.g());
	auto loss_function = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function, global_node.r(), global_node.t(), node.r(), node.t(), node.w());
}


VertexResidualIds EmbeddedDeformation::addFitCostWithoutICP(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;

	auto & nodes = boost::get(node_t(), _deformation_graph._graph);
	for (auto vp = boost::vertices(_deformation_graph._graph); vp.first != vp.second; ++vp.first) {
		auto vertex_handle = *vp.first;
		Node& node = nodes[vertex_handle];
		if (_fixed_positions.empty() || (std::find(_fixed_positions.begin(), _fixed_positions.end(), node.index()) != _fixed_positions.end()))
		{
			auto & target_position = _dst.getVertices()[node.index()].position;
			residual_ids[vertex_handle].push_back(addPointToPointCostForNode(problem, node, target_position));
			residual_ids[vertex_handle].push_back(addPointToPlaneCostForNode(problem, node, target_position));
		}
	}
	return residual_ids;
}


VertexResidualIds EmbeddedDeformation::addFitCost(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;

	auto & nodes = boost::get(node_t(), _deformation_graph._graph);
	for (auto vp = boost::vertices(_deformation_graph._graph); vp.first != vp.second; ++vp.first) {
		auto vertex_handle = *vp.first;
		Node& node = nodes[*vp.first];

		auto vertex = _deformation_graph.deformNode(vertex_handle);
		auto correspondent_point = _find_correspondence_point->correspondingPoint(vertex.position, vertex.normal);
		if (correspondent_point.first) {
			auto target_position = correspondent_point.second;
			node._nearest_point = target_position;
			node._found_nearest_point = true;			
			residual_ids[vertex_handle].push_back(addPointToPointCostForNode(problem, node, target_position));
			residual_ids[vertex_handle].push_back(addPointToPlaneCostForNode(problem, node, target_position));
		}
		else {
			node._nearest_point = vertex.position;
			node._found_nearest_point = false;
		}
	}
	return residual_ids;
}

ceres::ResidualBlockId EmbeddedDeformation::addSmoothCost(ceres::Problem &problem, Node & node_i, Node & node_j)
{
	ceres::CostFunction* cost_function = SmoothCostFunction::Create(node_i.g(), node_j.g());
	auto loss_function = new ceres::ScaledLoss(NULL, a_smooth, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function, node_i.r(), node_i.t(), node_j.t());
}

EdgeResidualIds EmbeddedDeformation::addSmoothCost(ceres::Problem &problem)
{
	EdgeResidualIds residual_ids;
	auto & g = _deformation_graph._graph;
	auto & nodes = boost::get(node_t(), g);
	for (auto ep = boost::edges(g); ep.first != ep.second; ++ep.first) {
		auto edge_index = (*ep.first);
		Node& node_i = nodes[boost::source(edge_index, g)];
		Node& node_j = nodes[boost::target(edge_index, g)];
		residual_ids[edge_index].push_back(addSmoothCost(problem, node_i, node_j));
		residual_ids[edge_index].push_back(addSmoothCost(problem, node_j, node_i));
	}
	return residual_ids;
}

ceres::ResidualBlockId EmbeddedDeformation::addRotationCost(ceres::Problem &problem, Node & node)
{
	ceres::CostFunction* cost_function = RotationCostFunction::Create();
	auto loss_function = new ceres::ScaledLoss(new ceres::SoftLOneLoss(0.001), a_rigid, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function, node.r());
}

VertexResidualIds EmbeddedDeformation::addRotationCost(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;

	auto & nodes = boost::get(node_t(), _deformation_graph._graph);
	for (auto vp = boost::vertices(_deformation_graph._graph); vp.first != vp.second; ++vp.first)
	{
		auto vertex_handle = *vp.first;
		residual_ids[vertex_handle].push_back(addRotationCost(problem, nodes[vertex_handle]));
	}
	return residual_ids;
}

VertexResidualIds EmbeddedDeformation::addConfCost(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;

	auto & nodes = boost::get(node_t(), _deformation_graph._graph);
	// confident cost
	for (auto vp = boost::vertices(_deformation_graph._graph); vp.first != vp.second; ++vp.first)
	{
		auto vertex_handle = *vp.first;
		Node& node = nodes[vertex_handle];
		ceres::CostFunction* cost_function = ConfCostFunction::Create();
		auto loss_function = new ceres::ScaledLoss(NULL, a_conf, ceres::TAKE_OWNERSHIP);
		residual_ids[vertex_handle].push_back(problem.AddResidualBlock(cost_function, loss_function, node.w()));
	}
	return residual_ids;
}

bool EmbeddedDeformation::solveIteration()
{
	if (!finished()) {
		_solve_iteration++;

		ceres::Solver::Summary summary;
		CeresIterationLoggerGuard logger(summary, _total_time_in_ms, _solve_iteration, _logger);
		ceres::Problem problem;

		// cost functions
		VertexResidualIds fit_residual_ids;
		if (_with_icp)
			fit_residual_ids = addFitCost(problem);
		else
			fit_residual_ids = addFitCostWithoutICP(problem);
		auto smooth_residual_ids = addSmoothCost(problem);
		auto rotation_residual_ids = addRotationCost(problem);
		auto conf_residual_ids = addConfCost(problem);

		// add global rotation cost
		addRotationCost(problem, _deformation_graph._global_rigid_deformation);		

		ceres::Solve(_options, &problem, &summary);

		evaluateResidual(problem, fit_residual_ids, smooth_residual_ids, rotation_residual_ids, conf_residual_ids);

		_last_cost = _current_cost;
		_current_cost = summary.final_cost;

		if (abs(_current_cost - _last_cost) < 0.00001 *(1 + _current_cost) &&
			(a_rigid > 1 || a_smooth > 0.1 || a_conf > 1.))
		{
			a_rigid /= 2.;
			a_smooth /= 2.;
			a_conf /= 2.;
			std::cout << "scale factor: smooth " << a_smooth << " rigid: " << a_rigid << std::endl;
		}

		_total_time_in_ms += logger.get_time_in_ms();
	}
	updateMeanCost();
	return finished();
}

bool EmbeddedDeformation::solve()
{
	while (!finished()) {
		solveIteration();
	}
	return true;
}

bool EmbeddedDeformation::finished()
{
	double tol = _options.function_tolerance;
	double error = abs(_last_cost - _current_cost);
	bool solved = error < (tol * _current_cost);
	return (_solve_iteration >= _max_iterations) || (solved && _solve_iteration > 2);
}

void EmbeddedDeformation::printCeresOptions()
{
	std::cout << "\nCeres Solver" << std::endl;
	std::cout << "Ceres preconditioner type: " << _options.preconditioner_type << std::endl;
	std::cout << "Ceres linear algebra type: " << _options.sparse_linear_algebra_library_type << std::endl;
	std::cout << "Ceres linear solver type: " << _options.linear_solver_type << std::endl;
}

EmbeddedDeformation::EmbeddedDeformation(const Mesh& src,
										 const Mesh& dst,
										 ceres::Solver::Options option,
										 unsigned int number_of_deformation_nodes,
										 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)
	, _logger(logger)
	, _with_icp(true)
{
	_find_correspondence_point = std::make_unique<FindCorrespondecePoint>(dst, 0.5, 45.);

	auto reduced_mesh = createReducedMesh(src, number_of_deformation_nodes);
	_deformation_graph = EmbeddedDeformationGraph(reduced_mesh);
	_deformed_mesh = std::make_unique<EmbeddedDeformedMesh>(src, _deformation_graph);
	printCeresOptions();
}

EmbeddedDeformation::EmbeddedDeformation(const Mesh& src,
										 const Mesh& dst,
										 const EmbeddedDeformationGraph & deformation_graph,
										 ceres::Solver::Options option,
										 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)
	, _deformation_graph(deformation_graph)
	, _logger(logger)
	, _with_icp(true)
{
	_find_correspondence_point = std::make_unique<FindCorrespondecePoint>(dst, 0.5, 45.);
	_deformed_mesh = std::make_unique<EmbeddedDeformedMesh>(src, _deformation_graph);
	printCeresOptions();
}




EmbeddedDeformation::EmbeddedDeformation(const Mesh& src,
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
	, a_rigid(1000.)
	, a_fit(10.)
{
	_deformed_mesh = std::make_unique<EmbeddedDeformedMesh>(src, _deformation_graph);
	printCeresOptions();
}



}