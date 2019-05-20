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

std::pair<std::vector<ml::vec3f>, std::vector<ml::vec3f>> AsRigidAsPossible::getDeformationGraph()
{
	return _deformation_graph.getDeformationGraphEdges();
}

DeformationGraph<ARAPGraph, ARAPNode> & AsRigidAsPossible::getARAPDeformationGraph()
{
	return _deformation_graph;
}

std::vector<ceres::ResidualBlockId> AsRigidAsPossible::addFitCost(ceres::Problem &problem)
{
	std::vector<ceres::ResidualBlockId> fit_residuals_ids;

	auto & g = _deformation_graph._graph;
	auto & global_node = _deformation_graph._global_rigid_deformation;
	auto & nodes = boost::get(node_t(), g);

	for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first) {
		auto& src_i = nodes[*vp.first];

		ml::vec3f pos = src_i.deformedPosition();
		ml::vec3f normal = src_i.deformedNormal();
		ml::vec3f pos_deformed = _deformation_graph._global_rigid_deformation.deformPosition(pos);
		ml::vec3f normal_deformed = _deformation_graph._global_rigid_deformation.deformPosition(normal);
		//auto correspondent_point = _find_correspondence_point.correspondingPoint(pos_deformed, normal_deformed);
		unsigned int i = _nn_search.nearest_index(pos_deformed);
		auto & correspondent_point = _dst.getVertices()[i].position;

		if (true) {//correspondent_point.first) {
			double weight = a_fit;
			// point to point cost function
			ceres::CostFunction* cost_function_point_to_point = FitStarPointToPointAngleAxisCostFunction::Create(correspondent_point, src_i.g(), global_node.g());
			auto loss_function_point_to_point = new ceres::ScaledLoss(NULL, 0.1 *weight, ceres::TAKE_OWNERSHIP);
			auto residual_id_point_to_point = problem.AddResidualBlock(cost_function_point_to_point, loss_function_point_to_point,
																	   global_node.r(), global_node.t(), src_i.t(), src_i.w());

			// point to plane cost function
			ceres::CostFunction* cost_function_point_to_plane = FitStarPointToPlaneAngleAxisCostFunction::Create(correspondent_point, src_i.g(), src_i.n(), global_node.g());
			auto loss_function_point_to_plane = new ceres::ScaledLoss(NULL, 0.9 *weight, ceres::TAKE_OWNERSHIP);
			auto residual_id_point_to_plane = problem.AddResidualBlock(cost_function_point_to_plane, loss_function_point_to_plane,
																	   global_node.r(), global_node.t(), src_i.r(), src_i.t(), src_i.w());

			fit_residuals_ids.push_back(residual_id_point_to_point);
			fit_residuals_ids.push_back(residual_id_point_to_plane);
		}
	}
	return fit_residuals_ids;
}

std::vector<ceres::ResidualBlockId> AsRigidAsPossible::addAsRigidAsPossibleCost(ceres::Problem &problem)
{
	std::vector<ceres::ResidualBlockId> smooth_residuals_ids;

	auto & g = _deformation_graph._graph;
	auto & nodes = boost::get(node_t(), g);

	// as rigid as possible cost
	for (auto ep = boost::edges(g); ep.first != ep.second; ++ep.first) {
		auto vi = boost::source(*ep.first, g);
		auto vj = boost::target(*ep.first, g);

		auto& src_i = nodes[vi];
		auto& src_j = nodes[vj];
		ceres::CostFunction* cost_function = AsRigidAsPossibleCostFunction::Create(src_i.g(), src_j.g());
		auto loss_function = new ceres::ScaledLoss(NULL, a_smooth, ceres::TAKE_OWNERSHIP);
		auto residual_id_smooth_e1 = problem.AddResidualBlock(cost_function, loss_function, src_i.r(), src_i.t(), src_j.t());

		ceres::CostFunction* cost_function_j = AsRigidAsPossibleCostFunction::Create(src_j.g(), src_i.g());
		auto loss_function_j = new ceres::ScaledLoss(NULL, a_smooth, ceres::TAKE_OWNERSHIP);
		auto residual_id_smooth_e2 = problem.AddResidualBlock(cost_function_j, loss_function_j, src_j.r(), src_j.t(), src_i.t());

		smooth_residuals_ids.push_back(residual_id_smooth_e1);
		smooth_residuals_ids.push_back(residual_id_smooth_e2);
	}

	return smooth_residuals_ids;
}

std::vector<ceres::ResidualBlockId> AsRigidAsPossible::addConfCost(ceres::Problem &problem)
{
	std::vector<ceres::ResidualBlockId> conf_residuals_ids;
	auto & g = _deformation_graph._graph;
	auto & nodes = boost::get(node_t(), g);
	for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first)
	{
		auto& src_i = nodes[*vp.first];
		ceres::CostFunction* cost_function = ConfCostFunction::Create();
		auto loss_function = new ceres::ScaledLoss(NULL, a_conf, ceres::TAKE_OWNERSHIP);
		auto residual_id = problem.AddResidualBlock(cost_function, loss_function, src_i.w());
		conf_residuals_ids.push_back(residual_id);
	}
	return conf_residuals_ids;
}

bool AsRigidAsPossible::solveIteration()
{
	if (!finished()) {
		_solve_iteration++;

		ceres::Solver::Summary summary;
		CeresIterationLoggerGuard logger(summary, _total_time_in_ms, _solve_iteration, _logger);

		ceres::Problem problem;
		
		std::vector<ceres::ResidualBlockId> fit_residuals_ids = addFitCost(problem);
		std::vector<ceres::ResidualBlockId> smooth_residuals_ids = addAsRigidAsPossibleCost(problem);
		std::vector<ceres::ResidualBlockId> conf_residuals_ids = addConfCost(problem);

		ceres::Solve(_options, &problem, &summary);

		_last_cost = _current_cost;
		_current_cost = summary.final_cost;
		
		auto scale_factor_tol = 0.00005;// 0.00001;
		if (abs(_current_cost - _last_cost) < scale_factor_tol * (1 + _current_cost) &&
			(a_smooth > 0.01 || a_conf > 1.))
		{
			a_smooth /= 2.;
			a_conf /= 2.;
			std::cout << "scale factor: smooth " << a_smooth << std::endl;
		}

		_total_time_in_ms += logger.get_time_in_ms();
	}
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

AsRigidAsPossible::AsRigidAsPossible(const Mesh& src,
									 const Mesh& dst,
									 ceres::Solver::Options option,
									 unsigned int number_of_deformation_nodes,
									 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)
	//, _find_correspondence_point(dst)
	, _nn_search(dst)
	, _logger(logger)
{
	auto reduced_mesh = createReducedMesh(src, number_of_deformation_nodes);

	_deformation_graph = ARAPDeformationGraph(reduced_mesh);
	_deformed_mesh = std::make_unique<ARAPDeformedMesh>(src, _deformation_graph);
	std::cout << "\nCeres Solver" << std::endl;
	std::cout << "Ceres preconditioner type: " << _options.preconditioner_type << std::endl;
	std::cout << "Ceres linear algebra type: " << _options.sparse_linear_algebra_library_type << std::endl;
	std::cout << "Ceres linear solver type: " << _options.linear_solver_type << std::endl;
}

AsRigidAsPossible::AsRigidAsPossible(const Mesh& src,
									 const Mesh& dst,
									 const ARAPDeformationGraph & deformation_graph,
									 ceres::Solver::Options option,
									 unsigned int number_of_deformation_nodes,
									 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)
	, _deformation_graph(deformation_graph)
	//, _find_correspondence_point(dst)
	, _nn_search(dst)
	, _logger(logger)
{
	_deformed_mesh = std::make_unique<ARAPDeformedMesh>(src, _deformation_graph);
	std::cout << "\nCeres Solver" << std::endl;
	std::cout << "Ceres preconditioner type: " << _options.preconditioner_type << std::endl;
	std::cout << "Ceres linear algebra type: " << _options.sparse_linear_algebra_library_type << std::endl;
	std::cout << "Ceres linear solver type: " << _options.linear_solver_type << std::endl;
}

















const Mesh & AsRigidAsPossibleWithoutICP::getSource()
{
	return _src;
}

const Mesh & AsRigidAsPossibleWithoutICP::getTarget()
{
	return _dst;
}

Mesh AsRigidAsPossibleWithoutICP::getDeformedPoints()
{
	return _deformed_mesh->deformPoints();
}

Mesh AsRigidAsPossibleWithoutICP::getInverseDeformedPoints()
{
	auto inverse_deformation = inverteDeformationGraph(_deformation_graph);
	ARAPDeformedMesh deformed(_dst, inverse_deformation);
	return deformed.deformPoints();
}

std::pair<std::vector<ml::vec3f>, std::vector<ml::vec3f>> AsRigidAsPossibleWithoutICP::getDeformationGraph()
{
	return _deformation_graph.getDeformationGraphEdges();
}

Mesh AsRigidAsPossibleWithoutICP::getDeformationGraphMesh()
{
	return _deformation_graph.getDeformationGraph();
}

DeformationGraph<ARAPGraph, ARAPNode> & AsRigidAsPossibleWithoutICP::getARAPDeformationGraph()
{
	return _deformation_graph;
}

std::vector<ml::vec3f> AsRigidAsPossibleWithoutICP::getFixedPostions()
{
	std::vector<ml::vec3f> positions;
	for (auto & i : _fixed_positions) {
		positions.push_back(_dst.getVertices()[i].position);
	}
	return positions;
}

void AsRigidAsPossibleWithoutICP::addFitCostWithoutICP(ceres::Problem &problem)
{
	auto & g = _deformation_graph._graph;
	auto & global_node = _deformation_graph._global_rigid_deformation;
	auto & nodes = boost::get(node_t(), g);

	float point_to_point_weighting = 0.1;
	float point_to_plane_weighting = 0.9;

	for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first) {
		auto& src_i = nodes[*vp.first];
		if (_fixed_positions.empty() || (std::find(_fixed_positions.begin(), _fixed_positions.end(), src_i.index()) != _fixed_positions.end()))
		{
			unsigned int i = src_i.index();
			double weight = a_fit;
			// point to point cost function
			ceres::CostFunction* cost_function_point_to_point = FitStarPointToPointAngleAxisCostFunction::Create(_dst.getVertices()[i].position, src_i.g(), global_node.g());
			auto loss_function_point_to_point = new ceres::ScaledLoss(NULL, point_to_point_weighting * weight, ceres::TAKE_OWNERSHIP);
			auto residual_id_point_to_point = problem.AddResidualBlock(cost_function_point_to_point, loss_function_point_to_point,
																	   global_node.r(), global_node.t(), src_i.t(), src_i.w());



			// point to plane cost function
			ceres::CostFunction* cost_function_point_to_plane = FitStarPointToPlaneAngleAxisCostFunction::Create(_dst.getVertices()[i].position, src_i.g(), src_i.n(), global_node.g());
			auto loss_function_point_to_plane = new ceres::ScaledLoss(NULL, point_to_plane_weighting * weight, ceres::TAKE_OWNERSHIP);
			auto residual_id_point_to_plane = problem.AddResidualBlock(cost_function_point_to_plane, loss_function_point_to_plane,
																	   global_node.r(), global_node.t(), src_i.r(), src_i.t(), src_i.w());

			_fit_point_to_point_residuals_ids.push_back(residual_id_point_to_point);
			_fit_point_to_plane_residuals_ids.push_back(residual_id_point_to_plane);
		}
	}
}


void AsRigidAsPossibleWithoutICP::addFitCost(ceres::Problem &problem)
{
	auto & g = _deformation_graph._graph;
	auto & global_node = _deformation_graph._global_rigid_deformation;
	auto & nodes = boost::get(node_t(), g);

	for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first) {
		auto& src_i = nodes[*vp.first];

		ml::vec3f pos = src_i.deformedPosition();
		ml::vec3f normal = src_i.deformedNormal();
		ml::vec3f pos_deformed = _deformation_graph._global_rigid_deformation.deformPosition(pos);
		ml::vec3f normal_deformed = _deformation_graph._global_rigid_deformation.deformPosition(normal);
		//auto correspondent_point = _find_correspondence_point.correspondingPoint(pos_deformed, normal_deformed);
		unsigned int i = _nn_search.nearest_index(pos_deformed);
		auto & correspondent_point = _dst.getVertices()[i].position;

		if (true) {//correspondent_point.first) {
			double weight = a_fit;
			// point to point cost function
			ceres::CostFunction* cost_function_point_to_point = FitStarPointToPointAngleAxisCostFunction::Create(correspondent_point, src_i.g(), global_node.g());
			auto loss_function_point_to_point = new ceres::ScaledLoss(NULL, 0.1 *weight, ceres::TAKE_OWNERSHIP);
			auto residual_id_point_to_point = problem.AddResidualBlock(cost_function_point_to_point, loss_function_point_to_point,
																	   global_node.r(), global_node.t(), src_i.t(), src_i.w());

			// point to plane cost function
			ceres::CostFunction* cost_function_point_to_plane = FitStarPointToPlaneAngleAxisCostFunction::Create(correspondent_point, src_i.g(), src_i.n(), global_node.g());
			auto loss_function_point_to_plane = new ceres::ScaledLoss(NULL, 0.9 *weight, ceres::TAKE_OWNERSHIP);
			auto residual_id_point_to_plane = problem.AddResidualBlock(cost_function_point_to_plane, loss_function_point_to_plane,
																	   global_node.r(), global_node.t(), src_i.r(), src_i.t(), src_i.w());

			_fit_point_to_point_residuals_ids.push_back(residual_id_point_to_point);
			_fit_point_to_plane_residuals_ids.push_back(residual_id_point_to_plane);
		}
	}
}

void AsRigidAsPossibleWithoutICP::addAsRigidAsPossibleCost(ceres::Problem &problem)
{
	auto & g = _deformation_graph._graph;
	auto & nodes = boost::get(node_t(), g);
	for (auto ep = boost::edges(g); ep.first != ep.second; ++ep.first) {
		auto vi = boost::source(*ep.first, g);
		auto vj = boost::target(*ep.first, g);

		auto& src_i = nodes[vi];
		auto& src_j = nodes[vj];
		ceres::CostFunction* cost_function = AsRigidAsPossibleCostFunction::Create(src_i.g(), src_j.g());
		auto loss_function = new ceres::ScaledLoss(NULL, a_smooth, ceres::TAKE_OWNERSHIP);
		auto residual_id_smooth_e1 = problem.AddResidualBlock(cost_function, loss_function, src_i.r(), src_i.t(), src_j.t());

		ceres::CostFunction* cost_function_j = AsRigidAsPossibleCostFunction::Create(src_j.g(), src_i.g());
		auto loss_function_j = new ceres::ScaledLoss(NULL, a_smooth, ceres::TAKE_OWNERSHIP);
		auto residual_id_smooth_e2 = problem.AddResidualBlock(cost_function_j, loss_function_j, src_j.r(), src_j.t(), src_i.t());

		_smooth_residuals_ids.push_back(residual_id_smooth_e1);
		_smooth_residuals_ids.push_back(residual_id_smooth_e2);
	}
}

void AsRigidAsPossibleWithoutICP::addConfCost(ceres::Problem &problem)
{
	auto & g = _deformation_graph._graph;
	auto & nodes = boost::get(node_t(), g);

	for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first)
	{
		auto& src_i = nodes[*vp.first];
		ceres::CostFunction* cost_function = ConfCostFunction::Create();
		auto loss_function = new ceres::ScaledLoss(NULL, a_conf, ceres::TAKE_OWNERSHIP);
		auto residual_id = problem.AddResidualBlock(cost_function, loss_function, src_i.w());
		_conf_residuals_ids.push_back(residual_id);
	}
}

bool AsRigidAsPossibleWithoutICP::solveIteration()
{

	if (!finished()) {
		_solve_iteration++;
		ceres::Solver::Summary summary;
		CeresIterationLoggerGuard logger(summary, _total_time_in_ms, _solve_iteration, _logger);

		_gradient.point.clear();
		auto & nodes = boost::get(node_t(), _deformation_graph._graph);
		for (auto vp = boost::vertices(_deformation_graph._graph); vp.first != vp.second; ++vp.first) {
			auto& src_i = nodes[*vp.first];
			ml::vec3f pos = _deformation_graph._global_rigid_deformation.deformPosition(src_i.deformedPosition());
			_gradient.point.push_back(pos);
		}
		_fit_point_to_point_residuals_ids.clear();
		_fit_point_to_plane_residuals_ids.clear();
		_smooth_residuals_ids.clear();
		_conf_residuals_ids.clear();

		ceres::Problem problem;

		if (_with_icp)
			addFitCost(problem);
		else			
			addFitCostWithoutICP(problem);
		addAsRigidAsPossibleCost(problem);
		addConfCost(problem);

		ceres::Solve(_options, &problem, &summary);

		// evaluate
		_gradient.fit_point_to_point_gradient = gradientOfResidualBlock(problem, _fit_point_to_point_residuals_ids);
		_gradient.fit_point_to_plane_gradient = gradientOfResidualBlock(problem, _fit_point_to_plane_residuals_ids);
		_gradient.smooth_gradient = gradientOfResidualBlock(problem, _smooth_residuals_ids);

		_last_cost = _current_cost;
		_current_cost = summary.final_cost;

		auto scale_factor_tol = 0.00005;// 0.00001;
		if (abs(_current_cost - _last_cost) < scale_factor_tol *(1 + _current_cost) &&
			(a_smooth > 0.1 || a_conf > 1.))
		{
			a_smooth /= 2.;
			a_conf /= 2.;
			std::cout << std::endl <<"scale factor: smooth " << a_smooth << " conf " << a_conf << std::endl;
		}

		_total_time_in_ms += logger.get_time_in_ms();
	}	
	return finished();
}


bool AsRigidAsPossibleWithoutICP::solve()
{
	while (!finished()) {
		solveIteration();
	}
	return true;
}

bool AsRigidAsPossibleWithoutICP::finished()
{
	auto tol = _options.function_tolerance;

	double error = abs(_last_cost - _current_cost);
	bool solved = error < (tol * _current_cost);
	return (_solve_iteration >= _max_iterations) || (solved && _solve_iteration > 2);
}

std::vector<NodeGradient> AsRigidAsPossibleWithoutICP::gradientOfResidualBlock(ceres::Problem & problem, std::vector<ceres::ResidualBlockId> & residual_block_ids)
{
	ceres::Problem::EvaluateOptions evaluate_options;
	evaluate_options.residual_blocks = residual_block_ids;

	auto & g = _deformation_graph._graph;

	auto & nodes = boost::get(node_t(), g);
	for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first) {
		auto& src_i = nodes[*vp.first];
		evaluate_options.parameter_blocks.push_back(src_i.r());
		evaluate_options.parameter_blocks.push_back(src_i.t());
		evaluate_options.parameter_blocks.push_back(src_i.w());
	}

	double total_cost = 0.0;
	std::vector<double> residuals;
	std::vector<double> gradients;

	problem.Evaluate(evaluate_options, &total_cost, &residuals, &gradients, nullptr);

	std::vector<NodeGradient> node_gradients;
	for (int i = 0; i < gradients.size(); i += 7)
	{
		NodeGradient n;
		n.rotation = { gradients[i], gradients[i + 1], gradients[i + 2] };
		n.translation = { gradients[i + 3], gradients[i + 4], gradients[i + 5] };
		n.w = gradients[i + 6];
		node_gradients.push_back(n);
	}
	return node_gradients;
}



ARAPGradient AsRigidAsPossibleWithoutICP::gradient()
{
	return _gradient;
}

void AsRigidAsPossibleWithoutICP::printCeresOptions()
{
	std::cout << "\nCeres Solver" << std::endl;
	std::cout << "Ceres preconditioner type: " << _options.preconditioner_type << std::endl;
	std::cout << "Ceres linear algebra type: " << _options.sparse_linear_algebra_library_type << std::endl;
	std::cout << "Ceres linear solver type: " << _options.linear_solver_type << std::endl;
}

AsRigidAsPossibleWithoutICP::AsRigidAsPossibleWithoutICP(const Mesh& src,
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
	, _nn_search(dst)
	, _with_icp(false)
	, a_smooth(10.)
	, a_fit(100.)
{
	_deformed_mesh = std::make_unique<ARAPDeformedMesh>(src, _deformation_graph);
	printCeresOptions();
}


AsRigidAsPossibleWithoutICP::AsRigidAsPossibleWithoutICP(const Mesh& src,
														 const Mesh& dst,
														 ceres::Solver::Options option,
														 unsigned int number_of_deformation_nodes,
														 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)
	//, _find_correspondence_point(dst)
	, _nn_search(dst)
	, _logger(logger)
{
	auto reduced_mesh = createReducedMesh(src, number_of_deformation_nodes);

	_deformation_graph = ARAPDeformationGraph(reduced_mesh);
	_deformed_mesh = std::make_unique<ARAPDeformedMesh>(src, _deformation_graph);
	printCeresOptions();
}

AsRigidAsPossibleWithoutICP::AsRigidAsPossibleWithoutICP(const Mesh& src,
														 const Mesh& dst,
														 const ARAPDeformationGraph & deformation_graph,
														 ceres::Solver::Options option,
														 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)
	, _deformation_graph(deformation_graph)
	, _nn_search(dst)
	, _logger(logger)
{
	_deformed_mesh = std::make_unique<ARAPDeformedMesh>(src, _deformation_graph);
	printCeresOptions();
}



