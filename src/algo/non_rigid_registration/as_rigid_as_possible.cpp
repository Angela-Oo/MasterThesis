#include "as_rigid_as_possible.h"
#include "as_rigid_as_possible_cost_function.h"
#include "non_rigid_registration_cost_function.h"
#include "algo/se3.h"
#include "algo/ceres_iteration_logger.h"
#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/connected_components.hpp"


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
	return _deformation_graph.deformPoints(_src);
}

Mesh AsRigidAsPossible::getInverseDeformedPoints()
{
	auto inverse_deformation = inverteDeformationGraph(_deformation_graph);
	return inverse_deformation.deformPoints(_dst);
}

std::pair<std::vector<ml::vec3f>, std::vector<ml::vec3f>> AsRigidAsPossible::getDeformationGraph()
{
	return _deformation_graph.getDeformationGraphEdges();
}

DeformationGraph<ARAPGraph, ARAPNode> & AsRigidAsPossible::getARAPDeformationGraph()
{
	return _deformation_graph;
}

bool AsRigidAsPossible::solveIteration()
{
	if (!finished()) {
		_solve_iteration++;

		ceres::Solver::Summary summary;
		CeresIterationLoggerGuard logger(summary, _total_time_in_ms, _solve_iteration, _logger);

		ceres::Problem problem;
		auto & g = _deformation_graph._graph;
		auto & global_node = _deformation_graph._global_rigid_deformation;


		auto & nodes = boost::get(node_t(), g);

		for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first) {
			auto& src_i = nodes[*vp.first];

			ml::vec3f pos = src_i.deformedPosition();
			ml::vec3f pos_deformed = _deformation_graph._global_rigid_deformation.deformPosition(pos);
			unsigned int i = _nn_search.nearest_index(pos_deformed);

			double weight = a_fit;
			// point to point cost function
			ceres::CostFunction* cost_function_point_to_point = FitStarPointToPointAngleAxisCostFunction::Create(_dst.getVertices()[i].position, src_i.g(), global_node.g());
			auto loss_function_point_to_point = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);
			problem.AddResidualBlock(cost_function_point_to_point, loss_function_point_to_point,
				global_node.r(), global_node.t(), src_i.t(), src_i.w());

			// point to plane cost function
			ceres::CostFunction* cost_function_point_to_plane = FitStarPointToPlaneAngleAxisCostFunction::Create(_dst.getVertices()[i].position, src_i.g(), src_i.n(), global_node.g());
			auto loss_function_point_to_plane = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);
			problem.AddResidualBlock(cost_function_point_to_plane, loss_function_point_to_plane,
				global_node.r(), global_node.t(), src_i.r(), src_i.t(), src_i.w());
		}
		for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first) {
			auto& src_i = nodes[*vp.first];
			for (auto avp = boost::adjacent_vertices(*vp.first, g); avp.first != avp.second; ++avp.first) {
				auto& src_j = nodes[*avp.first];
				ceres::CostFunction* cost_function = AsRigidAsPossibleCostFunction::Create(src_i.g(), src_j.g());
				auto loss_function = new ceres::ScaledLoss(NULL, a_smooth, ceres::TAKE_OWNERSHIP);
				problem.AddResidualBlock(cost_function, loss_function, src_i.r(), src_i.t(), src_j.t());
			}
		}
		for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first)
		{
			auto& src_i = nodes[*vp.first];
			ceres::CostFunction* cost_function = ConfCostFunction::Create();
			auto loss_function = new ceres::ScaledLoss(NULL, a_conf, ceres::TAKE_OWNERSHIP);
			problem.AddResidualBlock(cost_function, loss_function, src_i.w());
		}

		ceres::Solve(_options, &problem, &summary);

		_last_cost = _current_cost;
		_current_cost = summary.final_cost;

		if (abs(_current_cost - _last_cost) < 0.00001 *(1 + _current_cost) &&
			(a_smooth > 0.1 || a_conf > 1.))
		{
			a_smooth /= 2.;
			a_conf /= 2.;
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
	//double tol = 0.000001;
	double tol = 0.0000001;
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
	, _deformation_graph(src, number_of_deformation_nodes)
	, _nn_search(dst)
	, _logger(logger)
{
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
	, _nn_search(dst)
	, _logger(logger)
{
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
	return _deformation_graph.deformPoints(_src);
}

Mesh AsRigidAsPossibleWithoutICP::getInverseDeformedPoints()
{
	auto inverse_deformation = inverteDeformationGraph(_deformation_graph);
	return inverse_deformation.deformPoints(_dst);
}

std::pair<std::vector<ml::vec3f>, std::vector<ml::vec3f>> AsRigidAsPossibleWithoutICP::getDeformationGraph()
{
	return _deformation_graph.getDeformationGraphEdges();
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


bool AsRigidAsPossibleWithoutICP::solveIteration()
{
	if (!finished()) {
		_solve_iteration++;

		ceres::Solver::Summary summary;
		CeresIterationLoggerGuard logger(summary, _total_time_in_ms, _solve_iteration, _logger);

		ceres::Problem problem;
		auto & g = _deformation_graph._graph;
		auto & global_node = _deformation_graph._global_rigid_deformation;
		auto & nodes = boost::get(node_t(), g);

		// fit cost
		for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first) {
			auto& src_i = nodes[*vp.first];
			if (_fixed_positions.empty() || (std::find(_fixed_positions.begin(), _fixed_positions.end(), src_i.index()) != _fixed_positions.end()))
			{
				unsigned int i = src_i.index();
				double weight = a_fit;
				// point to point cost function
				ceres::CostFunction* cost_function_point_to_point = FitStarPointToPointAngleAxisCostFunction::Create(_dst.getVertices()[i].position, src_i.g(), global_node.g());
				auto loss_function_point_to_point = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);
				problem.AddResidualBlock(cost_function_point_to_point, loss_function_point_to_point,
										 global_node.r(), global_node.t(), src_i.t(), src_i.w());



				// point to plane cost function
				ceres::CostFunction* cost_function_point_to_plane = FitStarPointToPlaneAngleAxisCostFunction::Create(_dst.getVertices()[i].position, src_i.g(), src_i.n(), global_node.g());
				auto loss_function_point_to_plane = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);
				problem.AddResidualBlock(cost_function_point_to_plane, loss_function_point_to_plane,
										 global_node.r(), global_node.t(), src_i.r(), src_i.t(), src_i.w());
			}
		}
		// as rigid as possible cost
		for (auto ep = boost::edges(g); ep.first != ep.second; ++ep.first) {
			auto vi = boost::source(*ep.first, g);
			auto vj = boost::target(*ep.first, g);

			auto& src_i = nodes[vi];
			auto& src_j = nodes[vj];
			ceres::CostFunction* cost_function = AsRigidAsPossibleCostFunction::Create(src_i.g(), src_j.g());
			auto loss_function = new ceres::ScaledLoss(NULL, a_smooth, ceres::TAKE_OWNERSHIP);
			problem.AddResidualBlock(cost_function, loss_function, src_i.r(), src_i.t(), src_j.t());

			ceres::CostFunction* cost_function_j = AsRigidAsPossibleCostFunction::Create(src_j.g(), src_i.g());
			auto loss_function_j = new ceres::ScaledLoss(NULL, a_smooth, ceres::TAKE_OWNERSHIP);
			problem.AddResidualBlock(cost_function_j, loss_function_j, src_j.r(), src_j.t(), src_i.t());
		}
		// conf cost
		for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first)
		{
			auto& src_i = nodes[*vp.first];
			ceres::CostFunction* cost_function = ConfCostFunction::Create();
			auto loss_function = new ceres::ScaledLoss(NULL, a_conf, ceres::TAKE_OWNERSHIP);
			problem.AddResidualBlock(cost_function, loss_function, src_i.w());
		}

		ceres::Solve(_options, &problem, &summary);

		_last_cost = _current_cost;
		_current_cost = summary.final_cost;

		if (abs(_current_cost - _last_cost) < 0.00001 *(1 + _current_cost) &&
			(a_smooth > 0.1 || a_conf > 1.))
		{
			a_smooth /= 2.;
			a_conf /= 2.;
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
	//double tol = 0.000001;
	double tol = 0.0000001;
	double error = abs(_last_cost - _current_cost);
	bool solved = error < (tol * _current_cost);
	return (_solve_iteration >= _max_iterations) || (solved && _solve_iteration > 2);
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
{
	std::cout << "\nCeres Solver" << std::endl;
	std::cout << "Ceres preconditioner type: " << _options.preconditioner_type << std::endl;
	std::cout << "Ceres linear algebra type: " << _options.sparse_linear_algebra_library_type << std::endl;
	std::cout << "Ceres linear solver type: " << _options.linear_solver_type << std::endl;
}