#include "embedded_deformation.h"
#include "embedded_deformation_cost_function.h"
#include "non_rigid_registration_cost_function.h"
//#include "algo/se3.h"
#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/connected_components.hpp"
#include "algo/ceres_iteration_logger.h"

namespace ED {

const Mesh & EmbeddedDeformation::getSource()
{
	return _src;
}

const Mesh & EmbeddedDeformation::getTarget()
{
	return _dst;
}



Mesh EmbeddedDeformation::getInverseDeformedPoints()
{
	auto inverse_deformation = inverteDeformationGraph(_deformation_graph);
	return inverse_deformation.deformPoints(_dst);
}

Mesh EmbeddedDeformation::getDeformedPoints()
{
	return _deformation_graph.deformPoints(_src);
}

EmbeddedDeformationGraph & EmbeddedDeformation::getDeformationGraph()
{
	return _deformation_graph;// .getDeformationGraph();
}

void EmbeddedDeformation::solveIteration()
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
			Node& src_i = nodes[*vp.first];

			ml::vec3f pos = src_i.deformedPosition();
			ml::vec3f pos_deformed = _deformation_graph._global_rigid_deformation.deformPosition(pos);
			unsigned int i = _nn_search.nearest_index(pos_deformed);

			double weight = a_fit;
			// point to point cost function
			ceres::CostFunction* cost_function_point_to_point = FitStarPointToPointCostFunction::Create(_dst.getVertices()[i].position, src_i.g(), global_node.g());
			auto loss_function_point_to_point = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);
			problem.AddResidualBlock(cost_function_point_to_point, loss_function_point_to_point,
									 global_node.r(), global_node.t(), src_i.t(), src_i.w());

			// point to plane cost function
			ceres::CostFunction* cost_function_point_to_plane = FitStarPointToPlaneCostFunction::Create(_dst.getVertices()[i].position, src_i.g(), src_i.n(), global_node.g());
			auto loss_function_point_to_plane = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);
			problem.AddResidualBlock(cost_function_point_to_plane, loss_function_point_to_plane,
									 global_node.r(), global_node.t(),
									 src_i.r(), src_i.t(),
									 src_i.w());
		}
		for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first) {
			Node& src_i = nodes[*vp.first];
			for (auto avp = boost::adjacent_vertices(*vp.first, g); avp.first != avp.second; ++avp.first) {
				Node& src_j = nodes[*avp.first];
				ceres::CostFunction* cost_function = SmoothCostFunction::Create(src_i.g(), src_j.g());
				auto loss_function = new ceres::ScaledLoss(NULL, a_smooth, ceres::TAKE_OWNERSHIP);
				problem.AddResidualBlock(cost_function, loss_function, src_i.r(), src_i.t(), src_j.t());
			}
		}
		for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first)
		{
			Node& src_i = nodes[*vp.first];
			ceres::CostFunction* cost_function = RotationCostFunction::Create();
			auto loss_function = new ceres::ScaledLoss(NULL, a_rigid, ceres::TAKE_OWNERSHIP);
			problem.AddResidualBlock(cost_function, loss_function, src_i.r());
		}
		for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first)
		{
			Node& src_i = nodes[*vp.first];
			ceres::CostFunction* cost_function = ConfCostFunction::Create();
			auto loss_function = new ceres::ScaledLoss(NULL, a_conf, ceres::TAKE_OWNERSHIP);
			problem.AddResidualBlock(cost_function, loss_function, src_i.w());
		}
		ceres::CostFunction* cost_function = RotationCostFunction::Create();
		problem.AddResidualBlock(cost_function, NULL, global_node.r());

		ceres::Solve(_options, &problem, &summary);

		_last_cost = _current_cost;
		_current_cost = summary.final_cost;

		if (abs(_current_cost - _last_cost) < 0.00001 *(1 + _current_cost) &&
			(a_rigid > 1 || a_smooth > 0.1 || a_conf > 1.))
		{
			a_rigid /= 2.;
			a_smooth /= 2.;
			a_conf /= 2.;
		}

		_total_time_in_ms += logger.get_time_in_ms();
	}
}

Mesh EmbeddedDeformation::solve()
{
	ml::mat4f transformation = ml::mat4f::identity();
	while (!finished()) {
		solveIteration();
	}
	return getDeformedPoints();
}

bool EmbeddedDeformation::finished()
{
	//double tol = 0.000001;
	double tol = 0.0000001;
	return (_solve_iteration >= _max_iterations) ||
		(abs(_last_cost - _current_cost) < (tol * _current_cost) && _solve_iteration > 5);
}

EmbeddedDeformation::EmbeddedDeformation(const Mesh& src,
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

EmbeddedDeformation::EmbeddedDeformation(const Mesh& src,
										 const Mesh& dst,
										 const EmbeddedDeformationGraph & deformation_graph,
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


























const Mesh & EmbeddedDeformationWithoutICP::getSource()
{
	return _src;
}

const Mesh & EmbeddedDeformationWithoutICP::getTarget()
{
	return _dst;
}

std::vector<ml::vec3f> EmbeddedDeformationWithoutICP::getFixedPostions()
{
	std::vector<ml::vec3f> positions;
	for (auto & i : _fixed_positions) {
		positions.push_back(_dst.getVertices()[i].position);
	}
	return positions;
}

Mesh EmbeddedDeformationWithoutICP::getInverseDeformedPoints()
{
	auto inverse_deformation = inverteDeformationGraph(_deformation_graph);
	return inverse_deformation.deformPoints(_dst);
}

Mesh EmbeddedDeformationWithoutICP::getDeformedPoints()
{
	return _deformation_graph.deformPoints(_src);
}

EmbeddedDeformationGraph & EmbeddedDeformationWithoutICP::getDeformationGraph()
{
	return _deformation_graph;
}

void EmbeddedDeformationWithoutICP::solveIteration()
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
			Node& src_i = nodes[*vp.first];

			if (_fixed_positions.empty() || (std::find(_fixed_positions.begin(), _fixed_positions.end(), src_i.index()) != _fixed_positions.end()))
			{
				ml::vec3f pos = src_i.deformedPosition();
				ml::vec3f pos_deformed = _deformation_graph._global_rigid_deformation.deformPosition(pos);
				unsigned int i = src_i.index();

				double weight = a_fit;
				// point to point cost function
				ceres::CostFunction* cost_function_point_to_point = FitStarPointToPointCostFunction::Create(_dst.getVertices()[i].position, src_i.g(), global_node.g());
				auto loss_function_point_to_point = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);
				problem.AddResidualBlock(cost_function_point_to_point, loss_function_point_to_point,
										 global_node.r(), global_node.t(), src_i.t(), src_i.w());

				// point to plane cost function
				ceres::CostFunction* cost_function_point_to_plane = FitStarPointToPlaneCostFunction::Create(_dst.getVertices()[i].position, src_i.g(), src_i.n(), global_node.g());
				auto loss_function_point_to_plane = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);
				problem.AddResidualBlock(cost_function_point_to_plane, loss_function_point_to_plane,
										 global_node.r(), global_node.t(),
										 src_i.r(), src_i.t(),
										 src_i.w());
			}
		}
		// smooth cost
		for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first) {
			Node& src_i = nodes[*vp.first];
			for (auto avp = boost::adjacent_vertices(*vp.first, g); avp.first != avp.second; ++avp.first) {
				Node& src_j = nodes[*avp.first];
				ceres::CostFunction* cost_function = SmoothCostFunction::Create(src_i.g(), src_j.g());
				auto loss_function = new ceres::ScaledLoss(NULL, a_smooth, ceres::TAKE_OWNERSHIP);
				problem.AddResidualBlock(cost_function, loss_function, src_i.r(), src_i.t(), src_j.t());
			}
		}
		// rotation cost
		for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first)
		{
			Node& src_i = nodes[*vp.first];
			ceres::CostFunction* cost_function = RotationCostFunction::Create();
			auto loss_function = new ceres::ScaledLoss(NULL, a_rigid, ceres::TAKE_OWNERSHIP);
			problem.AddResidualBlock(cost_function, loss_function, src_i.r());
		}
		// confident cost
		//for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first)
		//{
		//	Node& src_i = nodes[*vp.first];
		//	ceres::CostFunction* cost_function = ConfCostFunction::Create();
		//	auto loss_function = new ceres::ScaledLoss(NULL, a_conf, ceres::TAKE_OWNERSHIP);
		//	problem.AddResidualBlock(cost_function, loss_function, src_i.w());
		//}
		ceres::CostFunction* cost_function = RotationCostFunction::Create();
		problem.AddResidualBlock(cost_function, NULL, global_node.r());

		ceres::Solve(_options, &problem, &summary);

		_last_cost = _current_cost;
		_current_cost = summary.final_cost;

		if (abs(_current_cost - _last_cost) < 0.00001 *(1 + _current_cost) &&
			(a_rigid > 1 || a_smooth > 0.1 || a_conf > 1.))
		{
			a_rigid /= 2.;
			a_smooth /= 2.;
			a_conf /= 2.;
		}

		_total_time_in_ms += logger.get_time_in_ms();
	}
}

Mesh EmbeddedDeformationWithoutICP::solve()
{
	ml::mat4f transformation = ml::mat4f::identity();
	while (!finished()) {
		solveIteration();
	}
	return getDeformedPoints();
}

bool EmbeddedDeformationWithoutICP::finished()
{
	//double tol = 0.000001;
	double tol = 0.000001;
	double error = abs(_last_cost - _current_cost);
	bool solved = error < (tol * _current_cost);
	return (_solve_iteration >= _max_iterations) || (solved && _solve_iteration > 5);
}

EmbeddedDeformationWithoutICP::EmbeddedDeformationWithoutICP(const Mesh& src,
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



}