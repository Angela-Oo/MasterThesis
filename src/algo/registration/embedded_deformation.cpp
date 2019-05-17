#include "embedded_deformation.h"
#include "embedded_deformation_cost_function.h"
#include "non_rigid_registration_cost_function.h"
#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/connected_components.hpp"
#include "algo/ceres_iteration_logger.h"
#include "algo/mesh_simplification/mesh_simplification.h"

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
	//return _deformation_graph.deformPoints(_src);
}

Mesh EmbeddedDeformation::getInverseDeformedPoints()
{
	auto inverse_deformation = inverteDeformationGraph(_deformation_graph);

	EmbeddedDeformedMesh deformed(_dst, inverse_deformation);
	return deformed.deformPoints();
	//return inverse_deformation.deformPoints(_dst);
}

std::pair<std::vector<ml::vec3f>, std::vector<ml::vec3f>> EmbeddedDeformation::getDeformationGraph()
{
	return _deformation_graph.getDeformationGraphEdges();
}

EmbeddedDeformationGraph & EmbeddedDeformation::getEmbeddedDeformationGraph()
{
	return _deformation_graph;
}

bool EmbeddedDeformation::solveIteration()
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
									 global_node.r(), global_node.t(), src_i.r(), src_i.t(), src_i.w());
		}
		// smooth cost
		for (auto ep = boost::edges(g); ep.first != ep.second; ++ep.first) {
			auto vi = boost::source(*ep.first, g);
			auto vj = boost::target(*ep.first, g);

			Node& src_i = nodes[vi];
			Node& src_j = nodes[vj];

			ceres::CostFunction* cost_function = SmoothCostFunction::Create(src_i.g(), src_j.g());
			auto loss_function = new ceres::ScaledLoss(NULL, a_smooth, ceres::TAKE_OWNERSHIP);
			problem.AddResidualBlock(cost_function, loss_function, src_i.r(), src_i.t(), src_j.t());

			ceres::CostFunction* cost_function_j = SmoothCostFunction::Create(src_j.g(), src_i.g());
			auto loss_function_j = new ceres::ScaledLoss(NULL, a_smooth, ceres::TAKE_OWNERSHIP);
			problem.AddResidualBlock(cost_function_j, loss_function_j, src_j.r(), src_j.t(), src_i.t());
		}
		// rotation cost
		for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first)
		{
			Node& src_i = nodes[*vp.first];
			ceres::CostFunction* cost_function = RotationCostFunction::Create();

			auto loss_function = new ceres::ScaledLoss(new ceres::SoftLOneLoss(0.001), a_rigid, ceres::TAKE_OWNERSHIP);
			problem.AddResidualBlock(cost_function, loss_function, src_i.r());
		}
		// confident cost
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
			std::cout << "scale factor: smooth " << a_smooth << " rigid: " << a_rigid << std::endl;
		}

		_total_time_in_ms += logger.get_time_in_ms();
	}
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
	//double tol = 0.0000001;
	double tol = _options.function_tolerance;
	double error = abs(_last_cost - _current_cost);
	bool solved = error < (tol * _current_cost);
	return (_solve_iteration >= _max_iterations) || (solved && _solve_iteration > 2);
}

EmbeddedDeformation::EmbeddedDeformation(const Mesh& src,
										 const Mesh& dst,
										 ceres::Solver::Options option,
										 unsigned int number_of_deformation_nodes,
										 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)
	, _nn_search(dst)
	, _logger(logger)
{
	auto reduced_mesh = createReducedMesh(src, number_of_deformation_nodes);
	_deformation_graph = EmbeddedDeformationGraph(reduced_mesh);
	_deformed_mesh = std::make_unique<EmbeddedDeformedMesh>(src, _deformation_graph);
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
	_deformed_mesh = std::make_unique<EmbeddedDeformedMesh>(src, _deformation_graph);
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
	EmbeddedDeformedMesh deformed(_dst, inverse_deformation);
	return deformed.deformPoints();
	//return inverse_deformation.deformPoints(_dst);
}

Mesh EmbeddedDeformationWithoutICP::getDeformedPoints()
{
	return _deformed_mesh->deformPoints();
	//return _deformation_graph.deformPoints(_src);
}

std::pair<std::vector<ml::vec3f>, std::vector<ml::vec3f>> EmbeddedDeformationWithoutICP::getDeformationGraph()
{
	return _deformation_graph.getDeformationGraphEdges();
}


EmbeddedDeformationGraph & EmbeddedDeformationWithoutICP::getEmeddedDeformationGraph()
{
	return _deformation_graph;
}

bool EmbeddedDeformationWithoutICP::solveIteration()
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
										 global_node.r(), global_node.t(), src_i.r(), src_i.t(), src_i.w());
			}
		}
		// smooth cost
		for (auto ep = boost::edges(g); ep.first != ep.second; ++ep.first) {
			auto vi = boost::source(*ep.first, g);
			auto vj = boost::target(*ep.first, g);

			Node& src_i = nodes[vi];
			Node& src_j = nodes[vj];

			ceres::CostFunction* cost_function = SmoothCostFunction::Create(src_i.g(), src_j.g());
			auto loss_function = new ceres::ScaledLoss(NULL, a_smooth, ceres::TAKE_OWNERSHIP);
			problem.AddResidualBlock(cost_function, loss_function, src_i.r(), src_i.t(), src_j.t());		

			ceres::CostFunction* cost_function_j = SmoothCostFunction::Create(src_j.g(), src_i.g());
			auto loss_function_j = new ceres::ScaledLoss(NULL, a_smooth, ceres::TAKE_OWNERSHIP);
			problem.AddResidualBlock(cost_function_j, loss_function_j, src_j.r(), src_j.t(), src_i.t());
		}
		// rotation cost
		for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first)
		{
			Node& src_i = nodes[*vp.first];
			ceres::CostFunction* cost_function = RotationCostFunction::Create();
			
			auto loss_function = new ceres::ScaledLoss(new ceres::SoftLOneLoss(0.001), a_rigid, ceres::TAKE_OWNERSHIP);
			problem.AddResidualBlock(cost_function, loss_function, src_i.r());
		}
		// confident cost
		for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first)
		{
			Node& src_i = nodes[*vp.first];
			ceres::CostFunction* cost_function = ConfCostFunction::Create();
			auto loss_function = new ceres::ScaledLoss(NULL, a_conf, ceres::TAKE_OWNERSHIP);
			problem.AddResidualBlock(cost_function, loss_function, src_i.w());
		}
		ceres::CostFunction* cost_function = RotationCostFunction::Create();
		problem.AddResidualBlock(cost_function, new ceres::SoftLOneLoss(0.001), global_node.r());

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
	return finished();
}

bool EmbeddedDeformationWithoutICP::solve()
{
	while (!finished()) {
		solveIteration();
	}
	return finished();
}

bool EmbeddedDeformationWithoutICP::finished()
{
	//double tol = 0.000001;
	double tol = 0.000001;
	double error = abs(_last_cost - _current_cost);
	bool solved = error < (tol * _current_cost);
	return (_solve_iteration >= _max_iterations) || (solved && _solve_iteration > 2);
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