#include "embedded_deformation.h"
#include "embedded_deformation_cost_function.h"
#include "../se3.h"
#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/connected_components.hpp"
#include "../ceres_iteration_logger.h"



std::vector<ml::vec3f> EmbeddedDeformation::getDeformedPoints()
{
	return _deformation_graph.deformPoints(_src);
}

std::vector<ml::vec3f> EmbeddedDeformation::getDeformationGraph()
{
	return _deformation_graph.getDeformationGraph();
}

void EmbeddedDeformation::solveIteration()
{
	if (!finished()) {
		_solve_iteration++;

		ceres::Solver::Summary summary;
		CeresIterationLoggerGuard logger(summary, _total_time_in_ms, _solve_iteration);

		ceres::Problem problem;
		auto & g = _deformation_graph._graph;
		auto & global_node = _deformation_graph._global_rigid_deformation;


		auto & nodes = boost::get(node_t(), g);

		for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first) {
			Node& src_i = nodes[*vp.first];

			ml::vec3f pos = src_i.deformedPosition();
			ml::vec3f pos_deformed = _deformation_graph._global_rigid_deformation.deformPosition(pos);
			unsigned int i = _nn_search.nearest_index(pos_deformed);
			
			//ceres::CostFunction* cost_function = FitStarCostFunction::Create(_dst[i], src_i._g, global_node._g);
			ceres::CostFunction* cost_function = FitStarWCostFunction::Create(_dst[i], src_i._g, global_node._g);
			//double weight = a_fit * std::pow(src_i._w, 2);
			double weight = a_fit;
			auto loss_function = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);		
			//problem.AddResidualBlock(cost_function, loss_function, 
			//						(&global_node._r)->getData(), (&global_node._t)->getData(), (&src_i._t)->getData());
			problem.AddResidualBlock(cost_function, loss_function,
				(&global_node._r)->getData(), (&global_node._t)->getData(), (&src_i._t)->getData(), &src_i._w);
		}
		for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first) {
			Node& src_i = nodes[*vp.first];
			for (auto avp = boost::adjacent_vertices(*vp.first, g); avp.first != avp.second; ++avp.first) {
				Node& src_j = nodes[*avp.first];
				ceres::CostFunction* cost_function = SmoothCostFunction::Create(src_i._g, src_j._g);
				auto loss_function = new ceres::ScaledLoss(NULL, a_smooth, ceres::TAKE_OWNERSHIP);
				problem.AddResidualBlock(cost_function, loss_function, (&src_i._r)->getData(), (&src_i._t)->getData(), (&src_j._t)->getData());
			}
		}
		for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first)
		{
			Node& src_i = nodes[*vp.first];
			ceres::CostFunction* cost_function = RotationCostFunction::Create();
			auto loss_function = new ceres::ScaledLoss(NULL, a_rigid, ceres::TAKE_OWNERSHIP);
			problem.AddResidualBlock(cost_function, loss_function, (&src_i._r)->getData());
		}
		for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first)
		{
			Node& src_i = nodes[*vp.first];
			ceres::CostFunction* cost_function = ConfCostFunction::Create();
			auto loss_function = new ceres::ScaledLoss(NULL, a_conf, ceres::TAKE_OWNERSHIP);
			problem.AddResidualBlock(cost_function, loss_function, &src_i._w);
		}
		ceres::CostFunction* cost_function = RotationCostFunction::Create();
		problem.AddResidualBlock(cost_function, NULL, (&global_node._r)->getData());

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

std::vector<ml::vec3f> EmbeddedDeformation::solve()
{
	ml::mat4f transformation = ml::mat4f::identity();
	while (!finished()) {
		solveIteration();
	}
	return getDeformedPoints();
}

bool EmbeddedDeformation::finished()
{
	double tol = 0.000001;
	return (_solve_iteration >= _max_iterations) ||
		(abs(_last_cost - _current_cost) < (tol * _current_cost) && _solve_iteration > 3);
}

EmbeddedDeformation::EmbeddedDeformation(const std::vector<ml::vec3f>& src,
										 const std::vector<ml::vec3f>& dst,
										 ceres::Solver::Options option,
										 unsigned int number_of_deformation_nodes)
	: _src(src)
	, _dst(dst)
	, _options(option)
	, _deformation_graph(src, number_of_deformation_nodes)
	, _nn_search(dst)
{
	std::cout << "\nCeres Solver" << std::endl;
	std::cout << "Ceres preconditioner type: " << _options.preconditioner_type << std::endl;
	std::cout << "Ceres linear algebra type: " << _options.sparse_linear_algebra_library_type << std::endl;
	std::cout << "Ceres linear solver type: " << _options.linear_solver_type << std::endl;
}