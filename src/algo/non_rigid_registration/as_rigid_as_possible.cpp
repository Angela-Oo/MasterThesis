#include "non_rigid_deformation.h"
#include "as_rigid_as_possible_cost_function.h"
#include "embedded_deformation_cost_function.h"
#include "../se3.h"
#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/connected_components.hpp"
#include "../ceres_iteration_logger.h"

//
//
//std::vector<size_t> AsRigidAsPossible::getNeighborIndices(size_t i, size_t size)
//{
//	std::vector<size_t> indices;
//	int j = static_cast<int>(i) - 1;
//	if (j >= 0)
//		indices.push_back(static_cast<size_t>(j));
//	j = static_cast<int>(i) + 1;
//	if (j < size)
//		indices.push_back(static_cast<size_t>(j));
//	return indices;
//}
//
//void AsRigidAsPossible::solveRotation()
//{
//	ceres::Solver::Summary summary;
//	ceres::Problem problem;
//	for (int i = 0; i < _src.size(); ++i) {
//		auto neighbors = getNeighborIndices(i, _src.size());
//		ml::vec3d * r_i = &_rotations[i];
//		for (auto & j : neighbors) {
//			ceres::CostFunction* cost_function = AsRigidAsPossibleCostFunction::Create(_solved_points[i], _solved_points[j], _src[i], _src[j]);
//			problem.AddResidualBlock(cost_function, NULL, r_i->array);
//		}
//	}
//	ceres::Solve(_options, &problem, &summary);
//}
//
//void AsRigidAsPossible::solvePositions()
//{
//	ceres::Solver::Summary summary;
//	ceres::Problem problem;
//	for (int i = 0; i < _src.size() - 1; ++i) {
//		ml::vec3d * dst_i = &_solved_points[i];
//		auto neighbors = getNeighborIndices(i, _src.size());
//		for (auto & j : neighbors) {
//			ceres::CostFunction* cost_function = AsRigidAsPossiblePointCostFunction::Create(_src[i], _src[j], _rotations[i], _solved_points[j]);
//			problem.AddResidualBlock(cost_function, NULL, dst_i->array);
//		}
//	}
//	for (int i = 0; i < _src.size() - 1; ++i)
//	{
//		ceres::CostFunction* cost_function = FitCostFunction::Create(_src[i]);
//		problem.AddResidualBlock(cost_function, NULL, (&_solved_points[i])->array);
//	}
//	ceres::Solve(_options, &problem, &summary);
//}
//
//
//std::vector<ml::vec3f> AsRigidAsPossible::solve()
//{
//	for (int i = 0; i < _rotations.size(); ++i) {
//		_rotations[i] = ml::vec3d(0., 0., 0.);
//	}
//
//	solveRotation();
//	solvePositions();
//
//	std::vector<ml::vec3f> solved_points;
//	for (int i = 0; i < _solved_points.size(); ++i) {
//		solved_points.push_back(ml::vec3d(_solved_points[i].x, _solved_points[i].y, _solved_points[i].z));
//	}
//	return solved_points;
//}
//
//AsRigidAsPossible::AsRigidAsPossible(const std::vector<ml::vec3f>& src,
//									 const std::vector<ml::vec3f>& dst,
//									 ceres::Solver::Options option)
//	: _src(src)
//	, _dst(dst)
//	, _options(option)
//{
//	for (int i = 0; i < _src.size(); ++i) {
//		_rotations.push_back(ml::vec3d(0., 0., 0.));
//	}
//	for (int i = 0; i < _dst.size(); ++i) {
//		_solved_points.push_back(ml::vec3d(_dst[i].x, _dst[i].y, _dst[i].z));
//	}
//
//	std::cout << "\nCeres Solver" << std::endl;
//	std::cout << "Ceres preconditioner type: " << _options.preconditioner_type << std::endl;
//	std::cout << "Ceres linear algebra type: " << _options.sparse_linear_algebra_library_type << std::endl;
//	std::cout << "Ceres linear solver type: " << _options.linear_solver_type << std::endl;
//}





std::vector<ml::vec3f> AsRigidAsPossible::getDeformedPoints()
{
	return _deformation_graph.deformPoints(_src);
}

std::vector<ml::vec3f> AsRigidAsPossible::getDeformationGraph()
{
	return _deformation_graph.getDeformationGraph();
}

void AsRigidAsPossible::solveIteration()
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

			ceres::CostFunction* cost_function = FitStarWCostFunction::Create(_dst[i], src_i._g, global_node._g);
			double weight = a_fit * std::pow(src_i._w, 2);
			auto loss_function = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);
			problem.AddResidualBlock(cost_function, loss_function,
				(&global_node._r)->getData(), (&global_node._t)->getData(), (&src_i._t)->getData());
		}
		for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first) {
			Node& src_i = nodes[*vp.first];
			for (auto avp = boost::adjacent_vertices(*vp.first, g); avp.first != avp.second; ++avp.first) {
				Node& src_j = nodes[*avp.first];
				ceres::CostFunction* cost_function = AsRigidAsPossibleCostFunction::Create(src_i._g, src_j._g, src_j._t);
				auto loss_function = new ceres::ScaledLoss(NULL, a_smooth, ceres::TAKE_OWNERSHIP);
				problem.AddResidualBlock(cost_function, loss_function, (&src_i._r)->getData(), (&src_i._t)->getData());
			}
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

std::vector<ml::vec3f> AsRigidAsPossible::solve()
{
	ml::mat4f transformation = ml::mat4f::identity();
	while (!finished()) {
		solveIteration();
	}
	return getDeformedPoints();
}

bool AsRigidAsPossible::finished()
{
	double tol = 0.000001;
	return (_solve_iteration >= _max_iterations) ||
		abs(_last_cost - _current_cost) < (tol * _current_cost);
}

AsRigidAsPossible::AsRigidAsPossible(const std::vector<ml::vec3f>& src,
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