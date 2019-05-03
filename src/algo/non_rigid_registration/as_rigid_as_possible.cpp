#include "non_rigid_deformation.h"
#include "as_rigid_as_possible_cost_function.h"
#include "non_rigid_registration_cost_function.h"
#include "algo/se3.h"
#include "algo/ceres_iteration_logger.h"
#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/connected_components.hpp"




const ml::mat3d & ARAPNode::rotation() const
{
	ml::mat3d r;
	ceres::AngleAxisToRotationMatrix(_r.array, r.getData());
	return r;
}

ml::vec3d ARAPNode::deformedPosition() const
{
	return _g + _t;
}

ml::vec3d ARAPNode::deformedNormal() const
{
	auto r_t = rotation().getTranspose();
	auto normal = r_t * _n;
	return normal.getNormalized();
}

ml::vec3d ARAPNode::deformPosition(const ml::vec3f & pos) const
{
	return (rotation()*(pos - _g)) + _g + _t;
}

ARAPNode::ARAPNode(const ml::vec3f & g, const ml::vec3d & n)
	: ARAPNode(g, n, ml::vec3d::origin, ml::vec3d::origin)
{}

ARAPNode::ARAPNode(const ml::vec3f & g, const ml::vec3d & n, const ml::vec3d & r, const ml::vec3d & t)
	: _g(g)
	, _n(n)
	, _r(r)
	, _t(t)
	, _w(1.)
{}

ARAPNode::ARAPNode()
	: ARAPNode(ml::vec3f::origin, ml::vec3f::eZ)
{}






Mesh AsRigidAsPossible::getDeformedPoints()
{
	return _deformation_graph.deformPoints(_src);
}

TemplateDeformationGraph<ARAPGraph, ARAPNode> & AsRigidAsPossible::getDeformationGraph()
{
	return _deformation_graph;
}

void AsRigidAsPossible::solveIteration()
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
}

Mesh AsRigidAsPossible::solve()
{
	ml::mat4f transformation = ml::mat4f::identity();
	while (!finished()) {
		solveIteration();
	}
	return getDeformedPoints();
}

bool AsRigidAsPossible::finished()
{
	//double tol = 0.000001;
	double tol = 0.0000001;
	return (_solve_iteration >= _max_iterations) ||
		abs(_last_cost - _current_cost) < (tol * _current_cost);
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