#include "embedded_deformation.h"
#include "embedded_deformation_cost_function.h"
#include "se3.h"
#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/connected_components.hpp"
#include "ceres_iteration_logger.h"

std::vector<size_t> EmbeddedDeformationLine::getNeighborIndices(size_t i, size_t size)
{
	std::vector<size_t> indices;
	int j = static_cast<int>(i) - 1;
	if (j >= 0)
		indices.push_back(static_cast<size_t>(j));
	j = static_cast<int>(i) + 1;
	if (j < size)
		indices.push_back(static_cast<size_t>(j));
	return indices;
}

std::vector<ml::vec3f> EmbeddedDeformationLine::solve()
{
	ceres::Solver::Summary summary;
	ceres::Problem problem;
	for (int i = 0; i < _src.size(); ++i) {
		auto neighbors = getNeighborIndices(i, _src.size());
		for (auto & j : neighbors) {
			ceres::CostFunction* cost_function = EmbeddedDeformationCostFunction::Create(_solved_points[i], _solved_points[j], _src[i], _src[j]);
			problem.AddResidualBlock(cost_function, NULL, (&_matrix[i])->getData());
		}
	}
	for (int i = 0; i < _src.size() - 1; ++i)
	{
		ceres::CostFunction* cost_function = RotationCostFunction::Create();
		problem.AddResidualBlock(cost_function, NULL, (&_matrix[i])->getData());
	}
	for (int i = 1; i < _src.size() - 1; ++i) {
		ml::vec3d * dst_i = &_solved_points[i];
		auto neighbors = getNeighborIndices(i, _src.size());
		for (auto & j : neighbors) {
			ceres::CostFunction* cost_function = EmbeddedDeformationPointsCostFunction::Create(_src[i], _src[j], _matrix[i], _solved_points[j]);
			problem.AddResidualBlock(cost_function, NULL, dst_i->array);
		}
	}
	ceres::Solve(_options, &problem, &summary);

	std::vector<ml::vec3f> solved_points;
	for (int i = 0; i < _solved_points.size(); ++i) {
		solved_points.push_back(ml::vec3d(_solved_points[i].x, _solved_points[i].y, _solved_points[i].z));
	}
	return solved_points;
}

EmbeddedDeformationLine::EmbeddedDeformationLine(const std::vector<ml::vec3f>& src,
										 const std::vector<ml::vec3f>& dst,
										 ceres::Solver::Options option)
	: _src(src)
	, _dst(dst)
	, _options(option)
{
	for (int i = 0; i < _src.size(); ++i) {
		_matrix.push_back(ml::mat3d::identity());
	}
	for (int i = 0; i < _dst.size(); ++i) {
		_solved_points.push_back(ml::vec3d(_dst[i].x, _dst[i].y, _dst[i].z));
	}

	std::cout << "\nCeres Solver" << std::endl;
	std::cout << "Ceres preconditioner type: " << _options.preconditioner_type << std::endl;
	std::cout << "Ceres linear algebra type: " << _options.sparse_linear_algebra_library_type << std::endl;
	std::cout << "Ceres linear solver type: " << _options.linear_solver_type << std::endl;
}



//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

std::vector<ml::vec3f> EmbeddedDeformation::getDeformedPoints()
{
	return _deformation_graph.deformPoints(_src);
	//return _deformation_graph.getDeformedPoints();
}

std::vector<ml::vec3f> EmbeddedDeformation::getDeformationGraph()
{
	std::vector<ml::vec3f> points;
	auto & nodes = boost::get(node_t(), _deformation_graph._graph);

	for (auto vp = boost::vertices(_deformation_graph._graph); vp.first != vp.second; ++vp.first) {
		Node& src_i = nodes[*vp.first];
		//ml::vec3f deformed = src_i.deformedPosition();
		
		ml::vec3f pos = src_i.deformedPosition();// .deformPosition(src_i._g);
		ml::vec3f global_pos = _deformation_graph._global_rigid_deformation.deformPosition(pos);
		//ml::vec3f test_pos = src_i.deformPosition(src_i._g);
		points.push_back(global_pos);
	}
	return points;
}

void EmbeddedDeformation::solveIteration()
{
	if (!finished()) {
		_solve_iteration++;

		ceres::Solver::Summary summary;
		CeresIterationLoggerGuard logger(summary, _total_time_in_ms, _solve_iteration);

		ceres::Problem problem;
		//for (int i = 0; i < _deformation_graph._graph.m_vertices.size(); ++i) {
		auto & g = _deformation_graph._graph;
		auto & global_node = _deformation_graph._global_rigid_deformation;

		double a_rigid = 1.;// 1000;
		double a_smooth = 0.1;// 100;
		double a_conf = 1.;// 100;
		double a_fit = 0.1;

		auto & nodes = boost::get(node_t(), g);

		//for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first) {
		//	Node& src_i = nodes[*vp.first];

		//	ml::vec3f pos = src_i.deformPosition(src_i._g);
		//	ml::vec3f pos_deformed = _deformation_graph._global_rigid_deformation.deformPosition(pos);
		//	unsigned int i = _nn_search.nearest_index(pos_deformed);

		//	std::vector<Node> neighbor_nodes_w;
		//	std::vector<Node*> neighbor_nodes;
		//	for (auto & n_id : src_i._nearestNeighbors) {
		//		neighbor_nodes_w.push_back(nodes[n_id]);	
		//		neighbor_nodes.push_back(&nodes[n_id]);
		//	}
		//	auto weights = _deformation_graph.weights(src_i._g, neighbor_nodes_w);

		//	
		//	ceres::CostFunction* cost_function = FitEDCostFunction::Create(_dst[i], src_i._g, global_node._g, neighbor_nodes[0]->_g, neighbor_nodes[1]->_g, neighbor_nodes[2]->_g, weights);
		//	//auto loss_function = ceres::ScaledLoss(NULL, src_i._w, ceres::TAKE_OWNERSHIP);
		//	double weight = a_fit * std::pow(src_i._w, 2);
		//	auto loss_function = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);
		//	//auto loss_function = new ceres::ScaledLoss(NULL, a_fit, ceres::TAKE_OWNERSHIP);			
		//	problem.AddResidualBlock(cost_function, loss_function, 
		//							(&global_node._r)->getData(), (&global_node._t)->getData(), 
		//							(&(neighbor_nodes[0])->_r)->getData(), (&(neighbor_nodes[0])->_t)->getData(),
		//							(&(neighbor_nodes[1])->_r)->getData(), (&(neighbor_nodes[1])->_t)->getData(),
		//							(&(neighbor_nodes[2])->_r)->getData(), (&(neighbor_nodes[2])->_t)->getData());
		//	
		//}
		for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first) {
			Node& src_i = nodes[*vp.first];

			ml::vec3f pos = src_i.deformedPosition();// .deformPosition(src_i._g);
			ml::vec3f pos_deformed = _deformation_graph._global_rigid_deformation.deformPosition(pos);
			unsigned int i = _nn_search.nearest_index(pos_deformed);
			//unsigned int i = _nn_search.nearest_index(src_i._g);
			
			ceres::CostFunction* cost_function = FitStarCostFunction::Create(_dst[i], src_i._g, global_node._g);
			double weight = a_fit * std::pow(src_i._w, 2);
			auto loss_function = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);		
			problem.AddResidualBlock(cost_function, loss_function, 
									(&global_node._r)->getData(), (&global_node._t)->getData(), (&src_i._t)->getData());			
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

		_current_tol = abs(_current_cost - summary.final_cost);
		_current_cost = summary.final_cost;

		_total_time_in_ms += logger.get_time_in_ms();
	}

	//return getDeformedPoints();
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
	double tol = 0.00001;
	return (_solve_iteration >= _max_iterations);// || _current_tol < tol);
}

EmbeddedDeformation::EmbeddedDeformation(const std::vector<ml::vec3f>& src,
										 const std::vector<ml::vec3f>& dst,
										 ceres::Solver::Options option)
	: _src(src)
	, _dst(dst)
	, _options(option)
	, _deformation_graph(src, 10)//00)
	, _nn_search(dst)
{
	std::cout << "\nCeres Solver" << std::endl;
	std::cout << "Ceres preconditioner type: " << _options.preconditioner_type << std::endl;
	std::cout << "Ceres linear algebra type: " << _options.sparse_linear_algebra_library_type << std::endl;
	std::cout << "Ceres linear solver type: " << _options.linear_solver_type << std::endl;
}