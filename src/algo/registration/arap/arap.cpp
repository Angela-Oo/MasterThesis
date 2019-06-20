#include "stdafx.h"

#include "arap.h"
#include "arap_cost_functions.h"
#include "algo/ceres_iteration_logger.h"
#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/connected_components.hpp"
#include "algo/mesh_simplification/mesh_simplification.h"

namespace ARAP
{


const SurfaceMesh & AsRigidAsPossible::getSource()
{
	return _src;
}

const SurfaceMesh & AsRigidAsPossible::getTarget()
{
	return _dst;
}

SurfaceMesh AsRigidAsPossible::getDeformedPoints()
{
	return _deformed_mesh->deformPoints();
}

SurfaceMesh AsRigidAsPossible::getInverseDeformedPoints()
{
	auto inverse_deformation = invertDeformationGraph(_deformation_graph);
	DG::DeformedMesh deformed(_dst, inverse_deformation);
	return deformed.deformPoints();
}

const DG::DeformationGraph & AsRigidAsPossible::getDeformationGraph()
{
	return _deformation_graph;
}

SurfaceMesh AsRigidAsPossible::getDeformationGraphMesh()
{
	return deformationGraphToSurfaceMesh(_deformation_graph, _evaluate_residuals);
}

std::vector<Point> AsRigidAsPossible::getFixedPostions()
{
	std::vector<Point> positions;
	for (auto & v : _fixed_positions) {
		positions.push_back(_dst.point(v));
	}
	return positions;
}

ceres::ResidualBlockId AsRigidAsPossible::addPointToPointCostForNode(ceres::Problem &problem, vertex_descriptor node, const Point & target_point)
{
	float point_to_point_weighting = 0.1f;
	double weight = a_fit * point_to_point_weighting;

	auto & global = _deformation_graph._global;
	auto n = _deformation_graph.getNode(node);

	auto cost_function = FitStarPointToPointAngleAxisCostFunction::Create(target_point, n._point, global._point);
	auto loss_function = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function, global._deformation->r(), global._deformation->t(), n._deformation->t(), n._deformation->w());
}

ceres::ResidualBlockId AsRigidAsPossible::addPointToPlaneCostForNode(ceres::Problem &problem,
																	 vertex_descriptor node,
																	 const Point & target_point, 
																	 const Vector & target_normal)
{
	float point_to_plane_weighting = 0.9f;
	double weight = a_fit * point_to_plane_weighting;
	auto & g = _deformation_graph._mesh;
	auto & global = _deformation_graph._global;
	auto n = _deformation_graph.getNode(node);

	auto cost_function = FitStarPointToPlaneAngleAxisCostFunction::Create(target_point, target_normal, n._point, global._point);
	auto loss_function = new ceres::ScaledLoss(NULL, point_to_plane_weighting * weight, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function,
									global._deformation->r(), global._deformation->t(), n._deformation->r(), n._deformation->t(), n._deformation->w());
}

VertexResidualIds AsRigidAsPossible::addFitCostWithoutICP(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;

	auto & mesh = _deformation_graph._mesh;
	auto deformations = mesh.property_map<vertex_descriptor, std::shared_ptr<IDeformation>>("v:node").first;

	auto target_normals = _dst.property_map<vertex_descriptor, Direction>("v:normal").first;
	for (auto & v : mesh.vertices())
	{
		auto vertex = _deformation_graph.deformNode(v);
		if (_fixed_positions.empty() || (std::find(_fixed_positions.begin(), _fixed_positions.end(), v) != _fixed_positions.end()))
		{
			residual_ids[v].push_back(addPointToPointCostForNode(problem, v, _dst.point(v)));
			residual_ids[v].push_back(addPointToPlaneCostForNode(problem, v, _dst.point(v), target_normals[v].vector()));
		}
	}
	//	std::cout << "used nodes " << i << " / " << mesh.number_of_vertices();
	return residual_ids;
}


VertexResidualIds AsRigidAsPossible::addFitCost(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;

	auto & mesh = _deformation_graph._mesh;
	auto deformations = mesh.property_map<vertex_descriptor, std::shared_ptr<IDeformation>>("v:node").first;

	auto vertex_used = mesh.property_map<vertex_descriptor, bool>("v:vertex_used").first;
	int i = 0;
	for (auto & v : mesh.vertices())
	{
		vertex_used[v] = false;
		
		bool use_vertex = true;
		if (_ignore_deformation_graph_border_vertices)
			use_vertex = !mesh.is_border(v, true);

		if(use_vertex) {
			auto vertex = _deformation_graph.deformNode(v);
			auto correspondent_point = _find_correspondence_point->correspondingPoint(vertex._point, vertex._normal.vector());

			if (correspondent_point.first) {		
				vertex_descriptor target_vertex = correspondent_point.second;
				auto target_point = _find_correspondence_point->getPoint(target_vertex);
				auto target_normal = _find_correspondence_point->getNormal(target_vertex);
				residual_ids[v].push_back(addPointToPointCostForNode(problem, v, target_point));
				residual_ids[v].push_back(addPointToPlaneCostForNode(problem, v, target_point, target_normal.vector()));

				i++;
				vertex_used[v] = true;
			}
		}
	}
	
	std::cout << "used nodes " << i << " / " << mesh.number_of_vertices() << " ";
	std::cout << " allowed distance " <<  _find_correspondence_point->median() << " ";
	return residual_ids;
}

EdgeResidualIds AsRigidAsPossible::addAsRigidAsPossibleCost(ceres::Problem &problem)
{
	EdgeResidualIds residual_ids;
	auto & mesh = _deformation_graph._mesh;
	auto deformations = mesh.property_map<vertex_descriptor, std::shared_ptr<IDeformation>>("v:node").first;
	for (auto e : mesh.halfedges())
	{		
		auto target = mesh.target(e);
		auto source = mesh.source(e);

		ceres::CostFunction* cost_function = AsRigidAsPossibleCostFunction::Create(mesh.point(source), mesh.point(target));
		auto loss_function = new ceres::ScaledLoss(NULL, a_smooth, ceres::TAKE_OWNERSHIP);
		auto residual_id = problem.AddResidualBlock(cost_function, loss_function, 
													deformations[source]->r(), deformations[source]->t(), deformations[target]->t());

		auto edge = _deformation_graph._mesh.edge(e);
		residual_ids[edge].push_back(residual_id);
	}

	return residual_ids;
}

VertexResidualIds AsRigidAsPossible::addConfCost(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;
	auto deformations = _deformation_graph._mesh.property_map<vertex_descriptor, std::shared_ptr<IDeformation>>("v:node").first;
	for(auto & v : _deformation_graph._mesh.vertices())
	{
		auto& deformation = deformations[v];
		ceres::CostFunction* cost_function = ConfCostFunction::Create();
		auto loss_function = new ceres::ScaledLoss(NULL, a_conf, ceres::TAKE_OWNERSHIP);
		residual_ids[v].push_back(problem.AddResidualBlock(cost_function, loss_function, deformation->w()));
	}
	return residual_ids;
}

bool AsRigidAsPossible::solveIteration()
{
	if (!finished()) {
		std::cout << std::endl;
		_solve_iteration++;
		ceres::Solver::Summary summary;
		CeresIterationLoggerGuard logger(summary, _total_time_in_ms, _solve_iteration, _logger);

		ceres::Problem problem;

		VertexResidualIds fit_residual_ids;
		if (_with_icp)
			fit_residual_ids = addFitCost(problem);
		else
			fit_residual_ids = addFitCostWithoutICP(problem);
		EdgeResidualIds arap_residual_ids = addAsRigidAsPossibleCost(problem);
		VertexResidualIds conf_residual_ids = addConfCost(problem);

		ceres::Solve(_options, &problem, &summary);

		// evaluate
		if (_evaluate_residuals) {
			evaluateResidual(problem, fit_residual_ids, arap_residual_ids, conf_residual_ids);
		}

		_last_cost = _current_cost;
		_current_cost = summary.final_cost;

		auto scale_factor_tol = 0.0001;// 0.00001;
		if (abs(_current_cost - _last_cost) < scale_factor_tol *(1 + _current_cost) &&
			(a_smooth > 0.05 && a_conf > 0.05))
		{
			a_smooth /= 2.;
			a_conf /= 2.;
			std::cout << std::endl << "scale factor: smooth " << a_smooth << " conf " << a_conf << std::endl;
		}

		_total_time_in_ms += logger.get_time_in_ms();
	}
	return finished();
}

size_t AsRigidAsPossible::currentIteration()
{
	return _solve_iteration;
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
										 std::map<vertex_descriptor, ResidualIds> & fit_residual_block_ids,
										 std::map<edge_descriptor, ResidualIds> & arap_residual_block_ids,
										 std::map<vertex_descriptor, ResidualIds> & conf_residual_block_ids)
{
	auto fit_cost = _deformation_graph._mesh.property_map<vertex_descriptor, double>("v:fit_cost").first;
	for (auto & r : fit_residual_block_ids) {
		fit_cost[r.first] = evaluateResidual(problem, r.second) * a_fit;
	}
	auto smooth_cost = _deformation_graph._mesh.property_map<edge_descriptor, double>("e:smooth_cost").first;
	for (auto & r : arap_residual_block_ids) {
		smooth_cost[r.first] = evaluateResidual(problem, r.second) * a_smooth;
	}
	auto conf_cost = _deformation_graph._mesh.property_map<vertex_descriptor, double>("v:conf_cost").first;
	for (auto & r : conf_residual_block_ids) {
		conf_cost[r.first] = evaluateResidual(problem, r.second) * a_conf;
	}
}


void AsRigidAsPossible::printCeresOptions()
{
	std::cout << "\nCeres Solver" << std::endl;
	std::cout << "Ceres preconditioner type: " << _options.preconditioner_type << std::endl;
	std::cout << "Ceres linear algebra type: " << _options.sparse_linear_algebra_library_type << std::endl;
	std::cout << "Ceres linear solver type: " << _options.linear_solver_type << std::endl;
}

void AsRigidAsPossible::setParameters()
{
	a_smooth = 5.;//  0.1;// 100;
	a_conf = 100.;// 1.;// 100;
	a_fit = 20.;
	_find_max_distance = 0.1;
	_find_max_angle_deviation = 45.;
}

AsRigidAsPossible::AsRigidAsPossible(const SurfaceMesh& src,
									 const SurfaceMesh& dst,
									 std::vector<vertex_descriptor> fixed_positions,
									 ceres::Solver::Options option,
									 bool evaluate_residuals,
									 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)
	, _deformation_graph(src, []() { return std::make_shared<Deformation>(); })
	, _fixed_positions(fixed_positions)
	, _logger(logger)
	, _evaluate_residuals(evaluate_residuals)
	, _with_icp(false)
	, _ignore_deformation_graph_border_vertices(false)
{
	setParameters();
	a_smooth = 10.;
	a_fit = 100.;
	_find_correspondence_point = std::make_unique<FindCorrespondingPoints>(dst, _find_max_distance, _find_max_angle_deviation);
	_deformed_mesh = std::make_unique<DG::DeformedMesh>(src, _deformation_graph);
	printCeresOptions();
}


AsRigidAsPossible::AsRigidAsPossible(const SurfaceMesh& src,
									 const SurfaceMesh& dst,
									 ceres::Solver::Options option,
									 double deformation_graph_edge_length,
									 bool evaluate_residuals,
									 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)
	, _logger(logger)
	, _evaluate_residuals(evaluate_residuals)
	, _ignore_deformation_graph_border_vertices(false)
{
	setParameters();
	_find_correspondence_point = std::make_unique<FindCorrespondingPoints>(dst, _find_max_distance, _find_max_angle_deviation);
	auto reduced_mesh = createReducedMesh(src, deformation_graph_edge_length);
	std::cout << "number of nodes " << reduced_mesh.num_vertices() << std::endl;
	_deformation_graph = DG::DeformationGraph(reduced_mesh, []() { return std::make_shared<Deformation>(); });
	_deformed_mesh = std::make_unique<DG::DeformedMesh>(src, _deformation_graph);
	printCeresOptions();
}

AsRigidAsPossible::AsRigidAsPossible(const SurfaceMesh& src,
									 const SurfaceMesh& dst,
									 const DG::DeformationGraph & deformation_graph,
									 ceres::Solver::Options option,
									 bool evaluate_residuals,
									 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)
	, _deformation_graph(deformation_graph)
	, _logger(logger)
	, _evaluate_residuals(evaluate_residuals)
	, _ignore_deformation_graph_border_vertices(false)
{
	setParameters();
	_find_correspondence_point = std::make_unique<FindCorrespondingPoints>(dst, _find_max_distance, _find_max_angle_deviation);

	//_deformation_graph = transformDeformationGraph(deformation_graph);
	_deformed_mesh = std::make_unique<DG::DeformedMesh>(src, _deformation_graph);
	printCeresOptions();
}



}