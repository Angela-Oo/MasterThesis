#include "stdafx.h"

#include "ed.h"
#include "ed_cost_functions.h"
#include "algo/ceres_iteration_logger.h"
#include "algo/mesh_simplification/mesh_simplification.h"
#include "ed_deformation.h"

namespace ED {

const SurfaceMesh & EmbeddedDeformation::getSource()
{
	return _src;
}

const SurfaceMesh & EmbeddedDeformation::getTarget()
{
	return _dst;
}

SurfaceMesh EmbeddedDeformation::getDeformedPoints()
{
	return _deformed_mesh->deformPoints();
}

//Mesh EmbeddedDeformation::getInverseDeformedPoints()
//{
//	auto inverse_deformation = inverteDeformationGraph(_deformation_graph);
//	EmbeddedDeformedMesh deformed(_dst, inverse_deformation);
//	return deformed.deformPoints();
//}

std::vector<Point> EmbeddedDeformation::getFixedPostions()
{
	std::vector<Point> positions;
	for (auto & v : _fixed_positions) {
		positions.push_back(_dst.point(v));
	}
	return positions;
}

const DG::DeformationGraph & EmbeddedDeformation::getDeformationGraph()
{
	return _deformation_graph;
}

SurfaceMesh EmbeddedDeformation::getDeformationGraphMesh()
{
	return deformationGraphToSurfaceMesh(_deformation_graph);
}

ceres::ResidualBlockId EmbeddedDeformation::addPointToPointCostForNode(ceres::Problem &problem, vertex_descriptor node, const Point & target_point)
{
	float point_to_point_weighting = 0.1f;
	double weight = a_fit * point_to_point_weighting;
	
	auto & global = _deformation_graph._global;
	auto n = _deformation_graph.getNode(node);

	auto cost_function = FitStarPointToPointCostFunction::Create(target_point, n._point, global._point);
	auto loss_function = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function,
									global._deformation->r(), global._deformation->t(), n._deformation->t(), n._deformation->w());
}

ceres::ResidualBlockId EmbeddedDeformation::addPointToPlaneCostForNode(ceres::Problem &problem, vertex_descriptor node, const Point & target_point, const Vector & target_normal)
{
	float point_to_plane_weighting = 0.9f;
	double weight = a_fit * point_to_plane_weighting;

	auto & global = _deformation_graph._global;
	auto n = _deformation_graph.getNode(node);

	ceres::CostFunction* cost_function = FitStarPointToPlaneCostFunction::Create(target_point, target_normal, n._point, global._point);
	auto loss_function = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function, global._deformation->r(), global._deformation->t(), n._deformation->r(), n._deformation->t(), n._deformation->w());
}


VertexResidualIds EmbeddedDeformation::addFitCostWithoutICP(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;

	auto & mesh = _deformation_graph._mesh;
	auto deformations = mesh.property_map<vertex_descriptor, std::shared_ptr<IDeformation>>("v:node").first;
	
	auto target_normals = _dst.property_map<vertex_descriptor, Direction>("v:normal").first;
	for (auto & v : mesh.vertices())
	{
		if (_fixed_positions.empty() || (std::find(_fixed_positions.begin(), _fixed_positions.end(), v) != _fixed_positions.end()))
		{
			residual_ids[v].push_back(addPointToPointCostForNode(problem, v, _dst.point(v)));
			residual_ids[v].push_back(addPointToPlaneCostForNode(problem, v, _dst.point(v), target_normals[v].vector()));
		}
	}
	return residual_ids;
}


VertexResidualIds EmbeddedDeformation::addFitCost(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;

	auto & mesh = _deformation_graph._mesh;
	auto deformations = mesh.property_map<vertex_descriptor, std::shared_ptr<IDeformation>>("v:node").first;
	int i = 0;
	for (auto & v : mesh.vertices())
	{
		auto vertex = _deformation_graph.deformNode(v);
		auto correspondent_point = _find_correspondence_point->correspondingPoint(vertex._point, vertex._normal.vector());

		if (correspondent_point.first) {
			//node._found_nearest_point = true;
			vertex_descriptor target_vertex = correspondent_point.second;
			auto target_point = _find_correspondence_point->getPoint(target_vertex);
			auto target_normal = _find_correspondence_point->getNormal(target_vertex);
			residual_ids[v].push_back(addPointToPointCostForNode(problem, v, target_point));
			residual_ids[v].push_back(addPointToPlaneCostForNode(problem, v, target_point, target_normal.vector()));
			i++;
		}
		else {
			//node._found_nearest_point = false;
		}
	}
	std::cout << "used " << i << " of " << mesh.number_of_vertices() << " deformation graph nodes" << std::endl;
	return residual_ids;
}

EdgeResidualIds EmbeddedDeformation::addSmoothCost(ceres::Problem &problem)
{
	EdgeResidualIds residual_ids;
	auto & mesh = _deformation_graph._mesh;
	auto deformations = mesh.property_map<vertex_descriptor, std::shared_ptr<IDeformation>>("v:node").first;
	for (auto e : mesh.halfedges())
	{
		auto target = mesh.target(e);
		auto source = mesh.source(e);

		ceres::CostFunction* cost_function = SmoothCostFunction::Create(mesh.point(source), mesh.point(target));
		auto loss_function = new ceres::ScaledLoss(NULL, a_smooth, ceres::TAKE_OWNERSHIP);
		auto residual_id = problem.AddResidualBlock(cost_function, loss_function,
													deformations[source]->r(), deformations[source]->t(), deformations[target]->t());

		auto edge = _deformation_graph._mesh.edge(e);
		residual_ids[edge].push_back(residual_id);
	}

	return residual_ids;
}

VertexResidualIds EmbeddedDeformation::addRotationCost(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;

	auto deformations = _deformation_graph._mesh.property_map<vertex_descriptor, std::shared_ptr<IDeformation>>("v:node").first;
	for (auto & v : _deformation_graph._mesh.vertices())
	{
		ceres::CostFunction* cost_function = RotationCostFunction::Create();
		auto loss_function = new ceres::ScaledLoss(new ceres::SoftLOneLoss(0.001), a_rigid, ceres::TAKE_OWNERSHIP);
		residual_ids[v].push_back(problem.AddResidualBlock(cost_function, loss_function, deformations[v]->r()));
	}
	return residual_ids;
}

VertexResidualIds EmbeddedDeformation::addConfCost(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;

	auto deformations = _deformation_graph._mesh.property_map<vertex_descriptor, std::shared_ptr<IDeformation>>("v:node").first;
	for (auto & v : _deformation_graph._mesh.vertices())
	{
		ceres::CostFunction* cost_function = ConfCostFunction::Create();
		auto loss_function = new ceres::ScaledLoss(NULL, a_conf, ceres::TAKE_OWNERSHIP);
		residual_ids[v].push_back(problem.AddResidualBlock(cost_function, loss_function, deformations[v]->w()));
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
		ceres::CostFunction* cost_function = RotationCostFunction::Create();
		auto loss_function = new ceres::ScaledLoss(new ceres::SoftLOneLoss(0.001), a_rigid, ceres::TAKE_OWNERSHIP);
		problem.AddResidualBlock(cost_function, loss_function, _deformation_graph._global._deformation->r());


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
	auto fit_cost = _deformation_graph._mesh.property_map<vertex_descriptor, double>("v:fit_cost").first;
	for (auto & r : fit_residual_block_ids) {
		fit_cost[r.first] = evaluateResidual(problem, r.second);
	}
	auto smooth_cost = _deformation_graph._mesh.property_map<edge_descriptor, double>("e:smooth_cost").first;
	for (auto & r : smooth_residual_block_ids) {
		smooth_cost[r.first] = evaluateResidual(problem, r.second);
	}
	auto conf_cost = _deformation_graph._mesh.property_map<vertex_descriptor, double>("v:conf_cost").first;
	for (auto & r : conf_residual_block_ids) {
		conf_cost[r.first] = evaluateResidual(problem, r.second);
	}

	//for (auto & r : rotation_residual_block_ids) {
	//	nodes[r.first]._rotation_cost = evaluateResidual(problem, r.second);
	//}
}

void EmbeddedDeformation::printCeresOptions()
{
	std::cout << "\nCeres Solver" << std::endl;
	std::cout << "Ceres preconditioner type: " << _options.preconditioner_type << std::endl;
	std::cout << "Ceres linear algebra type: " << _options.sparse_linear_algebra_library_type << std::endl;
	std::cout << "Ceres linear solver type: " << _options.linear_solver_type << std::endl;
}

EmbeddedDeformation::EmbeddedDeformation(const SurfaceMesh& src,
										 const SurfaceMesh& dst,
										 std::vector<vertex_descriptor> fixed_positions,
										 ceres::Solver::Options option,
										 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)
	, _deformation_graph(src, []() { return std::make_shared<Deformation>(); })
	, _fixed_positions(fixed_positions)
	, _logger(logger)
	, _with_icp(false)
	, a_smooth(10.)
	, a_rigid(1000.)
	, a_fit(10.)
{
	_find_correspondence_point = std::make_unique<FindCorrespondingPoints>(dst, _find_max_distance, _find_max_angle_deviation);
	_deformed_mesh = std::make_unique<DG::DeformedMesh>(src, _deformation_graph);
	printCeresOptions();
}



EmbeddedDeformation::EmbeddedDeformation(const SurfaceMesh& src,
										 const SurfaceMesh& dst,
										 ceres::Solver::Options option,
										 unsigned int number_of_deformation_nodes,
										 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)
	, _logger(logger)
	, _with_icp(true)
{
	_find_correspondence_point = std::make_unique<FindCorrespondingPoints>(dst, _find_max_distance, _find_max_angle_deviation);
	auto reduced_mesh = createReducedMesh(src, number_of_deformation_nodes);
	std::cout << "number of def nodes " << number_of_deformation_nodes << " true number " << reduced_mesh.num_vertices() << std::endl;
	_deformation_graph = DG::DeformationGraph(reduced_mesh, []() { return std::make_shared<Deformation>(); });
	_deformed_mesh = std::make_unique<DG::DeformedMesh>(src, _deformation_graph);
	printCeresOptions();
}

EmbeddedDeformation::EmbeddedDeformation(const SurfaceMesh& src,
										 const SurfaceMesh& dst,
										 const DG::DeformationGraph & deformation_graph,
										 ceres::Solver::Options option,
										 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)
	, _deformation_graph(deformation_graph)
	, _logger(logger)
	, _with_icp(true)
{
	_find_correspondence_point = std::make_unique<FindCorrespondingPoints>(dst, _find_max_distance, _find_max_angle_deviation);
	_deformed_mesh = std::make_unique<DG::DeformedMesh>(src, _deformation_graph);
	printCeresOptions();
}



}