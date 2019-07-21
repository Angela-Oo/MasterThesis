#include "stdafx.h"

#include "ed.h"
#include "ed_cost_functions.h"
#include "algo/ceres_iteration_logger.h"
#include "algo/mesh_simplification/mesh_simplification.h"
#include "ed_deformation.h"
#include "algo/registration/ceres_residual_evaluation.h"

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

SurfaceMesh EmbeddedDeformation::getInverseDeformedPoints()
{
	auto inverse_deformation = invertDeformationGraph(_deformation_graph);
	DG::DeformedMesh deformed(_dst, inverse_deformation, 4); // todo
	return deformed.deformPoints();
}

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
	return deformationGraphToSurfaceMesh(_deformation_graph, _evaluate_residuals);
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
	
	auto target_normals = _dst.property_map<vertex_descriptor, Vector>("v:normal").first;
	for (auto & v : mesh.vertices())
	{
		if (_fixed_positions.empty() || (std::find(_fixed_positions.begin(), _fixed_positions.end(), v) != _fixed_positions.end()))
		{
			residual_ids[v].push_back(addPointToPointCostForNode(problem, v, _dst.point(v)));
			residual_ids[v].push_back(addPointToPlaneCostForNode(problem, v, _dst.point(v), target_normals[v]));
		}
	}
	return residual_ids;
}


VertexResidualIds EmbeddedDeformation::addFitCost(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;

	auto & mesh = _deformation_graph._mesh;
	auto deformations = mesh.property_map<vertex_descriptor, std::shared_ptr<IDeformation>>("v:node").first;

	auto vertex_used = mesh.property_map<vertex_descriptor, bool>("v:vertex_used").first;
	int i = 0;
	for (auto & v : mesh.vertices())
	{
		vertex_used[v] = false;
		
		//if (!mesh.is_border(v, true)) {
		auto vertex = _deformation_graph.deformNode(v);
		auto correspondent_point = _find_correspondence_point->correspondingPoint(vertex._point, vertex._normal);

		if (correspondent_point.first) {				
			vertex_descriptor target_vertex = correspondent_point.second;
			auto target_point = _find_correspondence_point->getPoint(target_vertex);
			auto target_normal = _find_correspondence_point->getNormal(target_vertex);
			residual_ids[v].push_back(addPointToPointCostForNode(problem, v, target_point));
			residual_ids[v].push_back(addPointToPlaneCostForNode(problem, v, target_point, target_normal));

			i++;
			vertex_used[v] = true;
		}
		//}
	}
	std::cout << "used nodes " << i << " / " << mesh.number_of_vertices();
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

		auto logger = _ceres_logger.CreateCeresIterationLogger(summary);
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

		if(_evaluate_residuals)
			evaluateResidual(problem, fit_residual_ids, smooth_residual_ids, rotation_residual_ids, conf_residual_ids);

		_last_cost = _current_cost;
		_current_cost = summary.final_cost;

		auto scale_factor_tol = 0.0001;// 0.00001;
		if (abs(_current_cost - _last_cost) < scale_factor_tol *(1 + _current_cost) &&
			(a_rigid > 1 || a_smooth > 0.1 || a_conf > 1.))
		{
			a_rigid /= 2.;
			a_smooth /= 2.;
			a_conf /= 2.;
			std::cout << "scale factor: smooth " << a_smooth << " rigid: " << a_rigid << std::endl;
		}

	}
	return finished();
}

size_t EmbeddedDeformation::currentIteration()
{
	return _solve_iteration;
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

void EmbeddedDeformation::evaluateResidual(ceres::Problem & problem,
										   VertexResidualIds & fit_residual_block_ids,
										   EdgeResidualIds & smooth_residual_block_ids,
										   VertexResidualIds & rotation_residual_block_ids,
										   VertexResidualIds & conf_residual_block_ids)
{
	// smooth
	auto smooth_cost = _deformation_graph._mesh.property_map<edge_descriptor, double>("e:smooth_cost");
	if (smooth_cost.second)
		evaluateResiduals(_deformation_graph._mesh, problem, smooth_residual_block_ids, smooth_cost.first, a_smooth);

	// fit
	auto fit_cost = _deformation_graph._mesh.property_map<vertex_descriptor, double>("v:fit_cost");
	if (fit_cost.second)
		evaluateResiduals(_deformation_graph._mesh, problem, fit_residual_block_ids, fit_cost.first, a_fit);

	// conf
	auto conf_cost = _deformation_graph._mesh.property_map<vertex_descriptor, double>("v:conf_cost");
	if (conf_cost.second)
		evaluateResiduals(_deformation_graph._mesh, problem, conf_residual_block_ids, conf_cost.first, a_conf);
}


void EmbeddedDeformation::setRigidDeformation(const RigidDeformation & rigid_deformation)
{
	_deformation_graph.setRigidDeformation(createGlobalDeformationFromRigidDeformation(rigid_deformation));
}


bool EmbeddedDeformation::shouldBeSavedAsImage()
{
	return finished();
}

void EmbeddedDeformation::setParameters()
{
	a_rigid = 100.;// 1.;// 1000;
	a_smooth = 5.;// 0.1;// 100;
	a_conf = 10.;// 1.;// 100;
	a_fit = 10.;
	_find_max_distance = 0.1;
	_find_max_angle_deviation = 45.;
}


EmbeddedDeformation::EmbeddedDeformation(const SurfaceMesh& src,
										 const SurfaceMesh& dst,
										 std::vector<vertex_descriptor> fixed_positions,
										 const DG::DeformationGraph & deformation_graph,
										 ceres::Solver::Options option,
										 bool evaluate_residuals,
										 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)
	, _deformation_graph(deformation_graph)
	, _fixed_positions(fixed_positions)
	, _evaluate_residuals(evaluate_residuals)
	, _ceres_logger(logger)
	, _with_icp(false)
{
	setParameters();
	a_smooth = 10.;
	a_fit = 10.;
	_find_correspondence_point = std::make_unique<FindCorrespondingPoints>(dst, _find_max_distance, _find_max_angle_deviation, 10.);
	_deformed_mesh = std::make_unique<DG::DeformedMesh>(src, _deformation_graph, 4); // todo
}



EmbeddedDeformation::EmbeddedDeformation(const SurfaceMesh& src,
										 const SurfaceMesh& dst,
										 const DG::DeformationGraph & deformation_graph,
										 ceres::Solver::Options option,
										 bool evaluate_residuals,
										 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)
	, _deformation_graph(deformation_graph)
	, _evaluate_residuals(evaluate_residuals)
	, _ceres_logger(logger)
	, _with_icp(true)
{
	setParameters();
	_find_correspondence_point = std::make_unique<FindCorrespondingPoints>(dst, _find_max_distance, _find_max_angle_deviation, 10.);
	_deformed_mesh = std::make_unique<DG::DeformedMesh>(src, _deformation_graph, 4); // todo
}



//-----------------------------------------------------------------------------

DG::PositionAndDeformation createGlobalDeformationFromRigidDeformation(const RigidDeformation & rigid_deformation)
{
	DG::PositionAndDeformation global;
	global._point = rigid_deformation._g;
	global._normal = Vector(0., 0., 1.);
	
	auto r = rigid_deformation.rotation();

	//double x = r.m(0,1);
	ml::mat3d rotation(r.m(0, 0),r.m(0, 1), r.m(0, 2),r.m(1, 0), r.m(1, 1), r.m(1, 2), r.m(2, 0), r.m(2, 1), r.m(2, 2));
	global._deformation = std::make_shared<Deformation>(rotation, rigid_deformation._t);

	return global;
}


std::unique_ptr<EmbeddedDeformation> createEmbeddedDeformation(const SurfaceMesh& src,
															   const SurfaceMesh& dst,
															   std::vector<vertex_descriptor> fixed_positions,
															   ceres::Solver::Options option,
															   const RegistrationOptions & registration_options,
															   std::shared_ptr<FileWriter> logger)
{
	auto reduced_mesh = createReducedMesh(src, registration_options.dg_options.edge_length, registration_options.mesh_reduce_strategy);
	auto global = DG::createGlobalDeformation(reduced_mesh, createDeformation);
	auto deformation_graph = DG::createDeformationGraphFromMesh(reduced_mesh, global, createDeformation);
	return std::make_unique<EmbeddedDeformation>(src, dst, fixed_positions, deformation_graph, option, registration_options.evaluate_residuals, logger);
}


std::unique_ptr<EmbeddedDeformation> createEmbeddedDeformation(const SurfaceMesh& src,
															   const SurfaceMesh& dst,
															   ceres::Solver::Options option,
															   const RegistrationOptions & registration_options,
															   std::shared_ptr<FileWriter> logger)
{
	auto reduced_mesh = createReducedMesh(src, registration_options.dg_options.edge_length, registration_options.mesh_reduce_strategy);
	auto global = DG::createGlobalDeformation(reduced_mesh, createDeformation);
	auto deformation_graph = DG::createDeformationGraphFromMesh(reduced_mesh, global, createDeformation);
	return std::make_unique<EmbeddedDeformation>(src, dst, deformation_graph, option, registration_options.evaluate_residuals, logger);
}


std::unique_ptr<EmbeddedDeformation> createEmbeddedDeformation(const SurfaceMesh& src,
										                       const SurfaceMesh& dst,
										                       const RigidDeformation & rigid_deformation,
										                       ceres::Solver::Options option,
										                       const RegistrationOptions & registration_options,
										                       std::shared_ptr<FileWriter> logger)
{
	auto reduced_mesh = createReducedMesh(src, registration_options.dg_options.edge_length, registration_options.mesh_reduce_strategy);
	auto global = createGlobalDeformationFromRigidDeformation(rigid_deformation);
	auto deformation_graph = DG::createDeformationGraphFromMesh(reduced_mesh, global, createDeformation);
	return std::make_unique<EmbeddedDeformation>(src, dst, deformation_graph, option, registration_options.evaluate_residuals, logger);
}


std::unique_ptr<EmbeddedDeformation> createEmbeddedDeformation(const SurfaceMesh& src,
										                       const SurfaceMesh& dst,
										                       const RigidDeformation & rigid_deformation,
										                       const DG::DeformationGraph & deformation_graph,
										                       ceres::Solver::Options option,
										                       const RegistrationOptions & registration_options,
										                       std::shared_ptr<FileWriter> logger)
{
	auto global = createGlobalDeformationFromRigidDeformation(rigid_deformation);
	auto new_deformation_graph = DG::createDeformationGraphFromMesh(deformation_graph._mesh, global, deformation_graph._create_node);
	return std::make_unique<EmbeddedDeformation>(src, dst, new_deformation_graph, option, registration_options.evaluate_residuals, logger);
}



}