#include "ed.h"
#include "ed_deformation.h"
#include "ed_cost_functions.h"
#include "algo/registration/util/ceres_iteration_logger.h"
#include "algo/registration/util/ceres_residual_evaluation.h"
#include "algo/remeshing/mesh_simplification.h"

namespace Registration {

const SurfaceMesh & EmbeddedDeformation::getSource()
{
	return _source;
}

const SurfaceMesh & EmbeddedDeformation::getTarget()
{
	return _target;
}

SurfaceMesh EmbeddedDeformation::getDeformedPoints()
{
	return _deformed_mesh->deformPoints();
}

SurfaceMesh EmbeddedDeformation::getInverseDeformedPoints()
{
	auto inverse_deformation = _deformation_graph.invertDeformation();
	DeformedMesh<Deformation> deformed(_target, inverse_deformation, 4); // todo
	return deformed.deformPoints();
}

std::vector<Point> EmbeddedDeformation::getFixedPostions()
{
	std::vector<Point> positions;
	for (auto & v : _fixed_positions) {
		positions.push_back(_target.point(v));
	}
	return positions;
}



const DeformationGraph<EDDeformation> & EmbeddedDeformation::getDeformation()
{
	return _deformation_graph;
}

void EmbeddedDeformation::setDeformation(const Deformation & deformation_graph)
{
	_current_cost = 1.;
	_last_cost = 2.;
	_solve_iteration = 0;

	_deformation_graph = deformation_graph;
	_deformed_mesh = std::make_unique<DeformedMesh<Deformation>>(_source, _deformation_graph, 4);
}

SurfaceMesh EmbeddedDeformation::getDeformationGraphMesh()
{
	return deformationGraphToSurfaceMesh(_deformation_graph, _options.evaluate_residuals);
}

ceres::ResidualBlockId EmbeddedDeformation::addPointToPointCostForNode(ceres::Problem &problem, vertex_descriptor v, const Point & target_point)
{
	float point_to_point_weighting = 0.1f;
	double weight = a_fit * point_to_point_weighting;
	
	auto & global = _deformation_graph._global;
	auto n_w_vector = _deformed_mesh->nearestNodes(v).node_weight_vector;

	auto cost_function = ED::FitStarPointToPointCostFunction::Create(target_point, _deformed_mesh->point(v), global.position(),
																 _deformation_graph.getDeformation(n_w_vector[0].first).position(),
																 _deformation_graph.getDeformation(n_w_vector[1].first).position(),
																 _deformation_graph.getDeformation(n_w_vector[2].first).position(),
																 _deformation_graph.getDeformation(n_w_vector[3].first).position(),
																 n_w_vector[0].second, n_w_vector[1].second, n_w_vector[2].second, n_w_vector[3].second);
	auto loss_function = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function,
									global.d(),
									_deformation_graph.getDeformation(n_w_vector[0].first).d(),
									_deformation_graph.getDeformation(n_w_vector[1].first).d(),
									_deformation_graph.getDeformation(n_w_vector[2].first).d(),
									_deformation_graph.getDeformation(n_w_vector[3].first).d());
}

ceres::ResidualBlockId EmbeddedDeformation::addPointToPlaneCostForNode(ceres::Problem &problem, vertex_descriptor v, const Point & target_point, const Vector & target_normal)
{
	float point_to_plane_weighting = 0.9f;
	double weight = a_fit * point_to_plane_weighting;

	auto & global = _deformation_graph._global;
	auto n_w_vector = _deformed_mesh->nearestNodes(v).node_weight_vector;

	ceres::CostFunction* cost_function = ED::FitStarPointToPlaneCostFunction::Create(target_point, target_normal,
																				 _deformed_mesh->point(v), global.position(),
																				 _deformation_graph.getDeformation(n_w_vector[0].first).position(),
																				 _deformation_graph.getDeformation(n_w_vector[1].first).position(),
																				 _deformation_graph.getDeformation(n_w_vector[2].first).position(),
																				 _deformation_graph.getDeformation(n_w_vector[3].first).position(),
																				 n_w_vector[0].second, n_w_vector[1].second, n_w_vector[2].second, n_w_vector[3].second);
	auto loss_function = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function,
									global.d(),
									_deformation_graph.getDeformation(n_w_vector[0].first).d(),
									_deformation_graph.getDeformation(n_w_vector[1].first).d(),
									_deformation_graph.getDeformation(n_w_vector[2].first).d(),
									_deformation_graph.getDeformation(n_w_vector[3].first).d());
}


VertexResidualIds EmbeddedDeformation::addFitCostWithoutICP(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;
	auto & mesh = _deformation_graph._mesh;
	
	auto target_normals = _target.property_map<vertex_descriptor, Vector>("v:normal").first;
	for (auto & v : mesh.vertices())
	{
		if (_fixed_positions.empty() || (std::find(_fixed_positions.begin(), _fixed_positions.end(), v) != _fixed_positions.end()))
		{
			residual_ids[v].push_back(addPointToPointCostForNode(problem, v, _target.point(v)));
			residual_ids[v].push_back(addPointToPlaneCostForNode(problem, v, _target.point(v), target_normals[v]));
		}
	}
	return residual_ids;
}


VertexResidualIds EmbeddedDeformation::addFitCost(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;

	auto & mesh = _deformation_graph._mesh;

	auto vertex_used = mesh.property_map<vertex_descriptor, bool>("v:vertex_used").first;
	int i = 0;
	for (auto & v : mesh.vertices())
	{
		vertex_used[v] = false;
		
		auto deformed_point = _deformed_mesh->deformed_point(v);
		auto deformed_normal = _deformed_mesh->deformed_normal(v);
		auto correspondent_point = _find_correspondence_point->correspondingPoint(deformed_point, deformed_normal);
		//if (!mesh.is_border(v, true)) {

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
	auto deformations = mesh.property_map<vertex_descriptor, EDDeformation>("v:node_deformation").first;
	for (auto e : mesh.halfedges())
	{
		auto target = mesh.target(e);
		auto source = mesh.source(e);

		ceres::CostFunction* cost_function = ED::SmoothCostFunction::Create(mesh.point(source), mesh.point(target));
		auto loss_function = new ceres::ScaledLoss(NULL, a_smooth, ceres::TAKE_OWNERSHIP);
		auto residual_id = problem.AddResidualBlock(cost_function, loss_function,
													deformations[source].r(), deformations[source].t(), deformations[target].t());

		auto edge = _deformation_graph._mesh.edge(e);
		residual_ids[edge].push_back(residual_id);
	}

	return residual_ids;
}

VertexResidualIds EmbeddedDeformation::addRotationCost(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;

	auto deformations = _deformation_graph._mesh.property_map<vertex_descriptor, EDDeformation>("v:node_deformation").first;
	for (auto & v : _deformation_graph._mesh.vertices())
	{
		ceres::CostFunction* cost_function = ED::RotationCostFunction::Create();
		auto loss_function = new ceres::ScaledLoss(new ceres::SoftLOneLoss(0.001), a_rigid, ceres::TAKE_OWNERSHIP);
		residual_ids[v].push_back(problem.AddResidualBlock(cost_function, loss_function, deformations[v].r()));
	}
	return residual_ids;
}

VertexResidualIds EmbeddedDeformation::addConfCost(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;

	auto deformations = _deformation_graph._mesh.property_map<vertex_descriptor, EDDeformation>("v:node_deformation").first;
	for (auto & v : _deformation_graph._mesh.vertices())
	{
		ceres::CostFunction* cost_function = ED::ConfCostFunction::Create();
		auto loss_function = new ceres::ScaledLoss(NULL, a_conf, ceres::TAKE_OWNERSHIP);
		residual_ids[v].push_back(problem.AddResidualBlock(cost_function, loss_function, deformations[v].w()));
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
		ceres::CostFunction* cost_function = ED::RotationCostFunction::Create();
		auto loss_function = new ceres::ScaledLoss(new ceres::SoftLOneLoss(0.001), a_rigid, ceres::TAKE_OWNERSHIP);
		problem.AddResidualBlock(cost_function, loss_function, _deformation_graph._global.r());


		ceres::Solve(_ceres_option, &problem, &summary);

		if(_options.evaluate_residuals)
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
	double tol = _ceres_option.function_tolerance;
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
	_deformation_graph.setRigidDeformation(rigid_deformation);
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


EmbeddedDeformation::EmbeddedDeformation(const SurfaceMesh& source,
										 const SurfaceMesh& target,
										 std::vector<vertex_descriptor> fixed_positions,
										 const DeformationGraph<EDDeformation> & deformation_graph,
										 ceres::Solver::Options ceres_option,
										 const RegistrationOptions & options,
										 std::shared_ptr<FileWriter> logger)
	: _source(source)
	, _target(target)
	, _ceres_option(ceres_option)
	, _options(options)
	, _deformation_graph(deformation_graph)
	, _fixed_positions(fixed_positions)
	, _ceres_logger(logger)
	, _with_icp(false)
{
	setParameters();
	a_smooth = 10.;
	a_fit = 10.;
	_find_correspondence_point = std::make_unique<FindCorrespondingPoints>(_target, _find_max_distance, _find_max_angle_deviation, 10.);
	_deformed_mesh = std::make_unique<DeformedMesh<Deformation>>(_source, _deformation_graph, 4); // todo
}

EmbeddedDeformation::EmbeddedDeformation(const SurfaceMesh& source,
										 const SurfaceMesh& target,
										 ceres::Solver::Options ceres_option,
										 const RegistrationOptions & options,
										 std::shared_ptr<FileWriter> logger)
	: _source(source)
	, _target(target)
	, _ceres_option(ceres_option)
	, _options(options)
	, _ceres_logger(logger)
	, _with_icp(true)
{
	auto reduced_mesh = createReducedMesh(source, _options.dg_options.edge_length, _options.mesh_reduce_strategy);
	auto global = createGlobalDeformation<EDDeformation>(_source);
	_deformation_graph = createDeformationGraphFromMesh<EDDeformation>(reduced_mesh, global);

	setParameters();

	_find_correspondence_point = std::make_unique<FindCorrespondingPoints>(_target, _find_max_distance, _find_max_angle_deviation, 10.);
	_deformed_mesh = std::make_unique<DeformedMesh<Deformation>>(_source, _deformation_graph, 4); // todo
}


EmbeddedDeformation::EmbeddedDeformation(const SurfaceMesh& source,
										 const SurfaceMesh& target,
										 const DeformationGraph<EDDeformation> & deformation_graph,
										 ceres::Solver::Options ceres_option,
										 const RegistrationOptions & options,
										 std::shared_ptr<FileWriter> logger)
	: _source(source)
	, _target(target)
	, _ceres_option(ceres_option)
	, _options(options)
	, _deformation_graph(deformation_graph)
	, _ceres_logger(logger)
	, _with_icp(true)
{
	setParameters();
	_find_correspondence_point = std::make_unique<FindCorrespondingPoints>(_target, _find_max_distance, _find_max_angle_deviation, 10.);
	_deformed_mesh = std::make_unique<DeformedMesh<Deformation>>(_source, _deformation_graph, 4); // todo
}



//-----------------------------------------------------------------------------

EDDeformation createGlobalEDDeformationFromRigidDeformation(const RigidDeformation & rigid_deformation)
{	
	auto r = rigid_deformation.rotation();
	//double x = r.m(0,1);
	ml::mat3d rotation(r.m(0, 0),r.m(0, 1), r.m(0, 2),r.m(1, 0), r.m(1, 1), r.m(1, 2), r.m(2, 0), r.m(2, 1), r.m(2, 2));
	return EDDeformation(rigid_deformation._g, rotation, rigid_deformation._t);
}


std::unique_ptr<EmbeddedDeformation> createEmbeddedDeformation(const SurfaceMesh& src,
															   const SurfaceMesh& dst,
															   std::vector<vertex_descriptor> fixed_positions,
															   ceres::Solver::Options option,
															   const RegistrationOptions & registration_options,
															   std::shared_ptr<FileWriter> logger)
{
	auto reduced_mesh = createReducedMesh(src, registration_options.dg_options.edge_length, registration_options.mesh_reduce_strategy);
	auto global = createGlobalDeformation<EDDeformation>(reduced_mesh);
	auto deformation_graph = createDeformationGraphFromMesh<EDDeformation>(reduced_mesh, global);
	return std::make_unique<EmbeddedDeformation>(src, dst, fixed_positions, deformation_graph, option, registration_options, logger);
}


std::unique_ptr<EmbeddedDeformation> createEmbeddedDeformation(const SurfaceMesh& src,
															   const SurfaceMesh& dst,
															   ceres::Solver::Options option,
															   const RegistrationOptions & registration_options,
															   std::shared_ptr<FileWriter> logger)
{
	auto reduced_mesh = createReducedMesh(src, registration_options.dg_options.edge_length, registration_options.mesh_reduce_strategy);
	auto global = createGlobalDeformation<EDDeformation>(reduced_mesh);
	auto deformation_graph = createDeformationGraphFromMesh<EDDeformation>(reduced_mesh, global);
	return std::make_unique<EmbeddedDeformation>(src, dst, deformation_graph, option, registration_options, logger);
}


std::unique_ptr<EmbeddedDeformation> createEmbeddedDeformation(const SurfaceMesh& src,
										                       const SurfaceMesh& dst,
										                       const RigidDeformation & rigid_deformation,
										                       ceres::Solver::Options option,
										                       const RegistrationOptions & registration_options,
										                       std::shared_ptr<FileWriter> logger)
{
	auto reduced_mesh = createReducedMesh(src, registration_options.dg_options.edge_length, registration_options.mesh_reduce_strategy);
	auto global = createGlobalEDDeformationFromRigidDeformation(rigid_deformation);
	auto deformation_graph = createDeformationGraphFromMesh<EDDeformation>(reduced_mesh, global);
	return std::make_unique<EmbeddedDeformation>(src, dst, deformation_graph, option, registration_options, logger);
}


std::unique_ptr<EmbeddedDeformation> createEmbeddedDeformation(const SurfaceMesh& src,
										                       const SurfaceMesh& dst,
										                       const RigidDeformation & rigid_deformation,
										                       const DeformationGraph<EDDeformation> & deformation_graph,
										                       ceres::Solver::Options option,
										                       const RegistrationOptions & registration_options,
										                       std::shared_ptr<FileWriter> logger)
{
	auto global = createGlobalEDDeformationFromRigidDeformation(rigid_deformation);
	auto new_deformation_graph = createDeformationGraphFromMesh<EDDeformation>(deformation_graph._mesh, global);
	return std::make_unique<EmbeddedDeformation>(src, dst, new_deformation_graph, option, registration_options, logger);
}




}