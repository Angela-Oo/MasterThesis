#include "arap.h"
#include "arap_cost_functions.h"

#include "algo/registration/util/ceres_iteration_logger.h"
#include "algo/remeshing/mesh_simplification.h"
#include "algo/registration/util/ceres_residual_evaluation.h"

#include "arap_fit_cost.h"
#include "arap_fit_cost_without_icp.h"
#include <random>


namespace Registration {


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
	auto inverse_deformation = _deformation_graph.invertDeformation();
	DeformedMesh<Deformation> deformed(_dst, inverse_deformation, _registration_options.dg_options.number_of_interpolation_neighbors);
	return deformed.deformPoints();
}

const DeformationGraph<ARAPDeformation> & AsRigidAsPossible::getDeformation()
{
	return _deformation_graph;
}

SurfaceMesh AsRigidAsPossible::getDeformationGraphMesh()
{
	return deformationGraphToSurfaceMesh(_deformation_graph, _registration_options.evaluate_residuals);
}

std::vector<Point> AsRigidAsPossible::getFixedPostions()
{
	return _fit_cost->getFixedPostions();
}

EdgeResidualIds AsRigidAsPossible::addAsRigidAsPossibleCost(ceres::Problem &problem)
{
	EdgeResidualIds residual_ids;
	auto & mesh = _deformation_graph._mesh;
	auto deformations = mesh.property_map<vertex_descriptor, ARAPDeformation>("v:node_deformation").first;
	auto & edge_rigidity = mesh.property_map<edge_descriptor, double>("e:rigidity");
	for (auto e : mesh.halfedges())
	{		
		auto target = mesh.target(e);
		auto source = mesh.source(e);

		double smooth = _registration_options.smooth;
		if (edge_rigidity.second) {
			smooth = edge_rigidity.first[mesh.edge(e)];
		}

		ceres::CostFunction* cost_function = AsRigidAsPossibleCostFunction::Create(mesh.point(source), mesh.point(target));
		auto loss_function = new ceres::ScaledLoss(NULL, smooth, ceres::TAKE_OWNERSHIP);
		auto residual_id = problem.AddResidualBlock(cost_function, loss_function, 
													deformations[source].d(), deformations[target].d());

		auto edge = _deformation_graph._mesh.edge(e);
		residual_ids[edge].push_back(residual_id);
	}

	return residual_ids;
}

VertexResidualIds AsRigidAsPossible::addConfCost(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;
	auto deformations = _deformation_graph._mesh.property_map<vertex_descriptor, ARAPDeformation>("v:node_deformation").first;
	for(auto & v : _deformation_graph._mesh.vertices())
	{
		ceres::CostFunction* cost_function = ConfCostFunction::Create();
		auto loss_function = new ceres::ScaledLoss(NULL, _registration_options.conf, ceres::TAKE_OWNERSHIP);
		residual_ids[v].push_back(problem.AddResidualBlock(cost_function, loss_function, deformations[v].w()));
	}
	return residual_ids;
}

bool AsRigidAsPossible::solveIteration()
{
	if (_solve_iteration == 0)
		_ceres_logger.write("start non rigid registration with arap\n");
	if (!finished()) {
		_solve_iteration++;
		ceres::Solver::Summary summary;
		auto logger = _ceres_logger.CreateCeresIterationLogger(summary);

		ceres::Problem problem;
		VertexResidualIds fit_residual_ids = _fit_cost->addFitCost(problem, _deformation_graph, *(_deformed_mesh.get()), logger);
		EdgeResidualIds arap_residual_ids = addAsRigidAsPossibleCost(problem);

		ceres::Solve(_options, &problem, &summary);

		// evaluate
		if (_registration_options.evaluate_residuals) {
			evaluateResidual(problem, fit_residual_ids, arap_residual_ids, logger);
		}

		_last_cost = _current_cost;
		_current_cost = summary.final_cost;

		auto scale_factor_tol = 0.0001;// 0.00001;
		if (abs(_current_cost - _last_cost) < scale_factor_tol *(1 + _current_cost) &&
			(_registration_options.smooth > 0.005))// && a_conf > 0.05))
		{
			_registration_options.smooth /= 2.;
			_registration_options.conf /= 2.;
			std::cout << std::endl << "scale factor: smooth " << _registration_options.smooth << " conf " << _registration_options.conf;
		}
	}
	bool finished_registration = finished();
	if (finished_registration) {
		_ceres_logger.write("finished non rigid registration with arap ");
	}
	return finished_registration;
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
	return (_solve_iteration >= _registration_options.max_iterations) || (solved && _solve_iteration > 2);
}

void AsRigidAsPossible::evaluateResidual(ceres::Problem & problem,
										 std::map<vertex_descriptor, ResidualIds> & fit_residual_block_ids,
										 std::map<edge_descriptor, ResidualIds> & arap_residual_block_ids,
										 std::unique_ptr<CeresIterationLoggerGuard>& logger)
{
	// smooth
	auto smooth_cost = _deformation_graph._mesh.property_map<edge_descriptor, double>("e:smooth_cost");
	if (smooth_cost.second) {
		auto max_and_mean_cost = evaluateResiduals(_deformation_graph._mesh, problem, arap_residual_block_ids, smooth_cost.first, _registration_options.smooth);
		logger->write("max smooth: " + std::to_string(max_and_mean_cost.first) + " reference smooth " + std::to_string(max_and_mean_cost.second * 10.), false);
	}
	
	// fit
	//auto fit_cost = _deformation_graph._mesh.property_map<vertex_descriptor, double>("v:fit_cost");
	//if(fit_cost.second)
	//	evaluateResiduals(_deformation_graph._mesh, problem, fit_residual_block_ids, fit_cost.first, a_fit);

	//// conf
	//auto conf_cost = _deformation_graph._mesh.property_map<vertex_descriptor, double>("v:conf_cost");
	//if (conf_cost.second)
	//	evaluateResiduals(_deformation_graph._mesh, problem, conf_residual_block_ids, conf_cost.first, a_conf);
}

void AsRigidAsPossible::setDeformation(const Deformation & deformation_graph)
{
	_current_cost = 1.;
	_last_cost = 2.;
	_solve_iteration = 0;

	_deformation_graph = deformation_graph;
	_deformed_mesh = std::make_unique<DeformedMesh<Deformation>>(_src, _deformation_graph, _registration_options.dg_options.number_of_interpolation_neighbors);
}

void AsRigidAsPossible::setRigidDeformation(const RigidDeformation & rigid_deformation)
{
	_deformation_graph.setRigidDeformation(rigid_deformation);// createGlobalDeformationFromRigidDeformation(rigid_deformation));
}

bool AsRigidAsPossible::shouldBeSavedAsImage()
{
	return finished();
}

std::vector<vertex_descriptor> AsRigidAsPossible::subsetOfVerticesToFit()
{
	std::knuth_b rand_engine;
	auto randomBoolWithProb = [&rand_engine](double prob)
	{
		std::bernoulli_distribution d(prob);
		return d(rand_engine);
	};

	std::vector<vertex_descriptor> subset_of_vertices_to_fit;
	// comment out for random at in each iteration step
	if (_registration_options.use_vertex_random_probability < 1.) {
		for (auto & v : _deformed_mesh->vertices())
		{
			bool use_vertex = randomBoolWithProb(_registration_options.use_vertex_random_probability);
			if (use_vertex) {
				subset_of_vertices_to_fit.push_back(v);
			}
		}
		_ceres_logger.write("subset of vertices to use " + std::to_string(subset_of_vertices_to_fit.size()) + " / " + std::to_string(_deformed_mesh->number_of_vertices()), false);
	}
	return subset_of_vertices_to_fit;
}

void AsRigidAsPossible::init()
{
	_deformed_mesh = std::make_unique<DeformedMesh<Deformation>>(_src, _deformation_graph, _registration_options.dg_options.number_of_interpolation_neighbors);
	_ceres_logger.write("number of deformation graph nodes " + std::to_string(_deformation_graph._mesh.number_of_vertices()), false);
}


AsRigidAsPossible::AsRigidAsPossible(const SurfaceMesh& src,
									 const SurfaceMesh& dst,
									 std::vector<vertex_descriptor> fixed_positions,
									 const Deformation & deformation_graph,
									 ceres::Solver::Options option,
									 const RegistrationOptions & registration_options,
									 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)
	, _deformation_graph(deformation_graph)
	, _ceres_logger(logger)
	, _registration_options(registration_options)
	, _with_icp(false)
{
	init();
	_fit_cost = std::make_unique<AsRigidAsPossibleFitCostWithoutICP>(_dst, fixed_positions, subsetOfVerticesToFit(), _registration_options);
}


AsRigidAsPossible::AsRigidAsPossible(const SurfaceMesh& source,
									 const SurfaceMesh& target,
									 ceres::Solver::Options option,
									 const RegistrationOptions & registration_options,
									 std::shared_ptr<FileWriter> logger)
	: _src(source)
	, _dst(target)
	, _options(option)
	, _ceres_logger(logger)
	, _registration_options(registration_options)
{
	auto reduced_mesh = createReducedMesh(source, _registration_options.dg_options.edge_length, _registration_options.mesh_reduce_strategy);
	reduced_mesh.add_property_map<vertex_descriptor, double>("v:radius", _registration_options.dg_options.edge_length);

	auto global = createGlobalDeformation<ARAPDeformation>(source);
	_deformation_graph = createDeformationGraphFromMesh<ARAPDeformation>(reduced_mesh, global);
	init();
	_fit_cost = std::make_unique<AsRigidAsPossibleFitCost>(_dst, subsetOfVerticesToFit(), _registration_options);
}

AsRigidAsPossible::AsRigidAsPossible(const SurfaceMesh& src,
									 const SurfaceMesh& dst,
									 const Deformation & deformation_graph,
									 ceres::Solver::Options option,
									 const RegistrationOptions & registration_options,
									 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)
	, _deformation_graph(deformation_graph)
	, _ceres_logger(logger)
	, _registration_options(registration_options)
{
	init();
	_fit_cost = std::make_unique<AsRigidAsPossibleFitCost>(_dst, subsetOfVerticesToFit(), _registration_options);
}


//-----------------------------------------------------------------------------

ARAPDeformation createGlobalDeformationFromRigidDeformation(const RigidDeformation & rigid_deformation)
{
	ml::vec6d deformation;
	deformation[0] = rigid_deformation._r[0];
	deformation[1] = rigid_deformation._r[1];
	deformation[2] = rigid_deformation._r[2];
	deformation[3] = rigid_deformation._t[0];
	deformation[4] = rigid_deformation._t[1];
	deformation[5] = rigid_deformation._t[2];
	return ARAPDeformation(rigid_deformation._g, deformation);
}


std::unique_ptr<AsRigidAsPossible> createAsRigidAsPossible(const SurfaceMesh& src,
										                   const SurfaceMesh& dst,
										                   std::vector<vertex_descriptor> fixed_positions,
										                   ceres::Solver::Options option,
										                   const RegistrationOptions & registration_options,
										                   std::shared_ptr<FileWriter> logger)
{
	auto reduced_mesh = createReducedMesh(src, registration_options.dg_options.edge_length, registration_options.mesh_reduce_strategy);
	reduced_mesh.add_property_map<vertex_descriptor, double>("v:radius", registration_options.dg_options.edge_length);
	auto global = createGlobalDeformation<ARAPDeformation>(reduced_mesh);
	auto deformation_graph = createDeformationGraphFromMesh<ARAPDeformation>(reduced_mesh, global);
	return std::make_unique<AsRigidAsPossible>(src, dst, fixed_positions, deformation_graph, option, registration_options, logger);
}



}
