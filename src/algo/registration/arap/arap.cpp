#include "arap.h"
#include "arap_fit_cost.h"
#include "arap_fit_cost_without_icp.h"
#include "arap_smooth_cost.h"
#include "arap_smooth_cost_adaptive_rigidity.h"
#include "arap_smooth_cost_adaptive_rigidity_vertex.h"
#include "algo/registration/util/ceres_iteration_logger.h"
#include "algo/remeshing/mesh_simplification.h"
#include "algo/registration/util/ceres_residual_evaluation.h"
#include <random>
#include "algo/registration/source_point_selection/source_point_selection.h"


namespace Registration {


const SurfaceMesh & AsRigidAsPossible::getSource()
{
	return _source;
}

const SurfaceMesh & AsRigidAsPossible::getTarget()
{
	return _target;
}

SurfaceMesh AsRigidAsPossible::getDeformedPoints()
{
	return _deformed_mesh->deformPoints();
}

SurfaceMesh AsRigidAsPossible::getInverseDeformedPoints()
{
	auto inverse_deformation = _deformation_graph.invertDeformation();
	DeformedMesh<Deformation> deformed(_target, inverse_deformation);
	return deformed.deformPoints();
}

const DeformationGraph<ARAPDeformation> & AsRigidAsPossible::getDeformation()
{
	return _deformation_graph;
}

SurfaceMesh & AsRigidAsPossible::getDeformationMesh()
{
	return _deformation_graph._mesh;
}

SurfaceMesh AsRigidAsPossible::getDeformationGraphMesh()
{
	return deformationGraphToSurfaceMesh(_deformation_graph);
}

std::vector<Point> AsRigidAsPossible::getFixedPositions()
{
	return _fit_cost->getFixedPostions();
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
		_fit_cost->addFitCost(problem, _deformation_graph, *(_deformed_mesh.get()), logger);
		EdgeResidualIds arap_residual_ids = _smooth_cost->asRigidAsPossibleCost(problem, _options.smooth, _deformation_graph);

		ceres::Solve(_ceres_options, &problem, &summary);

		// evaluate
		if (_options.evaluate_residuals) {
			_smooth_cost->evaluateResiduals(problem, _deformation_graph._mesh, (*logger));
			_fit_cost->evaluateResiduals(problem, _deformation_graph._mesh, (*logger));
		}

		_last_cost = _current_cost;
		_current_cost = summary.final_cost;
		updateSmoothFactor();
	}
	bool finished_registration = finished();
	if (finished_registration) {
		_ceres_logger.write("finished non rigid registration with arap ");
	}
	return finished_registration;
}

double AsRigidAsPossible::currentError()
{
	return _current_cost;
}

void AsRigidAsPossible::updateSmoothFactor()
{
	if (_options.reduce_smooth_factor)
	{
		auto scale_factor_tol = 0.01;

		if (abs(_current_cost - _last_cost) < (scale_factor_tol * _current_cost) &&
			(_options.smooth > 0.01))
		{
			_options.smooth /= 2.;

			_ceres_logger.write(" scale factor: smooth " + std::to_string(_options.smooth), false);			
		}
	}
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
	auto tol = _ceres_options.function_tolerance;

	double error = abs(_last_cost - _current_cost);
	bool solved = error < (tol * _current_cost);
	return (_solve_iteration >= _options.max_iterations) || (solved && _solve_iteration > 2);
}

void AsRigidAsPossible::setDeformation(const Deformation & deformation_graph)
{
	_current_cost = 1.;
	_last_cost = 2.;
	_solve_iteration = 0;

	_deformation_graph = deformation_graph;
	_deformed_mesh = std::make_unique<DeformedMesh<Deformation>>(_source, _deformation_graph);

	_ceres_logger.write("Number of deformation graph nodes " + std::to_string(_deformation_graph._mesh.number_of_vertices()));
}



void AsRigidAsPossible::setRigidDeformation(const RigidDeformation & rigid_deformation)
{
	_deformation_graph.setRigidDeformation(rigid_deformation);// createGlobalDeformationFromRigidDeformation(rigid_deformation));
}

std::pair<bool, std::string> AsRigidAsPossible::shouldBeSavedAsImage()
{
	if (_options.adaptive_rigidity.enable && !_options.sequence_options.enable)
		return std::make_pair(true, "arap_adaptive_" + std::to_string(currentIteration()));
	if (finished())
		return std::make_pair(true, "arap_" + std::to_string(currentIteration()));
	return std::make_pair(false, "");
}

void AsRigidAsPossible::init()
{
	_deformed_mesh = std::make_unique<DeformedMesh<Deformation>>(_source, _deformation_graph);
	_ceres_logger.write("number of deformation graph nodes " + std::to_string(_deformation_graph._mesh.number_of_vertices()), false);
	if (_options.adaptive_rigidity.enable) {
		if(_options.adaptive_rigidity.refinement == Refinement::VERTEX)
			_smooth_cost = std::make_unique<AsRigidAsPossibleSmoothCostAdaptiveRigidityVertex>(_options);
		else
			_smooth_cost = std::make_unique<AsRigidAsPossibleSmoothCostAdaptiveRigidity>(_options);
	}
	else {
		_smooth_cost = std::make_unique<AsRigidAsPossibleSmoothCost>(_options.smooth);
	}

	_selected_subset = selectRandomSubset(*_deformed_mesh.get(), _options.use_vertex_random_probability);
	_ceres_logger.write("subset of vertices to use " + std::to_string(_selected_subset.size()) + " / " + std::to_string(_deformed_mesh->number_of_vertices()), false);

}

AsRigidAsPossible::AsRigidAsPossible(const SurfaceMesh& source,
									 const SurfaceMesh& target,
									 std::vector<vertex_descriptor> fixed_positions,
									 const Deformation & deformation_graph,
									 const RegistrationOptions & options,
									 std::shared_ptr<FileWriter> logger)
	: _source(source)
	, _target(target)
	, _ceres_options(options.ceres_options)
	, _deformation_graph(deformation_graph)
	, _ceres_logger(logger)
	, _options(options)
{
	init();
	_fit_cost = std::make_unique<AsRigidAsPossibleFitCostWithoutICP>(_target, fixed_positions, _selected_subset, _options);
}


AsRigidAsPossible::AsRigidAsPossible(const SurfaceMesh& source,
									 const SurfaceMesh& target,
									 const RegistrationOptions & options,
									 std::shared_ptr<FileWriter> logger)
	: _source(source)
	, _target(target)
	, _ceres_options(options.ceres_options)
	, _ceres_logger(logger)
	, _options(options)
{
	double edge_length = deformationGraphEdgeLength(source, _options.deformation_graph.edge_length_percentage_of_area);
	logger->write("used edge length " + std::to_string(edge_length));
	
	auto reduced_mesh = createReducedMesh(source, edge_length, _options.mesh_reduce_strategy);
	reduced_mesh.add_property_map<vertex_descriptor, double>("v:radius", edge_length);

	auto global = createGlobalDeformation<ARAPDeformation>(source);
	_deformation_graph = createDeformationGraphFromMesh<ARAPDeformation>(reduced_mesh, global, _options.deformation_graph.number_of_interpolation_neighbors);
	init();
	
	_fit_cost = std::make_unique<AsRigidAsPossibleFitCost>(_target, _selected_subset, _options);
}

AsRigidAsPossible::AsRigidAsPossible(const SurfaceMesh& src,
									 const SurfaceMesh& dst,
									 const Deformation & deformation_graph,
									 const RegistrationOptions & options,
									 std::shared_ptr<FileWriter> logger)
	: _source(src)
	, _target(dst)
	, _ceres_options(options.ceres_options)
	, _deformation_graph(deformation_graph)
	, _ceres_logger(logger)
	, _options(options)
{
	init();
	_fit_cost = std::make_unique<AsRigidAsPossibleFitCost>(_target, _selected_subset, _options);
}


AsRigidAsPossible::AsRigidAsPossible(const SurfaceMesh& source,
									 const SurfaceMesh& target,
									 const SurfaceMesh& previous_mesh,
									 const Deformation & deformation_graph,
									 const RegistrationOptions & options,
									 std::shared_ptr<FileWriter> logger)
	: AsRigidAsPossible(source, target, deformation_graph, options, logger)
{}

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


std::unique_ptr<AsRigidAsPossible> createAsRigidAsPossible(const SurfaceMesh& source,
										                   const SurfaceMesh& target,
										                   std::vector<vertex_descriptor> fixed_positions,
										                   const RegistrationOptions & options,
										                   std::shared_ptr<FileWriter> logger)
{
	double edge_length = deformationGraphEdgeLength(source, options.deformation_graph.edge_length_percentage_of_area);
	logger->write("used edge length " + std::to_string(edge_length));
	
	auto reduced_mesh = createReducedMesh(source, edge_length, options.mesh_reduce_strategy);
	reduced_mesh.add_property_map<vertex_descriptor, double>("v:radius", edge_length);
	auto global = createGlobalDeformation<ARAPDeformation>(reduced_mesh);
	auto deformation_graph = createDeformationGraphFromMesh<ARAPDeformation>(reduced_mesh, global, options.deformation_graph.number_of_interpolation_neighbors);
	return std::make_unique<AsRigidAsPossible>(source, target, fixed_positions, deformation_graph, options, logger);
}



}
