#include "ed.h"
#include "ed_deformation.h"
#include "ed_smooth_cost.h"
#include "ed_fit_cost.h"
#include "algo/registration/util/ceres_iteration_logger.h"
#include "algo/remeshing/mesh_simplification.h"
#include "algo/registration/source_point_selection/source_point_selection.h"
#include "ed_fit_cost_without_icp.h"

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
	DeformedMesh<Deformation> deformed(_target, inverse_deformation);
	return deformed.deformPoints();
}

std::vector<Point> EmbeddedDeformation::getFixedPositions()
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
	_deformed_mesh = std::make_unique<DeformedMesh<Deformation>>(_source, _deformation_graph);

	_ceres_logger.write("Number of deformation graph nodes " + std::to_string(_deformation_graph._mesh.number_of_vertices()));
}

	
SurfaceMesh EmbeddedDeformation::getDeformationGraphMesh()
{
	return deformationGraphToSurfaceMesh(_deformation_graph);
}


bool EmbeddedDeformation::solveIteration()
{
	if (_solve_iteration == 0)
		_ceres_logger.write("start non rigid registration with ed\n");
	
	if (!finished()) {
		_solve_iteration++;
		ceres::Solver::Summary summary;
		auto logger = _ceres_logger.CreateCeresIterationLogger(summary);
		
		ceres::Problem problem;

		_fit_cost->addFitCost(problem, _deformation_graph, *(_deformed_mesh.get()), logger);
		EdgeResidualIds smooth_residual_ids = _smooth_cost->smoothCost(problem, _options.smooth, _deformation_graph);
		VertexResidualIds rot_residual_ids = _smooth_cost->rotationCost(problem, _options.ed_rigid, _deformation_graph);

		ceres::Solve(_options.ceres_options, &problem, &summary);

		// evaluate
		if (_options.evaluate_residuals) {			
			_fit_cost->evaluateResiduals(problem, _deformation_graph._mesh, (*logger));
			_smooth_cost->evaluateResiduals(problem, _deformation_graph._mesh, (*logger));
		}

		_last_cost = _current_cost;
		_current_cost = summary.final_cost;
		updateSmoothFactor();

	}
	return finished();
}

double EmbeddedDeformation::currentError()
{
	return _current_cost;
}

void EmbeddedDeformation::updateSmoothFactor()
{
	if (_options.reduce_smooth_factor) {
		auto scale_factor_tol = 0.0001;
		if (abs(_current_cost - _last_cost) < scale_factor_tol *(1 + _current_cost) &&
			(_options.smooth > 0.005))// && a_conf > 0.05))
		{
			_options.smooth /= 2.;
			std::cout << std::endl << "scale factor: smooth " << _options.smooth;
		}
	}
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
	double tol = _options.ceres_options.function_tolerance;
	
	double error = abs(_last_cost - _current_cost);
	bool solved = error < (tol * _current_cost);
	return (_solve_iteration >= _options.max_iterations) || (solved && _solve_iteration > 2);
}


void EmbeddedDeformation::setRigidDeformation(const RigidDeformation & rigid_deformation)
{
	_deformation_graph.setRigidDeformation(rigid_deformation);
}


std::pair<bool, std::string> EmbeddedDeformation::shouldBeSavedAsImage()
{
	if (_options.adaptive_rigidity.enable && !_options.sequence_options.enable)
		return std::make_pair(true, "ed_adaptive_" + std::to_string(currentIteration()));
	else if (finished())
		return std::make_pair(true, "ed_" + std::to_string(currentIteration()));
	return std::make_pair(false, "");
}

void EmbeddedDeformation::init()
{
	_deformed_mesh = std::make_unique<DeformedMesh<Deformation>>(_source, _deformation_graph);
	_ceres_logger.write("number of deformation graph nodes " + std::to_string(_deformation_graph._mesh.number_of_vertices()), false);
	//if (_options.adaptive_rigidity.enable) {
	//	if (_options.adaptive_rigidity.refinement == Refinement::VERTEX)
	//		_smooth_cost = std::make_unique<AsRigidAsPossibleSmoothCostAdaptiveRigidityVertex>(_options);
	//	else
	//		_smooth_cost = std::make_unique<AsRigidAsPossibleSmoothCostAdaptiveRigidity>(_options);
	//}
	//else {
	_smooth_cost = std::make_unique<EmbeddedDeformationSmoothCost>(_options.smooth);
	//}
	_selected_subset = selectRandomSubset(*_deformed_mesh.get(), _options.use_vertex_random_probability);
	_ceres_logger.write("subset of vertices to use " + std::to_string(_selected_subset.size()) + " / " + std::to_string(_deformed_mesh->number_of_vertices()), false);
}



EmbeddedDeformation::EmbeddedDeformation(const SurfaceMesh& source,
										 const SurfaceMesh& target,
										 std::vector<vertex_descriptor> fixed_positions,
										 const DeformationGraph<EDDeformation> & deformation_graph,
										 const RegistrationOptions & options,
										 std::shared_ptr<FileWriter> logger)
	: _source(source)
	, _target(target)
	, _options(options)
	, _deformation_graph(deformation_graph)
	, _fixed_positions(fixed_positions)
	, _ceres_logger(logger)
{
	init();
	_fit_cost = std::make_unique<EmbeddedDeformationFitCostWithoutICP>(_target, fixed_positions, _selected_subset, _options);
}

EmbeddedDeformation::EmbeddedDeformation(const SurfaceMesh& source,
										 const SurfaceMesh& target,
										 const RegistrationOptions & options,
										 std::shared_ptr<FileWriter> logger)
	: _source(source)
	, _target(target)
	, _options(options)
	, _ceres_logger(logger)
{
	double edge_length = deformationGraphEdgeLength(source, _options.deformation_graph.edge_length_percentage_of_area);
	logger->write("used edge length " + std::to_string(edge_length));
	
	auto reduced_mesh = createReducedMesh(source, edge_length, _options.mesh_reduce_strategy);
	reduced_mesh.add_property_map<vertex_descriptor, double>("v:radius", edge_length);
	
	auto global = createGlobalDeformation<EDDeformation>(_source);
	_deformation_graph = createDeformationGraphFromMesh<EDDeformation>(reduced_mesh, global, _options.deformation_graph.number_of_interpolation_neighbors);

	init();
	_fit_cost = std::make_unique<EmbeddedDeformationFitCost>(_target, _selected_subset, _options);
}


EmbeddedDeformation::EmbeddedDeformation(const SurfaceMesh& source,
										 const SurfaceMesh& target,
										 const DeformationGraph<EDDeformation> & deformation_graph,
										 const RegistrationOptions & options,
										 std::shared_ptr<FileWriter> logger)
	: _source(source)
	, _target(target)
	, _options(options)
	, _deformation_graph(deformation_graph)
	, _ceres_logger(logger)
{
	init();
	_fit_cost = std::make_unique<EmbeddedDeformationFitCost>(_target, _selected_subset, _options);
}

EmbeddedDeformation::EmbeddedDeformation(const SurfaceMesh& source,
										 const SurfaceMesh& target,
										 const SurfaceMesh& previous_mesh,
										 const Deformation & deformation_graph,
										 const RegistrationOptions & options,
										 std::shared_ptr<FileWriter> logger)
	: EmbeddedDeformation(source, target, deformation_graph, options, logger)
{}

//-----------------------------------------------------------------------------

EDDeformation createGlobalEDDeformationFromRigidDeformation(const RigidDeformation & rigid_deformation)
{	
	return EDDeformation(rigid_deformation);
}


std::unique_ptr<EmbeddedDeformation> createEmbeddedDeformation(const SurfaceMesh& source,
															   const SurfaceMesh& target,
															   std::vector<vertex_descriptor> fixed_positions,
															   const RegistrationOptions & options,
															   std::shared_ptr<FileWriter> logger)
{
	double edge_length = deformationGraphEdgeLength(source, options.deformation_graph.edge_length_percentage_of_area);
	auto reduced_mesh = createReducedMesh(source, edge_length, options.mesh_reduce_strategy);
	reduced_mesh.add_property_map<vertex_descriptor, double>("v:radius", edge_length);
	auto global = createGlobalDeformation<EDDeformation>(reduced_mesh);
	auto deformation_graph = createDeformationGraphFromMesh<EDDeformation>(reduced_mesh, global, options.deformation_graph.number_of_interpolation_neighbors);
	return std::make_unique<EmbeddedDeformation>(source, target, fixed_positions, deformation_graph, options, logger);
}


std::unique_ptr<EmbeddedDeformation> createEmbeddedDeformation(const SurfaceMesh& source,
															   const SurfaceMesh& target,
															   const RegistrationOptions & options,
															   std::shared_ptr<FileWriter> logger)
{
	double edge_length = deformationGraphEdgeLength(source, options.deformation_graph.edge_length_percentage_of_area);
	auto reduced_mesh = createReducedMesh(source, edge_length, options.mesh_reduce_strategy);
	reduced_mesh.add_property_map<vertex_descriptor, double>("v:radius", edge_length);
	auto global = createGlobalDeformation<EDDeformation>(reduced_mesh);
	auto deformation_graph = createDeformationGraphFromMesh<EDDeformation>(reduced_mesh, global, options.deformation_graph.number_of_interpolation_neighbors);
	return std::make_unique<EmbeddedDeformation>(source, target, deformation_graph, options, logger);
}

}
