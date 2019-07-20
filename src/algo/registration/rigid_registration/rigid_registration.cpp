#include "stdafx.h"
#include "rigid_registration.h"
#include "rigid_deformation_cost_function.h"
#include "algo/ceres_iteration_logger.h"
#include "algo/registration/deformation_graph/deformed_mesh.h" // set color todo
#include "algo/registration/ceres_residual_evaluation.h"


const SurfaceMesh & RigidRegistration::getSource()
{
	return _source;
}

const SurfaceMesh & RigidRegistration::getTarget()
{
	return _target;
}

SurfaceMesh RigidRegistration::getDeformedPoints()
{
	RigidDeformedMesh deformed(getSource(), _deformation);
	return deformed.deformPoints();
}

SurfaceMesh RigidRegistration::getInverseDeformedPoints()
{
	auto inverseDeformation = _deformation.invertDeformation();
	RigidDeformedMesh deformed(getSource(), inverseDeformation);
	return deformed.deformPoints();
}

const RigidDeformation & RigidRegistration::getRigidDeformation()
{
	return _deformation;
}

ceres::ResidualBlockId RigidRegistration::addPointToPointCost(ceres::Problem &problem, const Point & source_point, vertex_descriptor target_vertex)
{
	float point_to_point_weight = 0.1f;
	auto target_point = _target.point(target_vertex);

	auto cost_function = FitPointToPointAngleAxisCostFunction::Create(source_point, target_point);
	auto loss_function = new ceres::ScaledLoss(NULL, point_to_point_weight, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function, _deformation.r(), _deformation.t());
}

ceres::ResidualBlockId RigidRegistration::addPointToPlaneCost(ceres::Problem &problem, const Point & source_point, vertex_descriptor target_vertex)
{
	float point_to_plane_weight = 0.9f;
	auto target_normals = _target.property_map<vertex_descriptor, Vector>("v:normal").first;
	auto target_point = _target.point(target_vertex);
	auto target_normal = target_normals[target_vertex];

	auto cost_function = FitPointToPlaneAngleAxisCostFunction::Create(source_point, target_point, target_normal);
	auto loss_function = new ceres::ScaledLoss(NULL, point_to_plane_weight, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function, _deformation.r(), _deformation.t());
}


std::map<vertex_descriptor, ResidualIds> RigidRegistration::addFitCost(ceres::Problem &problem, std::unique_ptr<CeresIterationLoggerGuard> & logger)
{
	VertexResidualIds residual_ids;

	int i = 0;
	auto normal = _source.property_map<vertex_descriptor, Vector>("v:normal").first;
	for (auto & v : _set_of_vertices_to_use)//_source.vertices())
	{
		auto point = _source.point(v);
		auto deformed_point = _deformation.deformPoint(point);
		auto deformed_normal = _deformation.deformNormal(normal[v]);
		auto correspondent_point = _find_correspondence_point->correspondingPoint(deformed_point, deformed_normal);
		if (correspondent_point.first) {
			auto target_vertex = correspondent_point.second;
			residual_ids[v].push_back(addPointToPointCost(problem, point, target_vertex));
			residual_ids[v].push_back(addPointToPlaneCost(problem, point, target_vertex));
			i++;
		}
	}
	logger->write("used " + std::to_string(i) + " / " + std::to_string(_source.number_of_vertices()) + " vertices ");
	return residual_ids;
}


std::map<vertex_descriptor, ResidualIds> RigidRegistration::addFitCostWithoutICP(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;
	// asuming points are ordered
	auto normal = _source.property_map<vertex_descriptor, Vector>("v:normal").first;
	for (auto & v : _source.vertices())
	{
		residual_ids[v].push_back(addPointToPointCost(problem, _source.point(v), v));
		residual_ids[v].push_back(addPointToPlaneCost(problem, _source.point(v), v));
	}
	return residual_ids;
}

bool RigidRegistration::solveIteration()
{
	if (!finished()) {
		_solve_iteration++;
		ceres::Solver::Summary summary;

		auto logger = _ceres_logger.CreateCeresIterationLogger(summary);

		ceres::Problem problem;

		VertexResidualIds fit_residual_ids;
		if (_with_icp)
			fit_residual_ids = addFitCost(problem, logger);
		else
			fit_residual_ids = addFitCostWithoutICP(problem);

		ceres::Solve(_options, &problem, &summary);

		// evaluate		
		evaluateResidual(problem, fit_residual_ids, logger);

		_last_cost = _current_cost;
		_current_cost = summary.final_cost;
	}
	bool finished_registration = finished();
	if (finished_registration) {
		_ceres_logger.write("finished Rigid registration \n");
	}
	return finished_registration;
}

size_t RigidRegistration::currentIteration()
{
	return _solve_iteration;
}

bool RigidRegistration::solve()
{
	while (!finished()) {
		solveIteration();
	}
	return true;
}

bool RigidRegistration::finished()
{
	auto tol = _options.function_tolerance * 0.1;

	double error = abs(_last_cost - _current_cost);
	bool solved = error < (tol * _current_cost);
	return (_solve_iteration >= _max_iterations) || (solved && _solve_iteration > 2);
}

void RigidRegistration::evaluateResidual(ceres::Problem & problem,
										 std::map<vertex_descriptor, ResidualIds> & fit_residual_block_ids,
										 std::unique_ptr<CeresIterationLoggerGuard>& logger)
{
	auto fit_cost = _source.add_property_map<vertex_descriptor, double>("v:fit_cost");

	if (fit_cost.second) {
		auto max_and_mean_cost = evaluateResiduals(_source, problem, fit_residual_block_ids, fit_cost.first, 1.);
		logger->write("max smooth costs: " + std::to_string(max_and_mean_cost.first) + " reference smooth cost " + std::to_string(max_and_mean_cost.second * 10.), false);
	}
}

RigidRegistration::RigidRegistration(const SurfaceMesh & source,
									 const SurfaceMesh & target,
									 ceres::Solver::Options option, 
									 double use_vertex_random_probability,
									 std::shared_ptr<FileWriter> logger)
	: _source(source)
	, _target(target)
	, _options(option)
	, _use_vertex_random_probability(use_vertex_random_probability)
	, _ceres_logger(logger)
{
	_ceres_logger.write("start Rigid registration \n");
	_find_correspondence_point = std::make_unique<FindCorrespondingPoints>(_target, 0.5, 45.);
	_rigid_deformed_mesh = std::make_unique<RigidDeformedMesh>(_source, _deformation);

	if (_use_vertex_random_probability < 1.) {

		std::knuth_b _rand_engine;
		std::bernoulli_distribution d(_use_vertex_random_probability);
		for (auto & v : _source.vertices())
		{
			bool use_vertex = d(_rand_engine);
			if (use_vertex) {
				_set_of_vertices_to_use.push_back(v);
			}
		}
		_ceres_logger.write("subset of vertices to use " + std::to_string(_set_of_vertices_to_use.size()) + " / " + std::to_string(_source.number_of_vertices()), false);
	}
	else {
		for (auto & v : _source.vertices())
		{
			_set_of_vertices_to_use.push_back(v);
		}
	}
}

