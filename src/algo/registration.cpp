#include "registration.h"

bool RigidRegistration::solve()
{
	_transformation = iterative_closest_points(_points_a, _points_b);
	return true;
}


void RigidRegistration::icp_calc_nn_in_cost_function()
{
	if (!_icp_nn) {
		ceres::Solver::Options options;
		options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
		options.minimizer_type = ceres::MinimizerType::TRUST_REGION;
		options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
		options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
		options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
		options.preconditioner_type = ceres::PreconditionerType::JACOBI;// SCHUR_JACOBI;
		options.max_num_iterations = 50;
		options.logging_type = ceres::LoggingType::SILENT;
		options.minimizer_progress_to_stdout = false;
		_icp_nn = std::make_unique<ICP>(_points_a, _points_b, options);

		//if (!_icp_nn->finished())
			//_transformation = _icp_nn->solveIteration();
		_transformation = _icp_nn->solve();
	}
}

std::vector<ml::vec3f> RigidRegistration::getPointsA()
{
	auto transformed_points = _points_a;
	std::for_each(transformed_points.begin(), transformed_points.end(), [&](ml::vec3f & p) { p = _transformation * p; });
	return transformed_points;
}

std::vector<ml::vec3f> RigidRegistration::getPointsB()
{
	return _points_b;
}

std::vector<ml::vec3f> RigidRegistration::getPointsDeformationGraph()
{
	return std::vector<ml::vec3f>();
}

RigidRegistration::RigidRegistration(const std::vector<ml::vec3f> & points_a, const std::vector<ml::vec3f> & points_b)
	: _points_a(points_a)
	, _points_b(points_b)
	, _transformation(ml::mat4f::identity())
{}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

bool NonRigidRegistration::solve()
{

	//AsRigidAsPossible arap(_points_a, _points_b, options);
	//_points_b = arap.solve();
	if (!_embedded_deformation) {
		ceres::Solver::Options options;
		options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
		options.minimizer_type = ceres::MinimizerType::TRUST_REGION;
		options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
		options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
		options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
		options.preconditioner_type = ceres::PreconditionerType::JACOBI;// SCHUR_JACOBI;
		options.max_num_iterations = 50;
		options.logging_type = ceres::LoggingType::SILENT;
		options.minimizer_progress_to_stdout = false;

		_embedded_deformation = std::make_unique<EmbeddedDeformation>(_points_a, _points_b, options);
		//_as_rigid_as_possible = std::make_unique<AsRigidAsPossible>(_points_a, _points_b, options, 1000);
	}
	if (_embedded_deformation && !_embedded_deformation->finished()) {
		_embedded_deformation->solveIteration();
		_points_a = _embedded_deformation->getDeformedPoints();
		return true;
	}
	if (_as_rigid_as_possible && !_as_rigid_as_possible->finished()) {
		_as_rigid_as_possible->solveIteration();
		_points_a = _as_rigid_as_possible->getDeformedPoints();
		return true;
	}
	return false;
}


std::vector<ml::vec3f> NonRigidRegistration::getPointsA()
{
	return _points_a;
}

std::vector<ml::vec3f> NonRigidRegistration::getPointsB()
{
	return _points_b;
}

std::vector<ml::vec3f> NonRigidRegistration::getPointsDeformationGraph()
{
	if (_embedded_deformation)
		return _embedded_deformation->getDeformationGraph();
	else if (_as_rigid_as_possible)
		return _as_rigid_as_possible->getDeformationGraph();
	else
		return std::vector<ml::vec3f>();
}

NonRigidRegistration::NonRigidRegistration()
	: _transformation()
{
	for (int i = 0; i < 50; i++) {
		float x = 0.01 *static_cast<float>(i);
		_points_a.push_back({ x,0.,0. });
	}

	_points_b = _points_a;
	for (int i = 25; i < 50; i++) {
		float z = 0.005 *static_cast<double>(i - 24);
		_points_b[i].y = z;
	}
}

NonRigidRegistration::NonRigidRegistration(const std::vector<ml::vec3f> & points_a, const std::vector<ml::vec3f> & points_b)
	: _points_a(points_a)
	, _points_b(points_b)
{}