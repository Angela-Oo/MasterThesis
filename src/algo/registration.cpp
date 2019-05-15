#include "registration.h"

ceres::Solver::Options ceresOption() {
	ceres::Solver::Options options;
	options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
	options.minimizer_type = ceres::MinimizerType::TRUST_REGION;
	options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
	options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
	options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY; //ceres::LinearSolverType::CGNR
	options.preconditioner_type = ceres::PreconditionerType::JACOBI;// SCHUR_JACOBI;
	options.max_num_iterations = 50;// 100;
	options.logging_type = ceres::LoggingType::SILENT;
	options.minimizer_progress_to_stdout = false;
	return options;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

bool RigidRegistration::solve()
{
	while (!_icp_nn->finished()) {
		_transformation = _icp_nn->solveIteration();
	}
	return true;
}

bool RigidRegistration::finished()
{
	return _icp_nn->finished();
}


bool RigidRegistration::solveIteration()
{
	if (!_icp_nn->finished()) {
		_transformation = _icp_nn->solveIteration();
	}
	return finished();
}

const Mesh & RigidRegistration::getSource()
{
	return _points_a;
}

const Mesh & RigidRegistration::getTarget()
{
	return _points_b;
}

Mesh RigidRegistration::getDeformedPoints()
{
	auto transformed_points = _points_a;
	std::for_each(transformed_points.m_vertices.begin(), transformed_points.m_vertices.end(), [&](Mesh::Vertex & p) { p.position = _transformation * p.position; });
	return transformed_points;
}

Mesh RigidRegistration::getInverseDeformedPoints()
{
	auto transformed_points = _points_a;
	auto inverse_transformation = _transformation.getInverse();
	std::for_each(transformed_points.m_vertices.begin(), transformed_points.m_vertices.end(), [&](Mesh::Vertex & p) { p.position = inverse_transformation * p.position; });
	return transformed_points;
}


RigidRegistration::RigidRegistration(const Mesh & points_a, const Mesh & points_b, std::shared_ptr<FileWriter> logger)
	: _points_a(points_a)
	, _points_b(points_b)
	, _transformation(ml::mat4f::identity())
	, _logger(logger)
{
	std::vector<ml::vec3f> positions_a;
	for (auto & p : _points_a.getVertices())
		positions_a.push_back(p.position);
	
	std::vector<ml::vec3f> positions_b;
	for (auto & p : _points_b.getVertices())
		positions_b.push_back(p.position);
	_icp_nn = std::make_unique<ICPNN>(positions_a, positions_b, ceresOption(), logger);
}

