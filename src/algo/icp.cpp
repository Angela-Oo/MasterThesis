#include "stdafx.h"
#include "icp.h"

using namespace Eigen;

#include <ceres/local_parameterization.h>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/types.h>
#include <ceres/rotation.h>
#include <ceres/loss_function.h>
#include "ext-flann/nearestNeighborSearchFLANN.h"
#include "core-util/nearestNeighborSearch.h"
#include "icp_cost_function.h"
#include "se3.h"
#include "ceres_iteration_logger.h"
#include "knn.h"


ICP::ICP(const std::vector<ml::vec3f>& src,
		 const std::vector<ml::vec3f>& dst,
		 ceres::Solver::Options option)
	: _src(src)
	, _dst(dst)
	, _options(option)
{
	std::cout << "\nCeres Solver" << std::endl;
	std::cout << "Ceres preconditioner type: " << _options.preconditioner_type << std::endl;
	std::cout << "Ceres linear algebra type: " << _options.sparse_linear_algebra_library_type << std::endl;
	std::cout << "Ceres linear solver type: " << _options.linear_solver_type << std::endl;
}

ml::mat4f ICP::solveFixNN(ml::vec6d transformation_se3)
{
	ceres::Solver::Summary summary;
	ICPLogIterationGuard log_guard(summary);
	KNN nn_search(_dst);
	ceres::Problem problem;
	for (int i = 0; i < _src.size(); ++i) {
		unsigned int index = nn_search.nearest_index(_src[i]);
		ceres::CostFunction* cost_function = PointToPointErrorSE3::Create(_dst[index], _src[i]);
		problem.AddResidualBlock(cost_function, NULL, transformation_se3.array);
	}
	ceres::Solve(_options, &problem, &summary);

	return rigid_transformation_from_se3(transformation_se3);
}

ml::mat4f ICP::solve()
{
	ceres::Solver::Summary summary;
	ICPLogIterationGuard log_guard(summary);

	KNN nn_search(_dst);
	ml::vec6d rotation_translation(0., 0., 0., 0., 0., 0.);
	ceres::Problem problem;
	for (int i = 0; i < _src.size(); ++i) {
		ceres::CostFunction* cost_function = PointToPointsErrorSE3NNSearch::Create(_src[i], std::bind(&KNN::nearest_f, &nn_search, std::placeholders::_1));
		problem.AddResidualBlock(cost_function, NULL, rotation_translation.array);
	}
	ceres::Solve(_options, &problem, &summary);
	return rigid_transformation_from_se3(rotation_translation);
}



//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------


ICPNN::ICPNN(const std::vector<ml::vec3f>& src,
			 const std::vector<ml::vec3f>& dst,
			 ceres::Solver::Options options)
	: _src(src)
	, _dst(dst)
	, _options(options)
	, _nn_search(dst)
{
	std::cout << "\nCeres Solver" << std::endl;
	std::cout << "Ceres preconditioner type: " << options.preconditioner_type << std::endl;
	std::cout << "Ceres linear algebra type: " << options.sparse_linear_algebra_library_type << std::endl;
	std::cout << "Ceres linear solver type: " << options.linear_solver_type << std::endl;
}

ml::mat4f ICPNN::solve()
{
	ml::mat4f transformation = ml::mat4f::identity();
	while (!finished()) {
		transformation = solveIteration();
	}
	return transformation;
}

ml::mat4f ICPNN::solveTransformDataset()
{
	ml::mat4f transformation = ml::mat4f::identity();
	while (!finished()) {
		transformation = solveIterationTransformDataset();
	}
	return transformation;
}

ml::mat4f ICPNN::solveIteration() // working
{
	if (!finished()) {
		ceres::Solver::Summary summary;
		ICPLogIterationGuard logger(summary, _total_time_in_ms, _solve_iteration);

		_solve_iteration++;
		
		ceres::Problem problem;
		for (int i = 0; i < _src.size(); i++) {
			ml::mat4f transform_matrix = rigid_transformation_from_se3(_transformation_se3);
			unsigned int index = _nn_search.nearest_index((transform_matrix * _src[i]));
			ceres::CostFunction* cost_function = PointToPointErrorSE3::Create(_dst[index], _src[i]);
			problem.AddResidualBlock(cost_function, NULL, _transformation_se3.array);
		}
		ceres::Solve(_options, &problem, &summary);

		_current_tol = abs(_current_cost - summary.final_cost);
		_current_cost = summary.final_cost;

		_total_time_in_ms += logger.get_time_in_ms();
	}

	return rigid_transformation_from_se3(_transformation_se3);
}

ml::mat4f ICPNN::solveIterationTransformDataset()
{
	if (!finished()) {
		ceres::Solver::Summary summary;
		ICPLogIterationGuard logger(summary, _total_time_in_ms, _solve_iteration);

		_solve_iteration++;

		// icp
		ml::vec6d transformation(0., 0., 0., 0., 0., 0.);
		ceres::Problem problem;
		for (int i = 0; i < _src.size(); i++) {
			unsigned int index = _nn_search.nearest_index(_src[i]);
			ceres::CostFunction* cost_function = PointToPointErrorSE3::Create(_dst[index], _src[i]);
			problem.AddResidualBlock(cost_function, NULL, transformation.array);
		}
		ceres::Solve(_options, &problem, &summary);


		ml::mat4f transform_matrix = rigid_transformation_from_se3(transformation);
		std::for_each(_src.begin(), _src.end(), [&](ml::vec3f & p) { p = transform_matrix * p; });
		_transformation_se3 += transformation;

		_current_tol = abs(_current_cost - summary.final_cost);
		_current_cost = summary.final_cost;

		_total_time_in_ms += logger.get_time_in_ms();
	}
	return rigid_transformation_from_se3(_transformation_se3);
}

bool ICPNN::finished()
{
	double tol = 0.001;
	return (_solve_iteration >= _max_iterations || _current_tol < tol);
}


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

ICPPointSubset::ICPPointSubset(const std::vector<ml::vec3f>& src,
							   const std::vector<ml::vec3f>& dst,
							   ceres::Solver::Options options)
	: _src(src)
	, _dst(dst)
	, _options(options)
	, _nn_search(dst)
{
	std::cout << "\nCeres Solver" << std::endl;
	std::cout << "Ceres preconditioner type: " << options.preconditioner_type << std::endl;
	std::cout << "Ceres linear algebra type: " << options.sparse_linear_algebra_library_type << std::endl;
	std::cout << "Ceres linear solver type: " << options.linear_solver_type << std::endl;
}


ml::mat4f ICPPointSubset::solve()
{
	ml::mat4f transformation = ml::mat4f::identity();
	while (!finished()) {
		transformation = solveIteration();
	}
	return transformation;
}

ml::mat4f ICPPointSubset::solveTransformDataset()
{
	ml::mat4f transformation = ml::mat4f::identity();
	while (!finished()) {
		transformation = solveIterationTransformDataset();
	}
	return transformation;
}

ml::mat4f ICPPointSubset::solveIteration()
{
	if (!finished()) {
		ceres::Solver::Summary summary;
		ICPLogIterationGuard logger(summary, _total_time_in_ms, _solve_iteration);

		_solve_iteration++;

		//int step = std::pow(15 - _solve_iteration, 2);
		int step = 20 - _solve_iteration;
		int step_size = std::max(step, 1);

		// icp
		ceres::Problem problem;
		for (int i = 0; i < _src.size(); i += step_size) {
			ml::mat4f transform_matrix = rigid_transformation_from_se3(_transformation_se3);
			unsigned int index = _nn_search.nearest_index((transform_matrix * _src[i]));
			ceres::CostFunction* cost_function = PointToPointErrorSE3::Create(_dst[index], _src[i]);
			problem.AddResidualBlock(cost_function, NULL, _transformation_se3.array);
		}
		ceres::Solve(_options, &problem, &summary);

		_current_tol = abs(_current_cost - summary.final_cost);
		_current_cost = summary.final_cost;

		_total_time_in_ms += logger.get_time_in_ms();
	}

	return rigid_transformation_from_se3(_transformation_se3);
}

ml::mat4f ICPPointSubset::solveIterationTransformDataset()
{
	if (!finished()) {
		ceres::Solver::Summary summary;
		ICPLogIterationGuard logger(summary, _total_time_in_ms, _solve_iteration);

		_solve_iteration++;

		//int step = std::pow(15 - _solve_iteration, 2);
		int step = 20 - _solve_iteration;
		int step_size = std::max(step, 1);

		// icp
		ml::vec6d transformation(0., 0., 0., 0., 0., 0.);
		ceres::Problem problem;
		for (int i = 0; i < _src.size(); i += step_size) {
			unsigned int index = _nn_search.nearest_index(_src[i]);
			ceres::CostFunction* cost_function = PointToPointErrorSE3::Create(_dst[index], _src[i]);
			problem.AddResidualBlock(cost_function, NULL, transformation.array);
		}
		ceres::Solve(_options, &problem, &summary);

		_transformation_se3 += transformation;

		ml::mat4f transform_matrix = rigid_transformation_from_se3(transformation);
		std::for_each(_src.begin(), _src.end(), [&](ml::vec3f & p) { p = transform_matrix * p; });


		_current_tol = abs(_current_cost - summary.final_cost);
		_current_cost = summary.final_cost;

		_total_time_in_ms += logger.get_time_in_ms();
	}

	return rigid_transformation_from_se3(_transformation_se3);
}

bool ICPPointSubset::finished()
{
	double tol = 0.001;
	return (_solve_iteration >= _max_iterations || (_current_tol < tol && _solve_iteration > 15));
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

ml::mat4f iterative_closest_points(std::vector<ml::vec3f> &src, std::vector<ml::vec3f> &dst) 
{
	ceres::Solver::Options options;
	options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
	options.minimizer_type = ceres::MinimizerType::TRUST_REGION;
	options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
	options.max_num_iterations = 50;
	options.logging_type = ceres::LoggingType::SILENT;
	options.minimizer_progress_to_stdout = false;
	options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
	options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
	options.preconditioner_type = ceres::PreconditionerType::JACOBI;

	{
		ICPPointSubset icp(src, dst, options);
		ml::mat4f translation = icp.solve();
	}
	{
		ICPPointSubset icp(src, dst, options);
		ml::mat4f translation = icp.solveTransformDataset();
		return translation;
	}

	//{
	//	// Ceres Solver Iteration: 11, Duration 4s 234ms, 
	//	// Total time: 66s 323ms, 
	//	// Initial cost: 0.413669, Final cost: 0.413423, Termination: 0
	//	std::cout << "line_search_direction_type: bfgs, linear_solver_type: sparse normal cholesky, preconditioner_type: jacobi" << std::endl;
	//	options.line_search_direction_type = ceres::LineSearchDirectionType::BFGS;
	//	options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
	//	options.preconditioner_type = ceres::PreconditionerType::JACOBI;
	//	ICPNN icp(src, dst, options);
	//	ml::mat4f translation = icp.solve();
	//}
	//{
	//	//64s 778ms
	//	// Ceres Solver Iteration: 11, Duration 4s 213ms, 
	//	// Total time: 64s 778ms, 
	//	// Initial cost: 0.414282, Final cost: 0.414047, Termination: 0
	//	std::cout << "line_search_direction_type: lbfgs, linear_solver_type: sparse normal cholesky, preconditioner_type: jacobi" << std::endl;
	//	options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
	//	options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
	//	options.preconditioner_type = ceres::PreconditionerType::JACOBI;
	//	ICPNN icp(src, dst, options);
	//	ml::mat4f translation = icp.solve();
	//}
	//{
	//	// Ceres Solver Iteration: 11, Duration 4s 198ms,
	//	// Total time: 66s 530ms, 
	//	// Initial cost: 0.413603, Final cost: 0.413373, Termination: 0
	//	std::cout << "line_search_direction_type: nonlinear conjugate gradient, linear_solver_type: sparse normal cholesky, preconditioner_type: jacobi" << std::endl;
	//	options.line_search_direction_type = ceres::LineSearchDirectionType::NONLINEAR_CONJUGATE_GRADIENT;
	//	options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
	//	options.preconditioner_type = ceres::PreconditionerType::JACOBI;
	//	ICPNN icp(src, dst, options);
	//	ml::mat4f translation = icp.solve();
	//}
	//{
	//	// Ceres Solver Iteration: 11, Duration 4s 242ms, 
	//	// Total time: 66s 634ms, 
	//	// Initial cost: 0.413723, Final cost: 0.413499, Termination: 0
	//	std::cout << "line_search_direction_type: steepest descent, linear_solver_type: sparse normal cholesky, preconditioner_type: jacobi" << std::endl;
	//	options.line_search_direction_type = ceres::LineSearchDirectionType::STEEPEST_DESCENT;
	//	options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
	//	options.preconditioner_type = ceres::PreconditionerType::JACOBI;
	//	ICPNN icp(src, dst, options);
	//	ml::mat4f translation = icp.solve();
	//}
	//{
	//	// Ceres Solver Iteration: 11, Duration 4s 279ms, 
	//	// Total time: 67s 959ms, 
	//	// Initial cost: 0.413694, Final cost: 0.41347, Termination: 0
	//	std::cout << "line_search_direction_type: lbfgs, linear_solver_type: cgnr, preconditioner_type: jacobi" << std::endl;
	//	options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
	//	options.linear_solver_type = ceres::LinearSolverType::CGNR;
	//	options.preconditioner_type = ceres::PreconditionerType::JACOBI;
	//	ICPNN icp(src, dst, options);
	//	ml::mat4f translation = icp.solve();
	//}
	//{
	//	// Ceres Solver Iteration: 11, Duration 4s 95ms, 
	//	// Total time: 65s 639ms, 
	//	// Initial cost: 0.413611, Final cost: 0.413378, Termination: 0
	//	std::cout << "line_search_direction_type: lbfgs, linear_solver_type: dense normal cholesky, preconditioner_type: jacobi" << std::endl;
	//	options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
	//	options.linear_solver_type = ceres::LinearSolverType::DENSE_NORMAL_CHOLESKY;
	//	options.preconditioner_type = ceres::PreconditionerType::JACOBI;
	//	ICPNN icp(src, dst, options);
	//	ml::mat4f translation = icp.solve();
	//}
	//{
	//	// Ceres Solver Iteration: 11, Duration 4s 169ms, 
	//	// Total time: 66s 476ms, 
	//	// Initial cost: 0.414134, Final cost: 0.413896, Termination: 0
	//	std::cout << "line_search_direction_type: lbfgs, linear_solver_type: dense qr, preconditioner_type: jacobi" << std::endl;
	//	options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
	//	options.linear_solver_type = ceres::LinearSolverType::DENSE_QR;
	//	options.preconditioner_type = ceres::PreconditionerType::JACOBI;
	//	ICPNN icp(src, dst, options);
	//	ml::mat4f translation = icp.solve();
	//}
	//{
	//	// Ceres Solver Iteration: 11, Duration 4s 197ms, 
	//	// Total time: 66s 618ms, 
	//	// Initial cost: 0.413585, Final cost: 0.413343, Termination: 0
	//	std::cout << "line_search_direction_type: lbfgs, linear_solver_type: dense schur, preconditioner_type: jacobi" << std::endl;
	//	options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
	//	options.linear_solver_type = ceres::LinearSolverType::DENSE_SCHUR;
	//	options.preconditioner_type = ceres::PreconditionerType::JACOBI;
	//	ICPNN icp(src, dst, options);
	//	ml::mat4f translation = icp.solve();
	//}
	//{
	//	// 64s 556ms,
	//	// Ceres Solver Iteration: 10, Duration 4s 249ms, 
	//	// Total time: 64s 556ms,
	//	// Initial cost: 0.41424, Final cost: 0.413876, Termination: 0
	//	std::cout << "line_search_direction_type: lbfgs, linear_solver_type: iterative schur, preconditioner_type: jacobi" << std::endl;
	//	options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
	//	options.linear_solver_type = ceres::LinearSolverType::ITERATIVE_SCHUR;
	//	options.preconditioner_type = ceres::PreconditionerType::JACOBI;
	//	ICPNN icp(src, dst, options);
	//	ml::mat4f translation = icp.solve();
	//}
	//{
	//	// Ceres Solver Iteration: 11, Duration 4s 212ms, 
	//	// Total time: 67s 57ms, 
	//	// Initial cost: 0.413714, Final cost: 0.413485, Termination: 0
	//	std::cout << "line_search_direction_type: lbfgs, linear_solver_type: sparse schur, preconditioner_type: jacobi" << std::endl;
	//	options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
	//	options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
	//	options.preconditioner_type = ceres::PreconditionerType::JACOBI;
	//	ICPNN icp(src, dst, options);
	//	ml::mat4f translation = icp.solve();
	//}
	//{
	//	// Ceres Solver Iteration: 1, Duration 1s 816ms, Total time: 3s 630ms, Initial cost: -1, Final cost: -1, Termination: 2
	//	std::cout << "line_search_direction_type: lbfgs, linear_solver_type: sparse normal cholesky, preconditioner_type: cluster jacobi" << std::endl;
	//	options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
	//	options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
	//	options.preconditioner_type = ceres::PreconditionerType::CLUSTER_JACOBI;
	//	ICPNN icp(src, dst, options);
	//	ml::mat4f translation = icp.solve();
	//}
	//{
	//	// Ceres Solver Iteration: 1, Duration 1s 801ms, Total time: 3s 618ms, Initial cost: -1, Final cost: -1, Termination: 2
	//	std::cout << "line_search_direction_type: lbfgs, linear_solver_type: sparse normal cholesky, preconditioner_type: cluster tridiagonal" << std::endl;
	//	options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
	//	options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
	//	options.preconditioner_type = ceres::PreconditionerType::CLUSTER_TRIDIAGONAL;
	//	ICPNN icp(src, dst, options);
	//	ml::mat4f translation = icp.solve();
	//}
	//{
	//	// Ceres Solver Iteration: 11, Duration 4s 204ms, 
	//	// Total time: 66s 585ms, 
	//	// Initial cost: 0.414105, Final cost: 0.413885, Termination: 0
	//	std::cout << "line_search_direction_type: lbfgs, linear_solver_type: sparse normal cholesky, preconditioner_type: identity" << std::endl;
	//	options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
	//	options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
	//	options.preconditioner_type = ceres::PreconditionerType::IDENTITY;
	//	ICPNN icp(src, dst, options);
	//	ml::mat4f translation = icp.solve();
	//}
	//{
	//	// Ceres Solver Iteration: 11, Duration 4s 169ms, Total time: 66s 159ms, Initial cost: 0.413824, Final cost: 0.413588, Termination: 0
	//	std::cout << "line_search_direction_type: lbfgs, linear_solver_type: sparse normal cholesky, preconditioner_type: schur jacobi" << std::endl;
	//	options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
	//	options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
	//	options.preconditioner_type = ceres::PreconditionerType::SCHUR_JACOBI;
	//	ICPNN icp(src, dst, options);
	//	ml::mat4f translation = icp.solve();
	//	return translation;
	//}
}




