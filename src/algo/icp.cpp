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

#include "knn.h"


ml::mat4f rigid_transformation_from_se3(ml::vec6d & rotation_translation)
{
	double rotation_matrix[9];
	ceres::AngleAxisToRotationMatrix(rotation_translation.array, rotation_matrix);

	// rotation
	//ml::vec3d angle_axis = { rotation_translation[0], rotation_translation[1], rotation_translation[2] };
	ml::mat4f rotation = ml::mat4f::identity();
	rotation(0, 0) = rotation_matrix[0];
	rotation(1, 0) = rotation_matrix[1];
	rotation(2, 0) = rotation_matrix[2];
	rotation(0, 1) = rotation_matrix[3];
	rotation(1, 1) = rotation_matrix[4];
	rotation(2, 1) = rotation_matrix[5];
	rotation(0, 2) = rotation_matrix[6];
	rotation(1, 2) = rotation_matrix[7];
	rotation(2, 2) = rotation_matrix[8];
	//if (angle_axis != ml::vec3f::origin)
	//	rotation = ml::mat4f::rotation(angle_axis.getNormalized(), angle_axis.length());

	// translation
	ml::vec3d translation_vector = { rotation_translation[3], rotation_translation[4], rotation_translation[5] };
	ml::mat4f translation = ml::mat4f::translation(translation_vector);

	return translation * rotation;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

ml::vec6d solve_icp(const std::vector<ml::vec3f>& src,
					const std::vector<ml::vec3f>& dst,
					const ceres::Solver::Options& options,
					ml::vec6d initial_transformation_se3,
					ceres::Solver::Summary & summary)
{
	KNN nn_search(dst);
	ceres::Problem problem;
	for (int i = 0; i < src.size(); ++i) {
		unsigned int index = nn_search.nearest_index(src[i]);
		ceres::CostFunction* cost_function = PointToPointErrorSE3::Create(dst[index], src[i]);
		problem.AddResidualBlock(cost_function, NULL, initial_transformation_se3.array);
	}
	ceres::Solve(options, &problem, &summary);
	return initial_transformation_se3;
}



//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------


ICPLogIterationGuard::ICPLogIterationGuard(const ceres::Solver::Summary& summary, long long total_time_in_ms, size_t iteration)
	: _summary(summary)
	, _total_time_in_ms(total_time_in_ms)
	, _iteration(iteration)
{
	_start_time = std::chrono::system_clock::now();

}

long long ICPLogIterationGuard::get_time_in_ms()
{
	auto end_time = std::chrono::system_clock::now();
	return std::chrono::duration_cast<std::chrono::milliseconds>(end_time - _start_time).count();
}

ICPLogIterationGuard::~ICPLogIterationGuard()
{
	//std::cout << "Final report:\n" << _summary.BriefReport() << std::endl << std::endl;
	auto elapse = get_time_in_ms();
	_total_time_in_ms += elapse;

	auto time_to_string = [](long long time_in_ms) {
		long long s = floor(static_cast<double>(time_in_ms) / 1000.);
		long long ms = time_in_ms - (s * 1000);
		return std::to_string(s) + "s " + std::to_string(ms) + "ms";
	};
	
	
	std::cout << "Ceres Solver Iteration: " << _iteration << ", Duration " << time_to_string(elapse) << ", Total time: " << time_to_string(_total_time_in_ms) 
		<< ", Initial cost: "<< _summary.initial_cost << ", Final cost: " << _summary.final_cost << ", Termination: " << _summary.termination_type << std::endl << std::endl;
}


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------


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


ml::vec6d ICP::solve_transformation(ml::vec6d transformation_se3)
{
	ceres::Solver::Summary summary;
	ICPLogIterationGuard log_guard(summary);
	transformation_se3 = solve_icp(_src, _dst, _options, transformation_se3, summary);
	return transformation_se3;
}


ml::mat4f ICP::solve(ml::vec6d transformation_se3)
{
	transformation_se3 = solve_transformation(transformation_se3);
	return rigid_transformation_from_se3(transformation_se3);
}


ml::mat4f ICP::solveNN()
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
		transformation = solveIterationTransformDataset();
	}
	return transformation;
}

ml::mat4f ICPNN::solvetest()
{
	ml::mat4f transformation = ml::mat4f::identity();
	while (!finished()) {
		transformation = solveIteration();
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



ml::mat4f ICPNN::solveIterationTransformDataset() // working
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



ml::mat4f ICPNN::solveIterationUsePointSubset()
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

		// 
		_transformation_se3 += transformation;

		// why necessary
		ml::mat4f transform_matrix = rigid_transformation_from_se3(transformation);
		std::for_each(_src.begin(), _src.end(), [&](ml::vec3f & p) { p = transform_matrix * p; });


		_current_tol = abs(_current_cost - summary.final_cost);
		_current_cost = summary.final_cost;

		_total_time_in_ms += logger.get_time_in_ms();
	}

	return rigid_transformation_from_se3(_transformation_se3);
}


bool ICPNN::finished()
{
	double tol = 0.001;
	return !(_solve_iteration < _max_iterations && _current_tol > tol);
}



//ml::mat4f matrix4d_to_mat4f_tmp(Eigen::Matrix4d mat)
//{
//	return ml::mat4f(mat(0, 0), mat(0, 1), mat(0, 2), mat(0, 3),
//					 mat(1, 0), mat(1, 1), mat(1, 2), mat(1, 3),
//					 mat(2, 0), mat(2, 1), mat(2, 2), mat(2, 3),
//					 mat(3, 0), mat(3, 1), mat(3, 2), mat(3, 3));
//}
//

ceres::Solver::Options getSolveOptions() {
	// Set a few options
	ceres::Solver::Options options;

#ifdef _WIN32
	options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
	options.linear_solver_type = ceres::ITERATIVE_SCHUR;
	options.preconditioner_type = ceres::SCHUR_JACOBI;
#else
	//options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
#endif // _WIN32

	//If you are solving small to medium sized problems, consider setting Solver::Options::use_explicit_schur_complement to true, it can result in a substantial performance boost.
	options.use_explicit_schur_complement = true;
	options.max_num_iterations = 50;

	return options;
}






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
		ICPNN icp(src, dst, options);
		ml::mat4f translation = icp.solve();
	}
	{
		ICPNN icp(src, dst, options);
		ml::mat4f translation = icp.solvetest();
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




