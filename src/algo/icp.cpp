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


ICP::ICP(const std::vector<ml::vec3f>& src,
		 const std::vector<ml::vec3f>& dst,
		 ceres::Solver::Options option)
	: _src(src)
	, _dst(dst)
	, _options(option)
{}



void ICP::printOptions()
{
	std::cout << "\nCeres Solver" << std::endl;
	std::cout << "Ceres preconditioner type: " << _options.preconditioner_type << std::endl;
	std::cout << "Ceres linear algebra type: " << _options.sparse_linear_algebra_library_type << std::endl;
	std::cout << "Ceres linear solver type: " << _options.linear_solver_type << std::endl;
}

ml::vec6d ICP::solve_transformation(ml::vec6d transformation_se3)
{
	printOptions();
	auto start_time = std::chrono::system_clock::now();

	KNN nn_search(_dst);
	ceres::Problem problem;
	for (int i = 0; i < _src.size(); ++i) {
		unsigned int index = nn_search.nearest_index(_src[i]);
		ceres::CostFunction* cost_function = PointToPointErrorSE3::Create(_dst[index], _src[i]);
		problem.AddResidualBlock(cost_function, NULL, transformation_se3.array);
	}
	ceres::Solver::Summary summary;
	ceres::Solve(_options, &problem, &summary);

	std::cout << "Final report:\n" << summary.BriefReport() << std::endl;// FullReport();
	auto end_time = std::chrono::system_clock::now();
	auto elapse = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
	std::cout << "\nCeres Solver duration " << elapse << "s\n" << std::endl;

	return transformation_se3;
}


ml::mat4f ICP::solve(ml::vec6d transformation_se3)
{
	transformation_se3 = solve_transformation(transformation_se3);
	return rigid_transformation_from_se3(transformation_se3);
}


ml::mat4f ICP::solveNN2()
{
	printOptions();
	auto start_time = std::chrono::system_clock::now();

	ml::vec6d translation(0., 0., 0., 0., 0., 0.);
	ml::mat4f test = ml::mat4f::identity();

	double tol = 0.001;
	double last_cost = 1.;
	double cur_tol = last_cost;

	KNN nn_search(_dst);
	for (int j = 0; (cur_tol > tol) && j < 50; j++) {		
		ml::vec6d rotation_translation(0., 0., 0., 0., 0., 0.);
		ceres::Problem problem;
		for (int i = 0; i < _src.size(); ++i) {
			unsigned int index = nn_search.nearest_index(_src[i]);
			ceres::CostFunction* cost_function = PointToPointErrorSE3::Create(_dst[index], _src[i]);
			problem.AddResidualBlock(cost_function, NULL, rotation_translation.array);
		}
		ceres::Solver::Summary summary;
		ceres::Solve(_options, &problem, &summary);

		std::cout << "Final report:\n" << summary.BriefReport() << std::endl;// FullReport();		

		ml::mat4f transform_matrix =  rigid_transformation_from_se3(rotation_translation);
		std::for_each(_src.begin(), _src.end(), [&](ml::vec3f & p) { p = transform_matrix * p; });
		translation += rotation_translation;
		test = transform_matrix * test;

		cur_tol = abs(last_cost - summary.final_cost);
		last_cost = summary.final_cost;
	}

	auto end_time = std::chrono::system_clock::now();
	auto elapse = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
	std::cout << "\nCeres Solver duration " << elapse << "s\n" << std::endl;

	return rigid_transformation_from_se3(translation);
}


ml::mat4f ICP::solveNN()
{
	printOptions();
	auto start_time = std::chrono::system_clock::now();

	KNN nn_search(_dst);
	ml::vec6d rotation_translation(0., 0., 0., 0., 0., 0.);
	ceres::Problem problem;
	for (int i = 0; i < _src.size(); ++i) {
		ceres::CostFunction* cost_function = PointToPointsErrorSE3NNSearch::Create(_src[i], std::bind(&KNN::nearest_f, &nn_search, std::placeholders::_1));
		problem.AddResidualBlock(cost_function, NULL, rotation_translation.array);
	}
	ceres::Solver::Summary summary;
	ceres::Solve(_options, &problem, &summary);

	std::cout << "Final report:\n" << summary.BriefReport() << std::endl;// FullReport();

	auto end_time = std::chrono::system_clock::now();
	auto elapse = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
	std::cout << "\nCeres Solver duration " << elapse << "s\n" << std::endl;

	return rigid_transformation_from_se3(rotation_translation);
}



ml::mat4f ICP::solveNN3()
{
	printOptions();
	auto start_time = std::chrono::system_clock::now();

	ml::vec6d translation(0., 0., 0., 0., 0., 0.);
	ml::mat4f test = ml::mat4f::identity();

	double tol = 0.001;
	double last_cost = 1.;
	double cur_tol = last_cost;

	ml::vec6d rotation_translation(0., 0., 0., 0., 0., 0.);
	for (int j = 0; (cur_tol > tol) && j < 10; j++) {

		std::vector<ml::vec3f> sub_set_src;
		std::vector<ml::vec3f> sub_set_dst;

		int step = std::pow(6 - j, 2);
		int step_size = std::max(step, 1);
		for (int i = 0; i < _src.size(); i += step_size)
		{
			sub_set_src.push_back(_src[i]);
			if(i <_dst.size())
				sub_set_dst.push_back(_dst[i]);
		}
		KNN nn_search(sub_set_dst);
				
		ceres::Problem problem;
		for (int i = 0; i < sub_set_src.size(); ++i) {
			unsigned int index = nn_search.nearest_index(sub_set_src[i]);
			ceres::CostFunction* cost_function = PointToPointErrorSE3::Create(sub_set_dst[index], sub_set_src[i]);
			problem.AddResidualBlock(cost_function, NULL, rotation_translation.array);
		}
		ceres::Solver::Summary summary;
		ceres::Solve(_options, &problem, &summary);

		std::cout << "Final report:\n" << summary.BriefReport() << std::endl;// FullReport();		

		//ml::mat4f transform_matrix = rigid_transformation_from_se3(rotation_translation);
		//std::for_each(_src.begin(), _src.end(), [&](ml::vec3f & p) { p = transform_matrix * p; });
		translation += rotation_translation;
		//test = transform_matrix * test;

		cur_tol = abs(last_cost - summary.final_cost);
		last_cost = summary.final_cost;
	}

	auto end_time = std::chrono::system_clock::now();
	auto elapse = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
	std::cout << "\nCeres Solver duration " << elapse << "s\n" << std::endl;

	return rigid_transformation_from_se3(rotation_translation);
}












ml::vec6d solve_icp(const std::vector<ml::vec3f>& src,
					const std::vector<ml::vec3f>& dst,
					ceres::Solver::Options options,
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



ICPNN::ICPNN(const std::vector<ml::vec3f>& src,
			 const std::vector<ml::vec3f>& dst,
			 ceres::Solver::Options option)
	: _src(src)
	, _dst(dst)
	, _options(option)
{}



void ICPNN::printOptions()
{
	std::cout << "\nCeres Solver" << std::endl;
	std::cout << "Ceres preconditioner type: " << _options.preconditioner_type << std::endl;
	std::cout << "Ceres linear algebra type: " << _options.sparse_linear_algebra_library_type << std::endl;
	std::cout << "Ceres linear solver type: " << _options.linear_solver_type << std::endl;
}


ml::mat4f ICPNN::solve()
{
	ceres::Solver::Summary summary;
	_transformation_se3 = solve_icp(_src, _dst, _options, _transformation_se3, summary);
	return rigid_transformation_from_se3(_transformation_se3);
}

ml::mat4f ICPNN::solveNN() // not working ?? why
{
	if(_solve_iteration == 0)
		printOptions();
	
	double tol = 0.001;
	if (_solve_iteration < 50 && _current_tol > tol) {
		auto start_time = std::chrono::system_clock::now();

		_solve_iteration++;

		ceres::Solver::Summary summary;
		_transformation_se3 = solve_icp(_src, _dst, _options, _transformation_se3, summary);
		std::cout << "Final report:\n" << summary.BriefReport() << std::endl;

		_current_tol = abs(_current_cost - summary.final_cost);
		_current_cost = summary.final_cost;

		auto end_time = std::chrono::system_clock::now();
		auto elapse = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
		_total_time_in_ms += elapse;
		std::cout << "Ceres Solver Iteration: " << _solve_iteration << " duration " << elapse << "ms " << " total time: " << _total_time_in_ms << std::endl << std::endl;
	}

	return rigid_transformation_from_se3(_transformation_se3);
}



ml::mat4f ICPNN::solveNN2() // working
{
	if (_solve_iteration == 0)
		printOptions();
	

	double tol = 0.001;
	if (_solve_iteration < 50 && _current_tol > tol) {
		auto start_time = std::chrono::system_clock::now();
		_solve_iteration++;


		ml::vec6d rotation_translation(0., 0., 0., 0., 0., 0.);

		ceres::Solver::Summary summary;
		rotation_translation = solve_icp(_src, _dst, _options, rotation_translation, summary);
		
		ml::mat4f transform_matrix = rigid_transformation_from_se3(rotation_translation);
		std::for_each(_src.begin(), _src.end(), [&](ml::vec3f & p) { p = transform_matrix * p; });
		_transformation_se3 += rotation_translation;


		// infos
		std::cout << "Final report:\n" << summary.BriefReport() << std::endl << std::endl;

		_current_tol = abs(_current_cost - summary.final_cost);
		_current_cost = summary.final_cost;

		auto end_time = std::chrono::system_clock::now();
		auto elapse = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
		_total_time_in_ms += elapse;
		std::cout << "Ceres Solver Iteration: " << _solve_iteration << " duration " << elapse << "ms " << " total time: " << _total_time_in_ms << std::endl << std::endl;
	}
	return rigid_transformation_from_se3(_transformation_se3);
}



ml::mat4f ICPNN::solveNN3()
{
	if (_solve_iteration == 0)
		printOptions();
	
	double tol = 0.001;

	if (_solve_iteration < 10 && _current_tol > tol) {
		auto start_time = std::chrono::system_clock::now();

		_solve_iteration++;

		std::vector<ml::vec3f> sub_set_src;
		std::vector<ml::vec3f> sub_set_dst;

		int step = std::pow(6 - _solve_iteration, 2);
		int step_size = std::max(step, 1);

		for (int i = 0; i < _src.size(); i += step_size) {
			sub_set_src.push_back(_src[i]);
		}
		//ml::mat4f transform_matrix = rigid_transformation_from_se3(_transformation_se3);
		//std::for_each(_src.begin(), _src.end(), [&](ml::vec3f & p) { p = transform_matrix * p; });
		for (int i = 0; i < _dst.size(); i += step_size) {
			sub_set_dst.push_back(_dst[i]);
		}

		ml::vec6d transformation(0., 0., 0., 0., 0., 0.);
		ceres::Solver::Summary summary;
		transformation = solve_icp(sub_set_src, sub_set_dst, _options, transformation, summary);
		_transformation_se3 += transformation;

		// why necessary
		ml::mat4f transform_matrix = rigid_transformation_from_se3(transformation);
		std::for_each(_src.begin(), _src.end(), [&](ml::vec3f & p) { p = transform_matrix * p; });

		// info
		std::cout << "Final report:\n" << summary.BriefReport() << std::endl;// FullReport();
				
		_current_tol = abs(_current_cost - summary.final_cost);
		_current_cost = summary.final_cost;

		auto end_time = std::chrono::system_clock::now();
		auto elapse = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
		_total_time_in_ms += elapse;
		std::cout << "Ceres Solver Iteration: " << _solve_iteration << " duration " << elapse << "ms " << " total time: " << _total_time_in_ms << std::endl << std::endl;
	}
	
	return rigid_transformation_from_se3(_transformation_se3);
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






ml::mat4f iterative_closest_points(std::vector<ml::vec3f> &src, std::vector<ml::vec3f> &dst, const ceres::Solver::Options & options) 
{
	std::cout << "\nCeres Solver" << std::endl;
	std::cout << "Ceres preconditioner type: " << options.preconditioner_type << std::endl;
	std::cout << "Ceres linear algebra type: " << options.sparse_linear_algebra_library_type << std::endl;
	std::cout << "Ceres linear solver type: " << options.linear_solver_type << std::endl;

	auto start_time = std::chrono::system_clock::now();

	KNN nn_search(dst);
	
	ml::vec6d rotation_translation(0., 0., 0., 0., 0., 0.);
	ceres::Problem problem;
	for (int i = 0; i < src.size(); ++i) {
		// first viewpoint : dstcloud, fixed		// second viewpoint: srcCloud, moves
		//if (dst[i] != ml::vec3f::origin && src[i] != ml::vec3f::origin) {
		ceres::CostFunction* cost_function = PointToPointsErrorSE3NNSearch::Create(src[i], std::bind(&KNN::nearest_f, &nn_search, std::placeholders::_1));
		// std::bind(&KNNBruteForce::nearest_f, &nn_search, std::placeholders::_1));
		problem.AddResidualBlock(cost_function, NULL, rotation_translation.array);
		//}
	}
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << "Final report:\n" << summary.BriefReport() << std::endl;// FullReport();

	auto end_time = std::chrono::system_clock::now();
	auto elapse = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
	std::cout << "\nCeres Solver duration " << elapse << "s\n" << std::endl;

	return rigid_transformation_from_se3(rotation_translation);
}




ml::mat4f pointToPointSE3(std::vector<ml::vec3f> &src, std::vector<ml::vec3f> &dst)
{
	{
		ceres::Solver::Options options;
		options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
		options.linear_solver_type = ceres::ITERATIVE_SCHUR;
		options.preconditioner_type = ceres::SCHUR_JACOBI;
		options.max_num_iterations = 50;

		ml::mat4f translation = iterative_closest_points(src, dst, options);
		return translation;
	}
	{
		ceres::Solver::Options options;
		options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
		options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
		options.linear_solver_type = ceres::LinearSolverType::CGNR;
		options.max_num_iterations = 50;
		ml::mat4f translation = iterative_closest_points(src, dst, options);
	}

	{
		ceres::Solver::Options options;
		options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
		options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
		options.max_num_iterations = 50;
		ml::mat4f translation = iterative_closest_points(src, dst, options);
		return translation;
	}


}