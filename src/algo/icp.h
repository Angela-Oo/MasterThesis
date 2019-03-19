#pragma once


#include "../mLibInclude.h"
#include <vector>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <functional>
#include "knn.h"

ml::mat4f iterative_closest_points(std::vector<ml::vec3f> &src, std::vector<ml::vec3f> &dst);
//
//ml::vec6d solve_icp(const std::vector<ml::vec3f>& src,
//					const std::vector<ml::vec3f>& dst,
//					const ceres::Solver::Options& options,
//					ml::vec6d initial_transformation_se3,
//					ceres::Solver::Summary & summary);



class ICPLogIterationGuard
{
	std::chrono::time_point<std::chrono::system_clock> _start_time;
	const ceres::Solver::Summary& _summary;
	long long _total_time_in_ms;
	size_t _iteration;
public:
	long long get_time_in_ms();
	ICPLogIterationGuard(const ceres::Solver::Summary& summary, long long total_time_in_ms = 0, size_t iteration = 0);
	~ICPLogIterationGuard();
};



class ICP
{
	std::vector<ml::vec3f> _src;
	std::vector<ml::vec3f> _dst;
	ceres::Solver::Options _options;	
public:
	ICP(const std::vector<ml::vec3f>& src,
		const std::vector<ml::vec3f>& dst,
		ceres::Solver::Options option);

	ml::vec6d solve_transformation(ml::vec6d transformation_se3 = ml::vec6d(0., 0., 0., 0., 0., 0.));
	ml::mat4f solve(ml::vec6d transformation_se3 = ml::vec6d(0., 0., 0., 0., 0., 0.));
	// Ceres Solver Iteration: 0, Duration 494s 980ms, Total time: 494s 980ms, Initial cost: 0.982676, Final cost: 0.412166, Termination: 0
	ml::mat4f solveNN();
};





class ICPNN
{
	std::vector<ml::vec3f> _src;
	std::vector<ml::vec3f> _dst;
	ceres::Solver::Options _options;
	ml::vec6d _transformation_se3 = ml::vec6d(0., 0., 0., 0., 0., 0.);
	size_t _solve_iteration = 0;
	double _current_cost = 1.;
	double _current_tol = 1.;
	long long _total_time_in_ms = 0;
	size_t _max_iterations = 20;
	KNN _nn_search;
public:
	ICPNN(const std::vector<ml::vec3f>& src,
		  const std::vector<ml::vec3f>& dst,
		  ceres::Solver::Options option);
public:
	ml::mat4f solve();
	ml::mat4f solveTransformDataset();
public:
	ml::mat4f solveIteration();
	// 12 iterations 157s 929ms (last episode 9s 561ms 
	ml::mat4f solveIterationTransformDataset();
	bool finished();
};



class ICPPointSubset
{
	std::vector<ml::vec3f> _src;
	std::vector<ml::vec3f> _dst;
	ceres::Solver::Options _options;
	ml::vec6d _transformation_se3 = ml::vec6d(0., 0., 0., 0., 0., 0.);
	size_t _solve_iteration = 0;
	double _current_cost = 1.;
	double _current_tol = 1.;
	long long _total_time_in_ms = 0;
	size_t _max_iterations = 20;
	KNN _nn_search;
public:
	ICPPointSubset(const std::vector<ml::vec3f>& src,
				   const std::vector<ml::vec3f>& dst,
				   ceres::Solver::Options option);
public:
	ml::mat4f solve();
	ml::mat4f solveTransformDataset();
public:
	ml::mat4f solveIteration();
	ml::mat4f solveIterationTransformDataset();
	bool finished();
};
