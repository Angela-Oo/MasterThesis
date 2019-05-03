#pragma once

#include "../../mLibInclude.h"
#include <vector>
#include <ceres/ceres.h>
#include <functional>
#include "algo/knn.h"
#include "algo/file_writer.h"

ml::mat4f iterative_closest_points(std::vector<ml::vec3f> &src, std::vector<ml::vec3f> &dst);

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
typedef ml::TriMeshf Mesh;

class ICP
{
	Mesh _src;
	Mesh _dst;
	ceres::Solver::Options _options;	
public:
	ICP(const Mesh& src,
		const Mesh& dst,
		ceres::Solver::Options option);

	ml::mat4f solveFixNN(ml::vec6d transformation_se3 = ml::vec6d(0., 0., 0., 0., 0., 0.));
	// Ceres Solver Iteration: 0, Duration 494s 980ms, Total time: 494s 980ms, Initial cost: 0.982676, Final cost: 0.412166, Termination: 0
	ml::mat4f solve();
};

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

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
	std::shared_ptr<FileWriter> _logger;
public:
	ml::mat4f solve();
	ml::mat4f solveIteration();
	bool finished();
public:
	ICPNN(const std::vector<ml::vec3f>& src,
		  const std::vector<ml::vec3f>& dst,
		  ceres::Solver::Options option,
		  std::shared_ptr<FileWriter> logger = nullptr);
};

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

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
