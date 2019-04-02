#pragma once

#include "as_rigid_as_possible.h"
#include "embedded_deformation.h"
#include <vector>
#include "knn.h"
//
//class NonRigidICP
//{
//	std::vector<ml::vec3f> _src;
//	std::vector<ml::vec3f> _dst;
//	ceres::Solver::Options _options;
//	ml::vec6d _transformation_se3 = ml::vec6d(0., 0., 0., 0., 0., 0.);
//	size_t _solve_iteration = 0;
//	double _current_cost = 1.;
//	double _current_tol = 1.;
//	long long _total_time_in_ms = 0;
//	size_t _max_iterations = 20;
//	KNN _nn_search;
//public:
//	NonRigidICP(const std::vector<ml::vec3f>& src,
//				const std::vector<ml::vec3f>& dst,
//				ceres::Solver::Options option);
//public:
//	ml::mat4f solve();
//public:
//	ml::mat4f solveIteration();
//	bool finished();
//};