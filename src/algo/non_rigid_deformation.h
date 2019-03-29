#pragma once

#include "as_rigid_as_possible.h"
#include "embedded_deformation.h"
//
//#include "../mLibInclude.h"
//#include <vector>
//#include <ceres/ceres.h>
//
//
//class AsRigidAsPossible
//{
//	std::vector<ml::vec3f> _src;
//	std::vector<ml::vec3f> _dst;
//	ceres::Solver::Options _options;
//	std::vector<ml::vec3d> _rotations;
//	std::vector<ml::vec3d> _solved_points;
//private:
//	void solvePositions();
//	void solveRotation();
//public:
//	std::vector<ml::vec3f> solve();
//	// expect src and dst points to match at the same array position
//	AsRigidAsPossible(const std::vector<ml::vec3f>& src,
//					  const std::vector<ml::vec3f>& dst,
//					  ceres::Solver::Options option);
//};
//
//
//class EmbeddedDeformation
//{
//	std::vector<ml::vec3f> _src;
//	std::vector<ml::vec3f> _dst;
//	ceres::Solver::Options _options;
//	std::vector<ml::mat3d> _matrix;
//	std::vector<ml::vec3d> _solved_points;
//private:
//	void solvePositions();
//	void solveMatrix();
//public:
//	// expect src and dst points to match at the same array position
//	EmbeddedDeformation(const std::vector<ml::vec3f>& src,
//					  const std::vector<ml::vec3f>& dst,
//					  ceres::Solver::Options option);
//
//	std::vector<ml::vec3f> solve();
//};