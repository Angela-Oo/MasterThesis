#pragma once
#include "i_registration.h"
#include "algo/file_writer.h"
#include "algo/registration/find_corresponding_points/find_corresponding_points.h"
#include "algo/surface_mesh/mesh_definition.h"
#include <ceres/ceres.h>

class RigidDeformation
{
private:
	ml::vec3d _r; // rotation matrix in angle axis
	ml::vec3d _t; // translation vector	
public:
	double * r() { return (&_r)->getData(); }
	double * t() { return (&_t)->getData(); }
public:
	Matrix rotation() const;
	Vector translation() const;
public:
	Point deformPoint(const Point & point) const;
	Vector deformNormal(const Vector & normal) const;
public:
	RigidDeformation();
};


class RigidDeformedMesh
{
private:
	const RigidDeformation & _deformation;
	SurfaceMesh _mesh;
public:
	SurfaceMesh deformPoints();
public:
	RigidDeformedMesh(const SurfaceMesh & mesh, const RigidDeformation & deformation_graph);
};

typedef std::vector<ceres::ResidualBlockId> ResidualIds;
typedef std::map<vertex_descriptor, ResidualIds> VertexResidualIds;

class RigidRegistration : public IRegistration
{
private:
	//std::unique_ptr<ICPNN> _icp_nn;
	SurfaceMesh _source;
	SurfaceMesh _target;
	ceres::Solver::Options _options;
	RigidDeformation _deformation;
	std::shared_ptr<FileWriter> _logger;
	std::unique_ptr<FindCorrespondingPoints> _find_correspondence_point;
	std::unique_ptr<RigidDeformedMesh> _rigid_deformed_mesh;
private:
	bool _with_icp = true;
	double _current_cost = 1.;
	double _last_cost = 2.;
	size_t _solve_iteration = 0;
	size_t _max_iterations = 100;
	long long _total_time_in_ms = 0;
private:
	void evaluateResidual(ceres::Problem & problem,
						  std::map<vertex_descriptor, ResidualIds> & fit_residual_block_ids);
private:
	ceres::ResidualBlockId addPointToPointCost(ceres::Problem &problem, const Point & source_point, const Point & target_position);
	ceres::ResidualBlockId addPointToPlaneCost(ceres::Problem &problem, const Point & source_point, const Vector & source_normal, const Point & target_position);
	std::map<vertex_descriptor, ResidualIds> addFitCost(ceres::Problem &problem);
	std::map<vertex_descriptor, ResidualIds> addFitCostSubSet(ceres::Problem &problem);
	std::map<vertex_descriptor, ResidualIds> addFitCostWithoutICP(ceres::Problem &problem);
public:
	bool finished() override;
	bool solveIteration() override;
	bool solve() override;
public:
	const SurfaceMesh & getSource() override;
	const SurfaceMesh & getTarget() override;
	SurfaceMesh getDeformedPoints() override;
	SurfaceMesh getDeformationGraphMesh() override;
	//SurfaceMesh getInverseDeformedPoints() override;
public:
	RigidRegistration(const SurfaceMesh & points_a,
					  const SurfaceMesh & points_b,
					  ceres::Solver::Options option,
					  std::shared_ptr<FileWriter> logger = nullptr);
};

