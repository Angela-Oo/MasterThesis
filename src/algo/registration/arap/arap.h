#pragma once

#include "mLibInclude.h"
#include "arap_deformation.h"
#include "algo/file_writer.h"
#include <ceres/ceres.h>
#include "algo/registration/i_registration.h"
#include "algo/surface_mesh/mesh_definition.h"
#include "algo/registration/deformation_graph/deformation_graph.h"
#include "algo/registration/deformation_graph/deformed_mesh.h"

#include "algo/registration/find_corresponding_points/find_corresponding_points.h"

#include "algo/ceres_iteration_logger.h"

namespace ARAP {



typedef std::vector<ceres::ResidualBlockId> ResidualIds;
typedef std::map<vertex_descriptor, ResidualIds> VertexResidualIds;
typedef std::map<edge_descriptor, ResidualIds> EdgeResidualIds;

class AsRigidAsPossible : public IRegistration
{
public:

private:
	SurfaceMesh _src;
	SurfaceMesh _dst;
	ceres::Solver::Options _options;
	DG::DeformationGraph _deformation_graph;
	std::unique_ptr<DG::DeformedMesh> _deformed_mesh;
	std::vector<vertex_descriptor> _fixed_positions;
	std::unique_ptr<FindCorrespondingPoints> _find_correspondence_point;
private:
	bool _ignore_deformation_graph_border_vertices;
	bool _evaluate_residuals;
	bool _with_icp = true;
	double _current_cost = 1.;
	double _last_cost = 2.;
	size_t _solve_iteration = 0;
	size_t _max_iterations = 50;
private:
	double a_smooth;
	double a_conf;
	double a_fit;
	double _find_max_distance;
	double _find_max_angle_deviation;
	CeresLogger _ceres_logger;
private:
	void setParameters();
	void printCeresOptions();
	void evaluateResidual(ceres::Problem & problem,
							std::map<vertex_descriptor, ResidualIds> & fit_residual_block_ids,
							std::map<edge_descriptor, ResidualIds> & arap_residual_block_ids,
							std::map<vertex_descriptor, ResidualIds> & conf_residual_block_ids);
private:
	ceres::ResidualBlockId addPointToPointCostForNode(ceres::Problem &problem, vertex_descriptor node, const Point & target_point);
	ceres::ResidualBlockId addPointToPlaneCostForNode(ceres::Problem &problem, vertex_descriptor node, const Point & target_point, const Vector & target_normal);
	std::map<vertex_descriptor, ResidualIds> addFitCost(ceres::Problem &problem);
	std::map<vertex_descriptor, ResidualIds> addFitCostWithoutICP(ceres::Problem &problem);
	std::map<edge_descriptor, ResidualIds> addAsRigidAsPossibleCost(ceres::Problem &problem);
	std::map<vertex_descriptor, ResidualIds> addConfCost(ceres::Problem &problem);
public:
	bool finished() override;
	bool solveIteration() override;
	size_t currentIteration() override;
	bool solve() override;
public:
	const SurfaceMesh & getSource() override;
	const SurfaceMesh & getTarget() override;
	SurfaceMesh getDeformedPoints() override;
	SurfaceMesh getInverseDeformedPoints() override;
	SurfaceMesh getDeformationGraphMesh() override;
public:
	const DG::DeformationGraph & getDeformationGraph() override;
	std::vector<Point> getFixedPostions() override;
public:
	// without icp
	AsRigidAsPossible(const SurfaceMesh& src,
					  const SurfaceMesh& dst,
					  std::vector<vertex_descriptor> fixed_positions,
					  ceres::Solver::Options option,
					  bool evaluate_residuals = false,
					  std::shared_ptr<FileWriter> logger = nullptr);
	// with icp
	AsRigidAsPossible(const SurfaceMesh& src,
					  const SurfaceMesh& dst,
					  ceres::Solver::Options option,
					  double deformation_graph_edge_length = 0.05,
					  bool evaluate_residuals = false,
					  std::shared_ptr<FileWriter> logger = nullptr);
	// with icp but init with passed deformation graph
	AsRigidAsPossible(const SurfaceMesh& src,
					  const SurfaceMesh& dst,
					  const DG::DeformationGraph & deformation_graph,
					  ceres::Solver::Options option,
					  bool evaluate_residuals = false,
					  std::shared_ptr<FileWriter> logger = nullptr);
};	

}