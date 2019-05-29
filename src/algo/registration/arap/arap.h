#pragma once

#include "mLibInclude.h"
#include "arap_node.h"
#include "algo/file_writer.h"
#include <ceres/ceres.h>
#include "algo/registration/i_registration.h"
#include "algo/surface_mesh/mesh_definition.h"
#include "algo/registration/deformation_graph/deformation_graph_cgal_mesh.h"

#include "algo/registration/find_corresponding_points/find_corresponding_points.h"

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
	DG::DeformationGraphCgalMesh _deformation_graph;
	std::unique_ptr<DG::DeformedMesh> _deformed_mesh;
	std::vector<vertex_descriptor> _fixed_positions;
	std::unique_ptr<FindCorrespondingPoints> _find_correspondence_point;
private:
	bool _with_icp = true;
	double _current_cost = 1.;
	double _last_cost = 2.;
	size_t _solve_iteration = 0;
	size_t _max_iterations = 100;
	long long _total_time_in_ms = 0;
private:
	double a_smooth = 100.;// 10.;// 0.1;// 100;
	double a_conf = 100.;// 1.;// 100;
	double a_fit = 10.;
	const double _find_max_distance = 0.5;
	const double _find_max_angle_deviation = 45.;
	std::shared_ptr<FileWriter> _logger;
private:
	void printCeresOptions();
	double evaluateResidual(ceres::Problem & problem, std::vector<ceres::ResidualBlockId> & residual_ids);
	void evaluateResidual(ceres::Problem & problem,
							std::map<vertex_descriptor, ResidualIds> & fit_residual_block_ids,
							std::map<edge_descriptor, ResidualIds> & arap_residual_block_ids,
							std::map<vertex_descriptor, ResidualIds> & conf_residual_block_ids);
private:
	ceres::ResidualBlockId addPointToPointCostForNode(ceres::Problem &problem, vertex_descriptor node, Point target_position);
	ceres::ResidualBlockId addPointToPlaneCostForNode(ceres::Problem &problem, vertex_descriptor node, Point target_position);
	std::map<vertex_descriptor, ResidualIds> addFitCost(ceres::Problem &problem);
	std::map<vertex_descriptor, ResidualIds> addFitCostWithoutICP(ceres::Problem &problem);
	std::map<edge_descriptor, ResidualIds> addAsRigidAsPossibleCost(ceres::Problem &problem);
	std::map<vertex_descriptor, ResidualIds> addConfCost(ceres::Problem &problem);
public:
	bool finished() override;
	bool solveIteration() override;
	bool solve() override;
public:
	const SurfaceMesh & getSource() override;
	const SurfaceMesh & getTarget() override;
	SurfaceMesh getDeformedPoints() override;
	//Mesh getInverseDeformedPoints() override;
	SurfaceMesh getDeformationGraphMesh() override;
public:
	const DG::DeformationGraphCgalMesh & getDeformationGraph() override;
	std::vector<Point> getFixedPostions() override;
public:
	// without icp
	AsRigidAsPossible(const SurfaceMesh& src,
						const SurfaceMesh& dst,
						std::vector<vertex_descriptor> fixed_positions,
						ceres::Solver::Options option,
						std::shared_ptr<FileWriter> logger = nullptr);
	// with icp
	AsRigidAsPossible(const SurfaceMesh& src,
						const SurfaceMesh& dst,
						ceres::Solver::Options option,
						unsigned int number_of_deformation_nodes = 1000,
						std::shared_ptr<FileWriter> logger = nullptr);
	// with icp but init with passed deformation graph
	AsRigidAsPossible(const SurfaceMesh& src,
						const SurfaceMesh& dst,
						const DG::DeformationGraphCgalMesh & deformation_graph,
						ceres::Solver::Options option,
						std::shared_ptr<FileWriter> logger = nullptr);
};	

}