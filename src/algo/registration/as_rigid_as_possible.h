#pragma once

#include "mLibInclude.h"
#include "as_rigid_as_possible_node.h"
#include "deformation_graph.h"
#include "deformed_mesh.h"
#include "algo/file_writer.h"
#include "algo/mesh_knn.h"
#include <ceres/ceres.h>
#include "i_registration.h"
#include "find_correspondece_point.h"

typedef ml::TriMeshf Mesh;
typedef DeformationGraph<ARAPGraph, ARAPNode> ARAPDeformationGraph;
typedef DeformedMesh<ARAPGraph, ARAPNode> ARAPDeformedMesh;


class AsRigidAsPossible : public IRegistration
{
private:
	Mesh _src;
	Mesh _dst;
	ceres::Solver::Options _options;
	ARAPDeformationGraph _deformation_graph;
	std::unique_ptr<ARAPDeformedMesh> _deformed_mesh;
	std::vector<int> _fixed_positions;
	//TriMeshKNN _nn_search;
	FindCorrespondecePoint _find_correspondence_point;
	bool _with_icp = true;
private:
	std::vector<ceres::ResidualBlockId> _fit_point_to_point_residuals_ids;
	std::vector<ceres::ResidualBlockId> _fit_point_to_plane_residuals_ids;
	std::vector<ceres::ResidualBlockId> _smooth_residuals_ids;
	std::vector<ceres::ResidualBlockId> _conf_residuals_ids;
	ARAPGradient _gradient;
private:
	double _current_cost = 1.;
	double _last_cost = 2.;
	size_t _solve_iteration = 0;
	size_t _max_iterations = 200;
	long long _total_time_in_ms = 0;
	double a_smooth = 100.;// 0.1;// 100;
	double a_conf = 100.;// 1.;// 100;
	double a_fit = 0.1;
	std::shared_ptr<FileWriter> _logger;
private:
	void printCeresOptions();
	std::vector<NodeGradient> gradientOfResidualBlock(ceres::Problem & problem, std::vector<ceres::ResidualBlockId> & residual_block_ids);
private:
	void addConfCost(ceres::Problem &problem);
	void addFitCost(ceres::Problem &problem);
	void addFitCostWithoutICP(ceres::Problem &problem);
	void addAsRigidAsPossibleCost(ceres::Problem &problem);
public:
	bool finished() override;
	bool solveIteration() override;	
	bool solve() override;

	ARAPGradient gradient() override;
public:
	const Mesh & getSource() override;
	const Mesh & getTarget() override;
	Mesh getDeformedPoints() override;
	Mesh getInverseDeformedPoints() override;
public:
	std::pair<std::vector<ml::vec3f>, std::vector<ml::vec3f>> getDeformationGraph() override;
	Mesh getDeformationGraphMesh() override;
	std::vector<ml::vec3f> getFixedPostions() override;
public:
	ARAPDeformationGraph & getARAPDeformationGraph();
public:
	// without icp
	AsRigidAsPossible(const Mesh& src,
								const Mesh& dst,
								std::vector<int> fixed_positions,
								ceres::Solver::Options option,
								std::shared_ptr<FileWriter> logger = nullptr);
	// with icp
	AsRigidAsPossible(const Mesh& src,
								const Mesh& dst,
								ceres::Solver::Options option,
								unsigned int number_of_deformation_nodes = 1000,
								std::shared_ptr<FileWriter> logger = nullptr);
	// with icp but init with passed deformation graph
	AsRigidAsPossible(const Mesh& src,
								const Mesh& dst,
								const ARAPDeformationGraph & deformation_graph,
								ceres::Solver::Options option,
								std::shared_ptr<FileWriter> logger = nullptr);
};