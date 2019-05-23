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

typedef std::vector<ceres::ResidualBlockId> ResidualIds;
typedef std::map<ARAPGraph::vertex_descriptor, ResidualIds> ARAPVertexResidualIds;
typedef std::map<ARAPGraph::edge_descriptor, ResidualIds> ARAPEdgeResidualIds;

class AsRigidAsPossible : public IRegistration
{
public:

private:
	Mesh _src;
	Mesh _dst;
	ceres::Solver::Options _options;
	ARAPDeformationGraph _deformation_graph;
	std::unique_ptr<ARAPDeformedMesh> _deformed_mesh;
	std::vector<int> _fixed_positions;
	FindCorrespondecePoint _find_correspondence_point;
	bool _with_icp = true;
private:
	double _current_cost = 1.;
	double _last_cost = 2.;
	size_t _solve_iteration = 0;
	size_t _max_iterations = 200;
	long long _total_time_in_ms = 0;
	double a_smooth = 100.;// 0.1;// 100;
	double a_conf = 100.;// 1.;// 100;
	double a_fit = 10.0;
	std::shared_ptr<FileWriter> _logger;
private:
	void printCeresOptions();
	double evaluateResidual(ceres::Problem & problem, std::vector<ceres::ResidualBlockId> & residual_ids);
	void evaluateResidual(ceres::Problem & problem,
						  ARAPVertexResidualIds & fit_residual_block_ids,
						  ARAPEdgeResidualIds & arap_residual_block_ids,
						  ARAPVertexResidualIds & conf_residual_block_ids);
private:	
	ceres::ResidualBlockId addPointToPointCostForNode(ceres::Problem &problem, ARAPNode & node, ml::vec3f target_position);
	ceres::ResidualBlockId addPointToPlaneCostForNode(ceres::Problem &problem, ARAPNode & node, ml::vec3f target_position);
	ARAPVertexResidualIds addFitCost(ceres::Problem &problem);
	ARAPVertexResidualIds addFitCostWithoutICP(ceres::Problem &problem);
	ARAPEdgeResidualIds addAsRigidAsPossibleCost(ceres::Problem &problem);
	ARAPVertexResidualIds addConfCost(ceres::Problem &problem);
public:
	bool finished() override;
	bool solveIteration() override;	
	bool solve() override;

	//std::map<std::string, std::map<vertex_index, std::vector<double>>> residuals() override;
public:
	const Mesh & getSource() override;
	const Mesh & getTarget() override;
	Mesh getDeformedPoints() override;
	Mesh getInverseDeformedPoints() override;
public:
	std::vector<Edge> getDeformationGraph() override;
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