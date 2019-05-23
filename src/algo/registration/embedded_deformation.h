#pragma once

#include "mLibInclude.h"
#include "deformation_graph.h"
#include "deformed_mesh.h"
#include "node.h"
#include "algo/file_writer.h"
#include <vector>
#include <ceres/ceres.h>

#include "algo/registration/i_registration.h"

class FindCorrespondecePoint;
namespace ED {

typedef DeformationGraph<Graph, Node> EmbeddedDeformationGraph;
typedef DeformedMesh<Graph, Node> EmbeddedDeformedMesh;
typedef std::vector<ceres::ResidualBlockId> ResidualIds;
typedef std::map<Graph::vertex_descriptor, ResidualIds> VertexResidualIds;
typedef std::map<Graph::edge_descriptor, ResidualIds> EdgeResidualIds;

class EmbeddedDeformation : public IRegistration
{
	Mesh _src;
	Mesh _dst;
	ceres::Solver::Options _options;
	EmbeddedDeformationGraph _deformation_graph;
	std::unique_ptr<EmbeddedDeformedMesh> _deformed_mesh;
	std::vector<int> _fixed_positions;
	std::unique_ptr<FindCorrespondecePoint> _find_correspondence_point;
private:
	bool _with_icp = false;
	double _current_cost = 1.;
	double _last_cost = 2.;
	size_t _solve_iteration = 0;
	size_t _max_iterations = 100;
	long long _total_time_in_ms = 0;
	double a_rigid = 1000.;// 1.;// 1000;
	double a_smooth = 100.;// 0.1;// 100;
	double a_conf = 100.;// 1.;// 100;
	double a_fit = 1.;
	std::shared_ptr<FileWriter> _logger;
private:
	double _k_mean_cost;
	void updateMeanCost();
private:
	double evaluateResidual(ceres::Problem & problem, std::vector<ceres::ResidualBlockId> & residual_ids);
	void evaluateResidual(ceres::Problem & problem,
						  VertexResidualIds & fit_residual_block_ids,
						  EdgeResidualIds & smoot_residual_block_ids,
						  VertexResidualIds & rotation_residual_block_ids,
						  VertexResidualIds & conf_residual_block_ids);
private:
	ceres::ResidualBlockId addPointToPointCostForNode(ceres::Problem &problem, Node & node, ml::vec3f & target_position);
	ceres::ResidualBlockId addPointToPlaneCostForNode(ceres::Problem &problem, Node & node, ml::vec3f & target_position);
	VertexResidualIds addFitCostWithoutICP(ceres::Problem &problem);
	VertexResidualIds addFitCost(ceres::Problem &problem);
	ceres::ResidualBlockId addSmoothCost(ceres::Problem &problem, Node & node_i, Node & node_j);
	EdgeResidualIds addSmoothCost(ceres::Problem &problem);
	ceres::ResidualBlockId addRotationCost(ceres::Problem &problem, Node & node);
	VertexResidualIds addRotationCost(ceres::Problem &problem);
	VertexResidualIds addConfCost(ceres::Problem &problem);
private:
	void printCeresOptions();
public:
	bool finished();
	bool solveIteration() override;
	bool solve() override;
public:
	const Mesh & getSource() override;
	const Mesh & getTarget() override;
	Mesh getDeformedPoints() override;
	Mesh getInverseDeformedPoints() override;
public:
	std::vector<ml::vec3f> getFixedPostions() override;
	std::vector<Edge> getDeformationGraph() override;	
	Mesh getDeformationGraphMesh() override;
public:
	EmbeddedDeformationGraph & getEmbeddedDeformationGraph();
public:
	EmbeddedDeformation(const Mesh& src,
						const Mesh& dst,
						ceres::Solver::Options option,
						unsigned int number_of_deformation_nodes = 1000,
						std::shared_ptr<FileWriter> logger = nullptr);

	EmbeddedDeformation(const Mesh& src,
						const Mesh& dst,
						const EmbeddedDeformationGraph & deformation_graph,
						ceres::Solver::Options option,
						std::shared_ptr<FileWriter> logger = nullptr);
	EmbeddedDeformation(const Mesh& src,
						const Mesh& dst,
						std::vector<int> fixed_positions,
						ceres::Solver::Options option,
						std::shared_ptr<FileWriter> logger);
};


}