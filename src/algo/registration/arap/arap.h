#pragma once

#include "mLibInclude.h"
#include "arap_node.h"
#include "algo/file_writer.h"
#include <ceres/ceres.h>
#include "algo/registration/i_registration.h"
#include "algo/surface_mesh/mesh_definition.h"
#include "algo/deformation_graph/deformation_graph_cgal_mesh.h"

#include "algo/registration/find_corresponding_points/find_corresponding_points.h"

namespace ARAP {

class ITestRegistration
{
public:
	virtual bool finished() = 0;
	virtual bool solveIteration() = 0;
	virtual bool solve() = 0;
public:
	virtual const SurfaceMesh & getSource() = 0;
	virtual const SurfaceMesh & getTarget() = 0;
	//virtual SurfaceMesh getDeformedPoints() = 0;
	//virtual SurfaceMesh getInverseDeformedPoints() = 0;
public:
	virtual std::vector<Point> getFixedPostions() { return std::vector<Point>(); }
	//virtual std::vector<Edge> getDeformationGraph() { return std::vector<Edge>(); }
	//virtual Mesh getDeformationGraphMesh() { return Mesh(); };
public:
	virtual ~ITestRegistration() = default;
};



typedef std::vector<ceres::ResidualBlockId> ResidualIds;
typedef std::map<vertex_descriptor, ResidualIds> VertexResidualIds;
typedef std::map<edge_descriptor, ResidualIds> EdgeResidualIds;

	class AsRigidAsPossible : public ITestRegistration
	{
	public:

	private:
		SurfaceMesh _src;
		SurfaceMesh _dst;
		ceres::Solver::Options _options;
		DeformationGraph::DeformationGraphCgalMesh _deformation_graph;
		std::unique_ptr<DeformationGraph::DeformedMesh> _deformed_mesh;
		std::vector<vertex_descriptor> _fixed_positions;
		std::unique_ptr<FindCorrespondingPoints> _find_correspondence_point;
		bool _with_icp = true;
	private:
		double _current_cost = 1.;
		double _last_cost = 2.;
		size_t _solve_iteration = 0;
		size_t _max_iterations = 100;
		long long _total_time_in_ms = 0;
		double a_smooth = 50.;// 10.;// 0.1;// 100;
		double a_conf = 100.;// 1.;// 100;
		double a_fit = 10.;
		const double _find_max_distance = 0.5;
		const double _find_max_angle_deviation = 45.;
		std::shared_ptr<FileWriter> _logger;
	private:
		double _fit_mean_cost;
		double _smooth_mean_cost;
		double _k_mean_cost;
	private:
		//void updateMeanCost();
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

		//std::map<std::string, std::map<vertex_index, std::vector<double>>> residuals() override;
	public:
		const SurfaceMesh & getSource() override;
		const SurfaceMesh & getTarget() override;
		//Mesh getDeformedPoints() override;
		//Mesh getInverseDeformedPoints() override;
	public:
		//std::vector<Edge> getDeformationGraph() override;
		//Mesh getDeformationGraphMesh() override;
		std::vector<Point> getFixedPostions() override;
	public:
		//DeformationGraph & getARAPDeformationGraph();
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
						  const DeformationGraph::DeformationGraphCgalMesh & deformation_graph,
						  ceres::Solver::Options option,
						  std::shared_ptr<FileWriter> logger = nullptr);
	};	

}