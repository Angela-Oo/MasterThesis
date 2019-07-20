#pragma once

#include "mLibInclude.h"
#include "algo/registration/deformation_graph/deformation_graph.h"
#include "algo/registration/deformation_graph/deformed_mesh.h"
#include "ed_deformation.h"
#include "algo/file_writer.h"
#include <vector>
#include <ceres/ceres.h>

#include "algo/registration/i_registration.h"

#include "algo/registration/find_corresponding_points/find_corresponding_points.h"

#include "algo/ceres_iteration_logger.h"

namespace ED {

typedef std::vector<ceres::ResidualBlockId> ResidualIds;
typedef std::map<vertex_descriptor, ResidualIds> VertexResidualIds;
typedef std::map<edge_descriptor, ResidualIds> EdgeResidualIds;

class EmbeddedDeformation : public INonRigidRegistration
{
	SurfaceMesh _src;
	SurfaceMesh _dst;
	ceres::Solver::Options _options;
	DG::DeformationGraph _deformation_graph;
	std::unique_ptr<DG::DeformedMesh> _deformed_mesh;
	std::vector<vertex_descriptor> _fixed_positions;
	std::unique_ptr<FindCorrespondingPoints> _find_correspondence_point;

	CeresLogger _ceres_logger;
private:
	bool _with_icp = false;
	double _current_cost = 1.;
	double _last_cost = 2.;
	size_t _solve_iteration = 0;
	size_t _max_iterations = 100;
	bool _evaluate_residuals;
public:
	double a_rigid;
	double a_smooth;
	double a_conf;
	double a_fit;
	double _find_max_distance = 0.5;
	double _find_max_angle_deviation = 45.;
private:
	//double _k_mean_cost;
	//void updateMeanCost();
private:
	void evaluateResidual(ceres::Problem & problem,
						  VertexResidualIds & fit_residual_block_ids,
						  EdgeResidualIds & smoot_residual_block_ids,
						  VertexResidualIds & rotation_residual_block_ids,
						  VertexResidualIds & conf_residual_block_ids);
private:
	ceres::ResidualBlockId addPointToPointCostForNode(ceres::Problem &problem, vertex_descriptor node, const Point & target_point);
	ceres::ResidualBlockId addPointToPlaneCostForNode(ceres::Problem &problem, vertex_descriptor node, const Point & target_point, const Vector & target_normal);
	VertexResidualIds addFitCostWithoutICP(ceres::Problem &problem);
	VertexResidualIds addFitCost(ceres::Problem &problem);
	EdgeResidualIds addSmoothCost(ceres::Problem &problem);
	VertexResidualIds addRotationCost(ceres::Problem &problem);
	VertexResidualIds addConfCost(ceres::Problem &problem);
private:
	void setParameters();
public:
	bool finished();
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
	void setRigidDeformation(const RigidDeformation & rigid_deformation);
	const DG::DeformationGraph & getDeformationGraph() override;
	std::vector<Point> getFixedPostions() override;
public:
	// without icp
	EmbeddedDeformation(const SurfaceMesh& src,
						const SurfaceMesh& dst,
						std::vector<vertex_descriptor> fixed_positions,
						const DG::DeformationGraph & deformation_graph,
						ceres::Solver::Options option,
						bool evaluate_residuals,
						std::shared_ptr<FileWriter> logger);

	// with icp
	EmbeddedDeformation(const SurfaceMesh& src,
						const SurfaceMesh& dst,
						const DG::DeformationGraph & deformation_graph,
						ceres::Solver::Options option,
						bool evaluate_residuals = false,
						std::shared_ptr<FileWriter> logger = nullptr);
};




DG::PositionAndDeformation createGlobalDeformationFromRigidDeformation(const RigidDeformation & rigid_deformation);


std::unique_ptr<EmbeddedDeformation> createEmbeddedDeformation(const SurfaceMesh& src,
										  const SurfaceMesh& dst,
										  std::vector<vertex_descriptor> fixed_positions,
										  ceres::Solver::Options option,
										  const RegistrationOptions & registration_options,
										  std::shared_ptr<FileWriter> logger = nullptr);

std::unique_ptr<EmbeddedDeformation> createEmbeddedDeformation(const SurfaceMesh& src,
										  const SurfaceMesh& dst,
										  ceres::Solver::Options option,
										  const RegistrationOptions & registration_options,
										  std::shared_ptr<FileWriter> logger = nullptr);


std::unique_ptr<EmbeddedDeformation> createEmbeddedDeformation(const SurfaceMesh& src,
										  const SurfaceMesh& dst,
										  const RigidDeformation & rigid_deformation,
										  ceres::Solver::Options option,
										  const RegistrationOptions & registration_options,
										  std::shared_ptr<FileWriter> logger = nullptr);


std::unique_ptr<EmbeddedDeformation> createEmbeddedDeformation(const SurfaceMesh& src,
										  const SurfaceMesh& dst,
										  const RigidDeformation & rigid_deformation,
										  const DG::DeformationGraph & deformation_graph,
										  ceres::Solver::Options option,
										  const RegistrationOptions & registration_options,
										  std::shared_ptr<FileWriter> logger = nullptr);


}