#pragma once

#include "ed_deformation.h"
#include "algo/registration/deformation_graph/deformation_graph.h"
#include "algo/registration/deformation_graph/deformed_mesh.h"
#include "algo/registration/deformation_graph/deformation_graph_deform_mesh.h"
#include "algo/registration/interface/i_registration.h"
#include "algo/registration/interface/registration_options.h"
#include "algo/registration/find_corresponding_points/find_corresponding_points.h"
#include "algo/registration/util/ceres_iteration_logger.h"
#include "util/file_writer.h"
#include <vector>
#include "util/ceres_include.h"

namespace Registration {

typedef std::vector<ceres::ResidualBlockId> ResidualIds;
typedef std::map<vertex_descriptor, ResidualIds> VertexResidualIds;
typedef std::map<edge_descriptor, ResidualIds> EdgeResidualIds;

class EmbeddedDeformation : public INonRigidRegistration
{
public:
	using PositionDeformation = EDDeformation;
	using Deformation = DeformationGraph<PositionDeformation>;
	using DeformMesh = DeformationGraphDeformMesh<typename Deformation>;
private:
	SurfaceMesh _source;
	SurfaceMesh _target;
	RegistrationOptions _options;
	DeformationGraph<EDDeformation> _deformation_graph;
	std::unique_ptr<DeformedMesh<Deformation>> _deformed_mesh;
	std::vector<vertex_descriptor> _fixed_positions;
	std::unique_ptr<FindCorrespondingPoints> _find_correspondence_point;
	CeresLogger _ceres_logger;
private:
	bool _with_icp = false;
	double _current_cost = 1.;
	double _last_cost = 2.;
	size_t _solve_iteration = 0;
	size_t _max_iterations = 100;
public:
	double a_rigid;
	double a_smooth;
	double a_conf;
	double a_fit;
	double _find_max_distance = 0.5;
	double _find_max_angle_deviation = 45.;
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
	bool finished() override;
	bool solveIteration() override;
	double currentError() override;
	size_t currentIteration() override;
	bool solve() override;
public:
	const SurfaceMesh & getSource() override;
	const SurfaceMesh & getTarget() override;
	SurfaceMesh getDeformedPoints() override;
	SurfaceMesh getInverseDeformedPoints() override;
	SurfaceMesh getDeformationGraphMesh() override;
public:
	void setRigidDeformation(const RigidDeformation & rigid_deformation) override;
	const DeformationGraph<EDDeformation> & getDeformation();
	void setDeformation(const Deformation & deformation_graph);
	std::vector<Point> getFixedPositions() override;
	std::pair<bool, std::string> shouldBeSavedAsImage() override;
public:
	// without icp
	EmbeddedDeformation(const SurfaceMesh& source,
						const SurfaceMesh& target,
						std::vector<vertex_descriptor> fixed_positions,
						const DeformationGraph<EDDeformation> & deformation_graph,
						const RegistrationOptions & options,
						std::shared_ptr<FileWriter> logger);

	EmbeddedDeformation(const SurfaceMesh& source,
						const SurfaceMesh& target,
						const RegistrationOptions & options,
						std::shared_ptr<FileWriter> logger);
	// with icp
	EmbeddedDeformation(const SurfaceMesh& source,
						const SurfaceMesh& target,
						const DeformationGraph<EDDeformation> & deformation_graph,
						const RegistrationOptions & options,
						std::shared_ptr<FileWriter> logger = nullptr);

	EmbeddedDeformation(const SurfaceMesh& source,
						const SurfaceMesh& target,
						const SurfaceMesh& previous_mesh,
						const DeformationGraph<EDDeformation> & deformation_graph,
						const RegistrationOptions & options,
						std::shared_ptr<FileWriter> logger = nullptr);
};




EDDeformation createGlobalEDDeformationFromRigidDeformation(const RigidDeformation & rigid_deformation);


std::unique_ptr<EmbeddedDeformation> createEmbeddedDeformation(const SurfaceMesh& src,
															   const SurfaceMesh& dst,
															   std::vector<vertex_descriptor> fixed_positions,
															   const RegistrationOptions & registration_options,
															   std::shared_ptr<FileWriter> logger = nullptr);

std::unique_ptr<EmbeddedDeformation> createEmbeddedDeformation(const SurfaceMesh& src,
															   const SurfaceMesh& dst,
															   const RegistrationOptions & registration_options,
															   std::shared_ptr<FileWriter> logger = nullptr);


std::unique_ptr<EmbeddedDeformation> createEmbeddedDeformation(const SurfaceMesh& src,
															   const SurfaceMesh& dst,
															   const RigidDeformation & rigid_deformation,
															   const RegistrationOptions & registration_options,
															   std::shared_ptr<FileWriter> logger = nullptr);


std::unique_ptr<EmbeddedDeformation> createEmbeddedDeformation(const SurfaceMesh& src,
															   const SurfaceMesh& dst,
															   const RigidDeformation & rigid_deformation,
															   const DeformationGraph<EDDeformation> & deformation_graph,
															   const RegistrationOptions & registration_options,
															   std::shared_ptr<FileWriter> logger = nullptr);



}