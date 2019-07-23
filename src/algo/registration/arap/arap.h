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

namespace Registration {

typedef std::vector<ceres::ResidualBlockId> ResidualIds;
typedef std::map<vertex_descriptor, ResidualIds> VertexResidualIds;
typedef std::map<edge_descriptor, ResidualIds> EdgeResidualIds;

class AsRigidAsPossible : public INonRigidRegistration
{
public:
	using Deformation = DeformationGraph;
private:
	SurfaceMesh _src;
	SurfaceMesh _dst;
	ceres::Solver::Options _options;
	DeformationGraph _deformation_graph;
	std::unique_ptr<DeformedMesh> _deformed_mesh;
	std::vector<vertex_descriptor> _set_of_vertices_to_use;
	std::vector<vertex_descriptor> _fixed_positions;
	std::unique_ptr<FindCorrespondingPoints> _find_correspondence_point;
private:
	RegistrationOptions _registration_options;
	bool _with_icp = true;
	double _current_cost = 1.;
	double _last_cost = 2.;
	size_t _solve_iteration = 0;
private:
	CeresLogger _ceres_logger;
	std::knuth_b _rand_engine;
private:
	void init();
	bool random_bool_with_prob(double prob);
	void evaluateResidual(ceres::Problem & problem,
						  std::map<vertex_descriptor, ResidualIds> & fit_residual_block_ids,
						  std::map<edge_descriptor, ResidualIds> & arap_residual_block_ids,
						  std::unique_ptr<CeresIterationLoggerGuard>& logger);
private:
	ResidualIds addPointToPointCostForNode(ceres::Problem &problem, vertex_descriptor node, const Point & target_point);
	ResidualIds addPointToPlaneCostForNode(ceres::Problem &problem, vertex_descriptor node, const Point & target_point, const Vector & target_normal);
	bool useVertex(vertex_descriptor & v);
	bool addFitCostVertex(ceres::Problem & problem, vertex_descriptor & v, VertexResidualIds &residual_ids);
	std::map<vertex_descriptor, ResidualIds> addFitCost(ceres::Problem &problem, std::unique_ptr<CeresIterationLoggerGuard>& logger);
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
	void setRigidDeformation(const RigidDeformation & rigid_deformation) override;
	const DeformationGraph & getDeformationGraph() override;
	const DeformationGraph & getDeformation();
	std::vector<Point> getFixedPostions() override;
	bool shouldBeSavedAsImage() override;
public:
	// without icp
	AsRigidAsPossible(const SurfaceMesh& src,
					  const SurfaceMesh& dst,
					  std::vector<vertex_descriptor> fixed_positions,
					  const DeformationGraph & deformation_graph,
					  ceres::Solver::Options option,
					  const RegistrationOptions & registration_options,
					  std::shared_ptr<FileWriter> logger = nullptr);

	// with icp
	AsRigidAsPossible(const SurfaceMesh& src,
					  const SurfaceMesh& dst,
					  const DeformationGraph & deformation_graph,
					  ceres::Solver::Options option,
					  const RegistrationOptions & registration_options,
					  std::shared_ptr<FileWriter> logger = nullptr);
};


//-----------------------------------------------------------------------------


PositionAndDeformation createGlobalDeformationFromRigidDeformation(const RigidDeformation & rigid_deformation);


std::unique_ptr<AsRigidAsPossible> createAsRigidAsPossible(const SurfaceMesh& src,
										  const SurfaceMesh& dst,
										  std::vector<vertex_descriptor> fixed_positions,
										  ceres::Solver::Options option,
										  const RegistrationOptions & registration_options,
										  std::shared_ptr<FileWriter> logger);

std::unique_ptr<AsRigidAsPossible> createAsRigidAsPossible(const SurfaceMesh& src,
										  const SurfaceMesh& dst,
										  ceres::Solver::Options option,
										  const RegistrationOptions & registration_options,
										  std::shared_ptr<FileWriter> logger);


std::unique_ptr<AsRigidAsPossible> createAsRigidAsPossible(const SurfaceMesh& src,
										  const SurfaceMesh& dst,
										  const RigidDeformation & rigid_deformation,
										  ceres::Solver::Options option,
										  const RegistrationOptions & registration_options,
										  std::shared_ptr<FileWriter> logger);


std::unique_ptr<AsRigidAsPossible> createAsRigidAsPossible(const SurfaceMesh& src,
										  const SurfaceMesh& dst,
										  const RigidDeformation & rigid_deformation,
										  const DeformationGraph & deformation_graph,
										  ceres::Solver::Options option,
										  const RegistrationOptions & registration_options,
										  std::shared_ptr<FileWriter> logger);



}