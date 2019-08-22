#pragma once

#include "arap_deformation.h"
#include "i_arap_fit_cost.h"
#include "util/file_writer.h"
#include "algo/registration/interface/i_registration.h"
#include "mesh/mesh_definition.h"

#include "algo/registration/deformation_graph/deformation_graph.h"
#include "algo/registration/deformation_graph/deformed_mesh.h"
#include "algo/registration/deformation_graph/deformation_graph_deform_mesh.h"
#include "algo/registration/find_corresponding_points/find_corresponding_points.h"
#include "algo/registration/util/ceres_iteration_logger.h"
#include <ceres/ceres.h>
#include <memory>

namespace Registration {

typedef std::vector<ceres::ResidualBlockId> ResidualIds;
typedef std::map<vertex_descriptor, ResidualIds> VertexResidualIds;
typedef std::map<edge_descriptor, ResidualIds> EdgeResidualIds;

class AsRigidAsPossible : public INonRigidRegistration
{
public:
	using PositionDeformation = ARAPDeformation;
	using Deformation = DeformationGraph<PositionDeformation>;
	using DeformMesh = DeformationGraphDeformMesh<typename Deformation>;
private:
	SurfaceMesh _src;
	SurfaceMesh _dst;
	ceres::Solver::Options _options;
	DeformationGraph<ARAPDeformation> _deformation_graph;
	std::unique_ptr<DeformedMesh<Deformation>> _deformed_mesh;
private:
	std::unique_ptr<IAsRigidAsPossibleFitCost> _fit_cost;
private:
	RegistrationOptions _registration_options;
	bool _with_icp = true;
	double _current_cost = 1.;
	double _last_cost = 2.;
	size_t _solve_iteration = 0;
private:
	CeresLogger _ceres_logger;
private:
	std::vector<vertex_descriptor> subsetOfVerticesToFit();
	void init();
	void evaluateResidual(ceres::Problem & problem,
						  std::map<edge_descriptor, ResidualIds> & arap_residual_block_ids,
						  std::unique_ptr<CeresIterationLoggerGuard>& logger);
private:
	std::map<edge_descriptor, ResidualIds> addAsRigidAsPossibleCost(ceres::Problem &problem);
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
	void setDeformation(const Deformation & deformation_graph);
	const DeformationGraph<ARAPDeformation> & getDeformation();
	std::vector<Point> getFixedPostions() override;
	bool shouldBeSavedAsImage() override;
public:
	// without icp
	AsRigidAsPossible(const SurfaceMesh& src,
					  const SurfaceMesh& dst,
					  std::vector<vertex_descriptor> fixed_positions,
					  const Deformation & deformation_graph,
					  ceres::Solver::Options option,
					  const RegistrationOptions & registration_options,
					  std::shared_ptr<FileWriter> logger = nullptr);

	// generate deformation graph
	AsRigidAsPossible(const SurfaceMesh& src,
					  const SurfaceMesh& dst,
					  ceres::Solver::Options option,
					  const RegistrationOptions & registration_options,
					  std::shared_ptr<FileWriter> logger = nullptr);

	// with icp
	AsRigidAsPossible(const SurfaceMesh& src,
					  const SurfaceMesh& dst,
					  const Deformation & deformation_graph,
					  ceres::Solver::Options option,
					  const RegistrationOptions & registration_options,
					  std::shared_ptr<FileWriter> logger = nullptr);
};


//-----------------------------------------------------------------------------


ARAPDeformation createGlobalDeformationFromRigidDeformation(const RigidDeformation & rigid_deformation);


std::unique_ptr<AsRigidAsPossible> createAsRigidAsPossible(const SurfaceMesh& src,
										  const SurfaceMesh& dst,
										  std::vector<vertex_descriptor> fixed_positions,
										  ceres::Solver::Options option,
										  const RegistrationOptions & registration_options,
										  std::shared_ptr<FileWriter> logger);

}