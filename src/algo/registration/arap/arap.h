#pragma once

#include "i_arap_fit_cost.h"
#include "arap_deformation.h"
#include "i_arap_smooth_cost.h"
#include "algo/registration/interface/i_registration.h"
#include "algo/registration/interface/registration_options.h"
#include "mesh/mesh_definition.h"
#include "algo/registration/deformation_graph/deformation_graph.h"
#include "algo/registration/deformation_graph/deformed_mesh.h"
#include "algo/registration/deformation_graph/deformation_graph_deform_mesh.h"
#include "algo/registration/util/ceres_iteration_logger.h"
#include "util/file_writer.h"
#include "util/ceres_include.h"
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
	SurfaceMesh _source;
	SurfaceMesh _target;
	ceres::Solver::Options _ceres_options;
	DeformationGraph<ARAPDeformation> _deformation_graph;
	std::unique_ptr<DeformedMesh<Deformation>> _deformed_mesh;
	std::vector<vertex_descriptor> _selected_subset;
private:
	std::unique_ptr<IAsRigidAsPossibleFitCost> _fit_cost;
	std::unique_ptr<IAsRigidAsPossibleSmoothCost> _smooth_cost;
private:
	RegistrationOptions _options;
	double _current_cost = 1.;
	double _last_cost = 2.;
	size_t _solve_iteration = 0;
private:
	CeresLogger _ceres_logger;
private:
	void init();
	void updateSmoothFactor();
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
	void setDeformation(const Deformation & deformation_graph);
	const DeformationGraph<ARAPDeformation> & getDeformation();
	std::vector<Point> getFixedPositions() override;
	std::pair<bool, std::string> shouldBeSavedAsImage() override;
public:
	// without icp
	AsRigidAsPossible(const SurfaceMesh& source,
					  const SurfaceMesh& target,
					  std::vector<vertex_descriptor> fixed_positions,
					  const Deformation & deformation_graph,
					  const RegistrationOptions & options,
					  std::shared_ptr<FileWriter> logger = nullptr);

	// generate deformation graph
	AsRigidAsPossible(const SurfaceMesh& source,
					  const SurfaceMesh& target,
					  const RegistrationOptions & options,
					  std::shared_ptr<FileWriter> logger = nullptr);

	// with icp
	AsRigidAsPossible(const SurfaceMesh& source,
					  const SurfaceMesh& target,
					  const Deformation & deformation_graph,
					  const RegistrationOptions & options,
					  std::shared_ptr<FileWriter> logger = nullptr);

	AsRigidAsPossible(const SurfaceMesh& source,
					  const SurfaceMesh& target,
					  const SurfaceMesh& previous_mesh,
					  const Deformation & deformation_graph,
					  const RegistrationOptions & options,
					  std::shared_ptr<FileWriter> logger = nullptr);
};


//-----------------------------------------------------------------------------


ARAPDeformation createGlobalDeformationFromRigidDeformation(const RigidDeformation & rigid_deformation);


std::unique_ptr<AsRigidAsPossible> createAsRigidAsPossible(const SurfaceMesh& src,
										  const SurfaceMesh& dst,
										  std::vector<vertex_descriptor> fixed_positions,
										  const RegistrationOptions & options,
										  std::shared_ptr<FileWriter> logger);

}