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
#include "i_ed_smooth_cost.h"
#include "i_ed_fit_cost.h"

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
	std::vector<vertex_descriptor> _selected_subset;
	CeresLogger _ceres_logger;
private:
	std::unique_ptr<IEmbeddedDeformationFitCost> _fit_cost;
	std::unique_ptr<IEmbeddedDeformationSmoothCost> _smooth_cost;
private:
	double _current_cost = 1.;
	double _last_cost = 2.;
	size_t _solve_iteration = 0;
public:
	double a_rigid{ 100. };
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
	// with icp
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
	// with icp
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

//
//std::unique_ptr<EmbeddedDeformation> createEmbeddedDeformation(const SurfaceMesh& src,
//															   const SurfaceMesh& dst,
//															   const RigidDeformation & rigid_deformation,
//															   const RegistrationOptions & registration_options,
//															   std::shared_ptr<FileWriter> logger = nullptr);
//
//
//std::unique_ptr<EmbeddedDeformation> createEmbeddedDeformation(const SurfaceMesh& src,
//															   const SurfaceMesh& dst,
//															   const RigidDeformation & rigid_deformation,
//															   const DeformationGraph<EDDeformation> & deformation_graph,
//															   const RegistrationOptions & registration_options,
//															   std::shared_ptr<FileWriter> logger = nullptr);



}
