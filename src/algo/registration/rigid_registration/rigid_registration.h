#pragma once

#include "rigid_deformation.h"
#include "rigid_deformed_mesh.h"
#include "algo/registration/i_registration.h"
#include "algo/file_writer.h"
#include "algo/registration/find_corresponding_points/find_corresponding_points.h"
#include "algo/surface_mesh/mesh_definition.h"
#include <ceres/ceres.h>
#include "algo/ceres_iteration_logger.h"
//#include <optional>
#include "boost/optional.hpp"

namespace Registration {

typedef std::vector<ceres::ResidualBlockId> ResidualIds;
typedef std::map<vertex_descriptor, ResidualIds> VertexResidualIds;

class RigidRegistration : public IRegistration
{
public:
	typedef typename RigidDeformation RigidRegistration::Deformation;
private:
	SurfaceMesh _source;
	SurfaceMesh _target;
	ceres::Solver::Options _ceres_options;
	boost::optional<RigidDeformation> _previouse_deformation;
	boost::optional<SurfaceMesh> _true_source;
	RigidDeformation _deformation;
	Point _global_position;
	CeresLogger _ceres_logger;
	std::unique_ptr<FindCorrespondingPoints> _find_correspondence_point;
	std::unique_ptr<RigidDeformedMesh> _rigid_deformed_mesh;
	std::vector<vertex_descriptor> _set_of_vertices_to_use;
private:
	bool _deformed_points_returns_deformed_previouse_frame = true;
	bool _with_icp = true;
	double _current_cost = 1.;
	double _last_cost = 2.;
	size_t _solve_iteration = 0;
	RegistrationOptions _options;
private:
	void evaluateResidual(ceres::Problem & problem,
						  std::map<vertex_descriptor, ResidualIds> & fit_residual_block_ids,
						  std::unique_ptr<CeresIterationLoggerGuard> & logger);
private:
	ceres::ResidualBlockId addPointToPointCost(ceres::Problem &problem, const Point & source_point, vertex_descriptor target_vertex);
	ceres::ResidualBlockId addPointToPlaneCost(ceres::Problem &problem, const Point & source_point, vertex_descriptor target_vertex);
	std::map<vertex_descriptor, ResidualIds> addFitCost(ceres::Problem &problem, std::unique_ptr<CeresIterationLoggerGuard> & logger);
	std::map<vertex_descriptor, ResidualIds> addFitCostWithoutICP(ceres::Problem &problem);
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
	RigidDeformation getRigidDeformation();
	RigidDeformation getDeformation();
	bool shouldBeSavedAsImage();
private:
	void init();
public:
	RigidRegistration(const SurfaceMesh & source,
					  const SurfaceMesh & target,
					  ceres::Solver::Options ceres_option,
					  RegistrationOptions options,
					  std::shared_ptr<FileWriter> logger = nullptr);

	RigidRegistration(const SurfaceMesh & source,
					  const SurfaceMesh & target,
					  RigidDeformation rigid_deformation,
					  ceres::Solver::Options ceres_option,
					  RegistrationOptions options,
					  std::shared_ptr<FileWriter> logger = nullptr);

	RigidRegistration(const SurfaceMesh & source,
					  const SurfaceMesh & target,
					  const SurfaceMesh & previous_mesh,
					  RigidDeformation rigid_deformation,
					  ceres::Solver::Options ceres_option,
					  RegistrationOptions options,
					  std::shared_ptr<FileWriter> logger = nullptr);
};

}