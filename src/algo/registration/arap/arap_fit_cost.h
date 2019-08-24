#pragma once

#include "i_arap_fit_cost.h"
#include "mesh/mesh_definition.h"
#include "algo/registration/interface/i_registration.h"
#include "arap_deformation.h"
#include "algo/registration/deformation_graph/deformation_graph.h"
#include "algo/registration/deformation_graph/deformed_mesh.h"
#include "algo/registration/find_corresponding_points/find_corresponding_points.h"
#include "algo/registration/util/ceres_iteration_logger.h"
#include <ceres/ceres.h>
#include <ceres/problem.h>
#include "algo/registration/util/ceres_iteration_logger.h"

namespace Registration
{

class AsRigidAsPossibleFitCost : public IAsRigidAsPossibleFitCost
{
private:
	std::map<edge_descriptor, std::vector<ceres::ResidualBlockId>> _fit_residual_ids;
	std::unique_ptr<FindCorrespondingPoints> _find_correspondence_point;
	std::vector<vertex_descriptor> _subset_of_vertices_to_fit;
	RegistrationOptions _options;
private:
	bool useVertex(vertex_descriptor & v, DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh);
	bool addFitCostVertex(ceres::Problem & problem, 
						  vertex_descriptor & v, 
						  DeformationGraph<ARAPDeformation> & deformation_graph,
						  DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh, 
						  VertexResidualIds &residual_ids);
public:
	void evaluateResiduals(ceres::Problem &problem, SurfaceMesh & mesh, CeresIterationLoggerGuard & logger);
	VertexResidualIds addFitCost(ceres::Problem &problem,
								 DeformationGraph<ARAPDeformation> & deformation_graph,
								 DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
								 std::unique_ptr<CeresIterationLoggerGuard>& logger) override;
public:
	AsRigidAsPossibleFitCost(SurfaceMesh & target,
							 std::vector<vertex_descriptor> & subset_of_vertices_to_fit,
							 RegistrationOptions options);
};



}