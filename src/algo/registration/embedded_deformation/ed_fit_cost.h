#pragma once

#include "i_ed_fit_cost.h"
#include "mesh/mesh_definition.h"
#include "algo/registration/interface/i_registration.h"
#include "algo/registration/interface/registration_options.h"
#include "algo/registration/find_corresponding_points/find_corresponding_points.h"

namespace Registration
{

class EmbeddedDeformationFitCost : public IEmbeddedDeformationFitCost
{
private:
	std::map<vertex_descriptor, std::vector<ceres::ResidualBlockId>> _fit_residual_ids;
	std::unique_ptr<FindCorrespondingPoints> _find_correspondence_point;
	std::vector<vertex_descriptor> _subset_of_vertices_to_fit;
	RegistrationOptions _options;
private:
	bool useVertex(vertex_descriptor & v, DeformedMesh<DeformationGraph<EDDeformation>> & deformed_mesh);
	bool addFitCostVertex(ceres::Problem & problem, 
						  vertex_descriptor & v, 
						  DeformationGraph<EDDeformation> & deformation_graph,
						  DeformedMesh<DeformationGraph<EDDeformation>> & deformed_mesh);
public:
	void evaluateResiduals(ceres::Problem &problem, SurfaceMesh & mesh, CeresIterationLoggerGuard & logger) override;
	VertexResidualIds addFitCost(ceres::Problem &problem,
								 DeformationGraph<EDDeformation> & deformation_graph,
								 DeformedMesh<DeformationGraph<EDDeformation>> & deformed_mesh,
								 std::unique_ptr<CeresIterationLoggerGuard>& logger) override;
public:
	EmbeddedDeformationFitCost(SurfaceMesh & target,
							   std::vector<vertex_descriptor> & subset_of_vertices_to_fit,
							   RegistrationOptions options);
};



}