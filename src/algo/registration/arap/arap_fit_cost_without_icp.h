#pragma once


#include "i_arap_fit_cost.h"
#include "arap_deformation.h"
#include "algo/registration/interface/i_registration.h"
#include "algo/registration/interface/registration_options.h"
#include "mesh/mesh_definition.h"
#include "algo/registration/find_corresponding_points/find_corresponding_points.h"
#include "util/ceres_include.h"

namespace Registration
{

class AsRigidAsPossibleFitCostWithoutICP : public IAsRigidAsPossibleFitCost
{
private:
	SurfaceMesh & _target;
	std::vector<vertex_descriptor> _subset_of_vertices_to_fit;
	RegistrationOptions _options;
	std::vector<vertex_descriptor> _fixed_positions;
private:
	ResidualIds addFitCostVertex(ceres::Problem & problem,
								 DeformationGraph<ARAPDeformation> & deformation_graph,
								 DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
								 SurfaceMesh::Vertex_index & v);
public:
	std::vector<Point> getFixedPostions() override;
	VertexResidualIds addFitCost(ceres::Problem &problem,
								 DeformationGraph<ARAPDeformation> & deformation_graph,
								 DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
								 std::unique_ptr<CeresIterationLoggerGuard>& logger) override;

public:
	AsRigidAsPossibleFitCostWithoutICP(SurfaceMesh & target,
										std::vector<vertex_descriptor> fixed_positions,
										std::vector<vertex_descriptor> & subset_of_vertices_to_fit,
										RegistrationOptions options);
};


}