#pragma once


#ifndef GLOG_NO_ABBREVIATED_SEVERITIES
#define GLOG_NO_ABBREVIATED_SEVERITIES
#endif // !GLOG_NO_ABBREVIATED_SEVERITIES

#include "glog/logging.h"

#include "algo/registration/interface/i_registration.h"
#include "mesh/mesh_definition.h"
#include "arap_deformation.h"
#include "algo/registration/deformation_graph/deformation_graph.h"
#include "algo/registration/deformation_graph/deformed_mesh.h"
#include "algo/registration/find_corresponding_points/find_corresponding_points.h"
#include "algo/registration/util/ceres_iteration_logger.h"
#include <ceres/ceres.h>
#include <ceres/problem.h>

namespace Registration
{

// Point to Point

const ceres::ResidualBlockId & pointToPointCost3NN(ceres::Problem & problem,
												   double loss_weighting,
												   DeformationGraph<ARAPDeformation> & deformation_graph,
												   DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
												   const vertex_descriptor &v,
												   const Point & target_point);

const ceres::ResidualBlockId & pointToPointCost4NN(ceres::Problem & problem,
												   double loss_weighting,
												   DeformationGraph<ARAPDeformation> & deformation_graph,
												   DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
												   const vertex_descriptor &v,
												   const Point & target_point);

const ceres::ResidualBlockId & pointToPointCost(ceres::Problem & problem,
												double loss_weighting,
												double k_neighbors,
												DeformationGraph<ARAPDeformation> & deformation_graph,
												DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
												const vertex_descriptor &v,
												const Point & target_point);

// Point to Plane

const ceres::ResidualBlockId & pointToPlaneCost3NN(ceres::Problem & problem,
												   double loss_weighting,
												   DeformationGraph<ARAPDeformation> & deformation_graph,
												   DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
												   const vertex_descriptor &v,
												   const Point & target_point,
												   const Vector & target_normal);

const ceres::ResidualBlockId & pointToPlaneCost4NN(ceres::Problem & problem,
												   double loss_weighting,
												   DeformationGraph<ARAPDeformation> & deformation_graph,
												   DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
												   const vertex_descriptor &v,
												   const Point & target_point,
												   const Vector & target_normal);

const ceres::ResidualBlockId & pointToPlaneCost(ceres::Problem & problem,
												double loss_weighting,
												double k_neighbors,
												DeformationGraph<ARAPDeformation> & deformation_graph,
												DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
												const vertex_descriptor &v,
												const Point & target_point,
												const Vector & target_normal);

typedef std::vector<ceres::ResidualBlockId> ResidualIds;
typedef std::map<vertex_descriptor, ResidualIds> VertexResidualIds;

class AsRigidAsPossibleFitCost
{
private:
	std::unique_ptr<FindCorrespondingPoints> _find_correspondence_point;
	std::vector<vertex_descriptor> _subset_of_vertices_to_fit;
	RegistrationOptions _options;
private:
	bool useVertex(vertex_descriptor & v, DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh);
	bool addFitCostVertex(ceres::Problem & problem, vertex_descriptor & v, DeformationGraph<ARAPDeformation> & deformation_graph,
						  DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh, VertexResidualIds &residual_ids);
public:
	std::map<vertex_descriptor, ResidualIds> addFitCost(ceres::Problem &problem, 
														DeformationGraph<ARAPDeformation> & deformation_graph,
														DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh, 
														std::unique_ptr<CeresIterationLoggerGuard>& logger);
public:
	AsRigidAsPossibleFitCost(SurfaceMesh & target,
							 std::vector<vertex_descriptor> & subset_of_vertices_to_fit,
							 RegistrationOptions options);
};

class AsRigidAsPossibleFitCostWithoutICP
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
	std::map<vertex_descriptor, ResidualIds> addFitCost(ceres::Problem &problem,
														DeformationGraph<ARAPDeformation> & deformation_graph,
														DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
														std::unique_ptr<CeresIterationLoggerGuard>& logger);

public:
	AsRigidAsPossibleFitCostWithoutICP(SurfaceMesh & target,
									   std::vector<vertex_descriptor> fixed_positions,
									   std::vector<vertex_descriptor> & subset_of_vertices_to_fit,
									   RegistrationOptions options);
};


}