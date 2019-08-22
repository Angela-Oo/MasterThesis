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

ceres::ResidualBlockId pointToPointCost3NN(ceres::Problem & problem,
										   double loss_weighting,
										   DeformationGraph<ARAPDeformation> & deformation_graph,
										   DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
										   const vertex_descriptor &v,
										   const Point & target_point);

ceres::ResidualBlockId pointToPointCost4NN(ceres::Problem & problem,
										   double loss_weighting,
										   DeformationGraph<ARAPDeformation> & deformation_graph,
										   DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
										   const vertex_descriptor &v,
										   const Point & target_point);

ceres::ResidualBlockId pointToPointCost(ceres::Problem & problem,
										double loss_weighting,
										double k_neighbors,
										DeformationGraph<ARAPDeformation> & deformation_graph,
										DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
										const vertex_descriptor &v,
										const Point & target_point);

// Point to Plane

ceres::ResidualBlockId pointToPlaneCost3NN(ceres::Problem & problem,
										   double loss_weighting,
										   DeformationGraph<ARAPDeformation> & deformation_graph,
										   DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
										   const vertex_descriptor &v,
										   const Point & target_point,
										   const Vector & target_normal);

ceres::ResidualBlockId pointToPlaneCost4NN(ceres::Problem & problem,
										   double loss_weighting,
										   DeformationGraph<ARAPDeformation> & deformation_graph,
										   DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
										   const vertex_descriptor &v,
										   const Point & target_point,
										   const Vector & target_normal);

ceres::ResidualBlockId pointToPlaneCost(ceres::Problem & problem,
										double loss_weighting,
										double k_neighbors,
										DeformationGraph<ARAPDeformation> & deformation_graph,
										DeformedMesh<DeformationGraph<ARAPDeformation>> & deformed_mesh,
										const vertex_descriptor &v,
										const Point & target_point,
										const Vector & target_normal);


// As Rigid As Possible


ceres::ResidualBlockId asRigidAsPossibleCost(ceres::Problem & problem,
											 double loss_weighting,
											 DeformationGraph<ARAPDeformation> & deformation_graph,
											 const vertex_descriptor &source,
											 const vertex_descriptor &target);


std::map<edge_descriptor, std::vector<ceres::ResidualBlockId>> asRigidAsPossibleCostUseRigidity(ceres::Problem &problem,
																					            double loss_weighting,
																					            DeformationGraph<ARAPDeformation> & deformation_graph);


std::map<edge_descriptor, std::vector<ceres::ResidualBlockId>> asRigidAsPossibleCost(ceres::Problem &problem,
																					 double loss_weighting,
																					 DeformationGraph<ARAPDeformation> & deformation_graph);

}