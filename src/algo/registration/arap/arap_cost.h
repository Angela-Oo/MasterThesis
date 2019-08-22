#pragma once


#ifndef GLOG_NO_ABBREVIATED_SEVERITIES
#define GLOG_NO_ABBREVIATED_SEVERITIES
#endif // !GLOG_NO_ABBREVIATED_SEVERITIES

#include "glog/logging.h"

#include "mesh/mesh_definition.h"
#include "arap_deformation.h"
#include "algo/registration/deformation_graph/deformation_graph.h"
#include "algo/registration/deformation_graph/deformed_mesh.h"
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

}