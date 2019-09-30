#pragma once

#include "mesh/mesh_definition.h"
#include "ed_deformation.h"
#include "algo/registration/deformation_graph/deformation_graph.h"
#include "algo/registration/deformation_graph/deformed_mesh.h"
#include "util/ceres_include.h"

namespace Registration {

// Point to Point

ceres::ResidualBlockId pointToPointCost3NN(ceres::Problem & problem,
										   double loss_weighting,
										   DeformationGraph<EDDeformation> & deformation_graph,
										   DeformedMesh<DeformationGraph<EDDeformation>> & deformed_mesh,
										   const vertex_descriptor &v,
										   const Point & target_point);

ceres::ResidualBlockId pointToPointCost4NN(ceres::Problem & problem,
										   double loss_weighting,
										   DeformationGraph<EDDeformation> & deformation_graph,
										   DeformedMesh<DeformationGraph<EDDeformation>> & deformed_mesh,
										   const vertex_descriptor &v,
										   const Point & target_point);

ceres::ResidualBlockId pointToPointCost5NN(ceres::Problem & problem,
										   double loss_weighting,
										   DeformationGraph<EDDeformation> & deformation_graph,
										   DeformedMesh<DeformationGraph<EDDeformation>> & deformed_mesh,
										   const vertex_descriptor &v,
										   const Point & target_point);

ceres::ResidualBlockId pointToPointCost6NN(ceres::Problem & problem,
										   double loss_weighting,
										   DeformationGraph<EDDeformation> & deformation_graph,
										   DeformedMesh<DeformationGraph<EDDeformation>> & deformed_mesh,
										   const vertex_descriptor &v,
										   const Point & target_point);

ceres::ResidualBlockId pointToPointCost(ceres::Problem & problem,
										double loss_weighting,
										DeformationGraph<EDDeformation> & deformation_graph,
										DeformedMesh<DeformationGraph<EDDeformation>> & deformed_mesh,
										const vertex_descriptor &v,
										const Point & target_point);

// Point to Plane

ceres::ResidualBlockId pointToPlaneCost3NN(ceres::Problem & problem,
										   double loss_weighting,
										   DeformationGraph<EDDeformation> & deformation_graph,
										   DeformedMesh<DeformationGraph<EDDeformation>> & deformed_mesh,
										   const vertex_descriptor &v,
										   const Point & target_point,
										   const Vector & target_normal);

ceres::ResidualBlockId pointToPlaneCost4NN(ceres::Problem & problem,
										   double loss_weighting,
										   DeformationGraph<EDDeformation> & deformation_graph,
										   DeformedMesh<DeformationGraph<EDDeformation>> & deformed_mesh,
										   const vertex_descriptor &v,
										   const Point & target_point,
										   const Vector & target_normal);

ceres::ResidualBlockId pointToPlaneCost5NN(ceres::Problem & problem,
										   double loss_weighting,
										   DeformationGraph<EDDeformation> & deformation_graph,
										   DeformedMesh<DeformationGraph<EDDeformation>> & deformed_mesh,
										   const vertex_descriptor &v,
										   const Point & target_point,
										   const Vector & target_normal);

ceres::ResidualBlockId pointToPlaneCost6NN(ceres::Problem & problem,
										   double loss_weighting,
										   DeformationGraph<EDDeformation> & deformation_graph,
										   DeformedMesh<DeformationGraph<EDDeformation>> & deformed_mesh,
										   const vertex_descriptor &v,
										   const Point & target_point,
										   const Vector & target_normal);

ceres::ResidualBlockId pointToPlaneCost(ceres::Problem & problem,
										double loss_weighting,
										DeformationGraph<EDDeformation> & deformation_graph,
										DeformedMesh<DeformationGraph<EDDeformation>> & deformed_mesh,
										const vertex_descriptor &v,
										const Point & target_point,
										const Vector & target_normal);



}