#pragma once

#include "algo/surface_mesh/mesh_definition.h"
#include <optional>

namespace Registration
{

halfedge_descriptor splitEdgeAtCenter(edge_descriptor e, SurfaceMesh & mesh);

void splitEdge(edge_descriptor e, SurfaceMesh & mesh);

boost::optional<halfedge_descriptor> halfedgeCorrespondingToFace(face_descriptor f, vertex_descriptor v, const SurfaceMesh & mesh);

bool splitFace(face_descriptor f, vertex_descriptor v0, vertex_descriptor v1, SurfaceMesh & mesh);

bool splitFace(face_descriptor f, vertex_descriptor v0, vertex_descriptor v1, vertex_descriptor v2, SurfaceMesh & mesh);

void splitFace(face_descriptor f, SurfaceMesh & mesh);


}