#pragma once

#include "mesh/mesh_definition.h"
#include <optional>

namespace Registration
{

halfedge_descriptor splitEdgeAtCenter(edge_descriptor e, SurfaceMesh & mesh);

void splitEdge(edge_descriptor e, SurfaceMesh & mesh);

boost::optional<halfedge_descriptor> halfedgeCorrespondingToFace(face_descriptor f, vertex_descriptor v, const SurfaceMesh & mesh);

bool splitFaceAtEdge(face_descriptor f, vertex_descriptor v0, SurfaceMesh & mesh);

bool splitFaceAtEdge(face_descriptor f, vertex_descriptor v0, vertex_descriptor v1, SurfaceMesh & mesh);

bool splitFaceAtEdge(face_descriptor f, vertex_descriptor v0, vertex_descriptor v1, vertex_descriptor v2, SurfaceMesh & mesh);

void splitFaceAtEdge(face_descriptor f, SurfaceMesh & mesh);

// at center
vertex_descriptor splitFace(face_descriptor f, SurfaceMesh & mesh);

void flipEdges(std::vector<edge_descriptor> & edges, SurfaceMesh & mesh);


void splitFaces(const std::map<face_descriptor, std::vector<vertex_descriptor>> & face_vertices_to_split_map, SurfaceMesh & mesh);




std::set<vertex_descriptor> oneRingNeighborhood(const SurfaceMesh & mesh, vertex_descriptor v);

std::set<vertex_descriptor> twoRingNeighborhood(const SurfaceMesh & mesh, vertex_descriptor v);

}