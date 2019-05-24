#pragma once

#include "mLibInclude.h"
#include "boost/graph/adjacency_list.hpp"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

#include "algo/deformation_graph/i_node.h"


#include <CGAL/Search_traits_3.h>
#include <CGAL/Search_traits_adapter.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Kernel::Point_3> DeformationGraphMesh;
typedef CGAL::Surface_mesh<Kernel::Point_3> SurfaceMesh;
typedef DeformationGraphMesh::Vertex_index vertex_descriptor;
typedef DeformationGraphMesh::Edge_index edge_descriptor;
typedef DeformationGraphMesh::Face_index face_descriptor;
typedef DeformationGraphMesh::Point Point;

//typedef boost::property_map<vertex_descriptor, std::shared_ptr<INode>>::type NodeProperty;

// search
typedef boost::property_map<SurfaceMesh, CGAL::vertex_point_t>::type Vertex_point_pmap;
typedef boost::graph_traits<SurfaceMesh>::vertices_size_type size_type;

typedef CGAL::Search_traits_3<Kernel> TreeTraits;
typedef CGAL::Search_traits_adapter<boost::graph_traits<SurfaceMesh>::vertex_descriptor, Vertex_point_pmap, TreeTraits> Traits;

typedef CGAL::Orthogonal_k_neighbor_search<Traits> Neighbor_search;
typedef Neighbor_search::Tree Tree;

typedef ml::TriMeshf Mesh;
//
//// mesh.add_property_map<vertex_descriptor, std::shared_ptr<INode>>("node", nullptr);
//class INode
//{
//public:
//	virtual ml::vec3f & position() = 0;
//	virtual ml::vec3f & normal() = 0;
//	virtual double * r() = 0;
//	virtual double * t() = 0;
//	virtual double * w() = 0;
//public:
//	virtual ml::mat3d rotation() const = 0;
//	virtual ml::vec3d translation() const = 0;
//	virtual double weight() const = 0;
//public:
//	virtual ml::vec3d deformedPosition() const = 0;
//	virtual ml::vec3d deformedNormal() const = 0;
//	virtual ml::vec3d deformPosition(const ml::vec3f & pos) const = 0;
//	virtual ml::vec3d deformNormal(const ml::vec3f & normal) const = 0;
//};
//
////auto property_map = mesh.add_property_map<edge_descriptor, std::shared_ptr<IEdge>>("edge", nullptr);
//class IEdge
//{
//public:
//	virtual Kernel::Point_3 deformed() = 0;
//};


DeformationGraphMesh convertToDeformationGraphMesh(const Mesh& triMesh);
//
//Mesh convertToTriMesh(DeformationGraphMesh& mesh);
