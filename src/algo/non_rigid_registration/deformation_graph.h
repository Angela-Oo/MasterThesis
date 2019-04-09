#pragma once

#include "mLibInclude.h"
#include "node.h"
#include "deformation_graph_knn.h"

class DeformationGraph
{
private:
	const int _k = 4;
public:
	Node _global_rigid_deformation;
	Graph _graph;
	std::unique_ptr<DeformationGraphKNN> _knn;
private:
	double weight(const ml::vec3f & point, Node & node, double dmax);
	std::vector<double> weights(const ml::vec3f & point, std::vector<Node>& k_plus_1_nearest_nodes);
	ml::vec3f deformPoint(const ml::vec3f & point, std::vector<Node> & k_plus_1_nearest_nodes);
public:
	std::vector<ml::vec3f> deformPoints(const std::vector<ml::vec3f> & points);
public:
	DeformationGraph(const std::vector<ml::vec3f> & nodes, size_t number_of_nodes);
};


//class DeformationGraph
//{
//private:
//	const std::vector<ml::vec3f> & _points;
//	const int k = 4;
//	std::vector<std::vector<vertex_index>> _k_nearest_nodes_of_points;	
//public:
//	Node _global_rigid_deformation;
//	Graph _graph;	
//public:
//	double weight(const ml::vec3f & point, Node & node, double dmax);
//	std::vector<double> weights(const ml::vec3f & point, std::vector<Node>& nodes);
//	ml::vec3f deformPoint(const ml::vec3f & point, std::vector<vertex_index> & k_nearest_node_ids);
//	std::vector<ml::vec3f> deformPoints(const std::vector<ml::vec3f> & points);
//	std::vector<ml::vec3f> getDeformedPoints();
//public:
//	DeformationGraph(const std::vector<ml::vec3f> & points, size_t number_of_nodes);
//};
