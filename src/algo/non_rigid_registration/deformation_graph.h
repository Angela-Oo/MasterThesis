#pragma once

#include "mLibInclude.h"
#include "node.h"
#include "deformation_graph_knn.h"

typedef ml::TriMeshf Mesh;

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
	Mesh deformPoints(const Mesh & points);
	std::vector<ml::vec3f> getDeformationGraph();
public:
	DeformationGraph() = default;
	DeformationGraph(const Mesh & nodes, size_t number_of_nodes);
	DeformationGraph(const DeformationGraph & deformation_graph);
	DeformationGraph & operator=(DeformationGraph other);
};