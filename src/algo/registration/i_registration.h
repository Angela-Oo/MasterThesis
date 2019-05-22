#pragma once

#include "mLibInclude.h"
#include <vector>

typedef ml::TriMeshf Mesh;

struct NodeGradient
{
	ml::vec3d translation;
	ml::vec3d rotation;
	double w;
};

struct ARAPGradient
{
	std::vector<ml::vec3f> point;
	std::vector<NodeGradient> fit_point_to_point_gradient;
	std::vector<NodeGradient> fit_point_to_plane_gradient;
	std::vector<NodeGradient> smooth_gradient;
	std::vector<NodeGradient> all;
};


class IRegistration
{
public:
	virtual bool finished() = 0;
	virtual bool solveIteration() = 0;
	virtual bool solve() = 0;

	virtual std::map<std::string, std::map<vertex_index, std::vector<double>>> residuals()
	{ return std::map<std::string, std::map<vertex_index, std::vector<double>>>(); };
public:
	virtual const Mesh & getSource() = 0;
	virtual const Mesh & getTarget() = 0;
	virtual Mesh getDeformedPoints() = 0;
	virtual Mesh getInverseDeformedPoints() = 0;
public:
	virtual std::vector<ml::vec3f> getFixedPostions() { return std::vector<ml::vec3f>(); }
	virtual std::pair<std::vector<ml::vec3f>, std::vector<ml::vec3f>> getDeformationGraph() { return std::make_pair(std::vector<ml::vec3f>(), std::vector<ml::vec3f>()); }
	virtual Mesh getDeformationGraphMesh() { return Mesh(); };
public:
	virtual ~IRegistration() = default;
};
