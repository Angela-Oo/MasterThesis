#pragma once

#include "mLibInclude.h"
#include <vector>

typedef ml::TriMeshf Mesh;

class IRegistration
{
public:
	virtual bool finished() = 0;
	virtual bool solveIteration() = 0;
	virtual bool solve() = 0;

	virtual std::vector<double> gradient() { return std::vector<double>(); };
public:
	virtual const Mesh & getSource() = 0;
	virtual const Mesh & getTarget() = 0;
	virtual Mesh getDeformedPoints() = 0;
	virtual Mesh getInverseDeformedPoints() = 0;
public:
	virtual std::vector<ml::vec3f> getFixedPostions() { return std::vector<ml::vec3f>(); }
	virtual std::pair<std::vector<ml::vec3f>, std::vector<ml::vec3f>> getDeformationGraph() { return std::make_pair(std::vector<ml::vec3f>(), std::vector<ml::vec3f>()); }
public:
	virtual ~IRegistration() = default;
};
