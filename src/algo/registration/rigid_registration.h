#pragma once
#include "i_registration.h"
#include "icp.h"
#include "algo/file_writer.h"

typedef ml::TriMeshf Mesh;

class RigidRegistration : public IRegistration
{
private:
	std::unique_ptr<ICPNN> _icp_nn;
	Mesh _points_a;
	Mesh _points_b;
	ml::mat4f _transformation;
	std::shared_ptr<FileWriter> _logger;
public:
	bool finished() override;
	bool solveIteration() override;
	bool solve() override;
public:
	const Mesh & getSource() override;
	const Mesh & getTarget() override;
	Mesh getDeformedPoints() override;
	Mesh getInverseDeformedPoints() override;
public:
	RigidRegistration(const Mesh & points_a, const Mesh & points_b, ceres::Solver::Options option, std::shared_ptr<FileWriter> logger = nullptr);
};

