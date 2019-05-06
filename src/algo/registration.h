#pragma once
#include "algo/rigid_registration/icp.h"
#include "algo/non_rigid_registration/embedded_deformation.h"
#include "algo/non_rigid_registration/as_rigid_as_possible.h"
#include "algo/file_writer.h"

typedef ml::TriMeshf Mesh;

ceres::Solver::Options ceresOption();

class IRegistration
{
public:
	virtual bool solve() = 0;
	virtual Mesh getPointsA() = 0;
	virtual Mesh getPointsB() = 0;
	virtual std::vector<ml::vec3f> getPointsDeformationGraph() = 0;
	virtual std::pair<std::vector<ml::vec3f>, std::vector<ml::vec3f>> getDeformationGraph() = 0;
	virtual std::vector<ml::vec3f> getFixedPositions() = 0;
	virtual ~IRegistration() = default;
};

class RigidRegistration : public IRegistration
{
private:
	std::unique_ptr<ICPNN> _icp_nn;
	Mesh _points_a;
	Mesh _points_b;
	ml::mat4f _transformation;
	std::shared_ptr<FileWriter> _logger;
public:
	bool solve() override;
	Mesh getPointsA() override;
	Mesh getPointsB() override;
	std::vector<ml::vec3f> getPointsDeformationGraph() override;
	std::pair<std::vector<ml::vec3f>, std::vector<ml::vec3f>> getDeformationGraph() override { return std::make_pair(std::vector<ml::vec3f>(), std::vector<ml::vec3f>());  };
	std::vector<ml::vec3f> getFixedPositions() override { return std::vector<ml::vec3f>(); }
public:
	RigidRegistration(const Mesh & points_a, const Mesh & points_b, std::shared_ptr<FileWriter> logger = nullptr);
};

class NonRigidRegistration : public IRegistration
{
private:
	//std::unique_ptr<ED::EmbeddedDeformation> _embedded_deformation;
	std::unique_ptr<ED::EmbeddedDeformationWithoutICP> _embedded_deformation;
public:
	bool solve() override;
	Mesh getPointsA() override;
	Mesh getPointsB() override;
	std::vector<ml::vec3f> getPointsDeformationGraph() override;
	std::pair<std::vector<ml::vec3f>, std::vector<ml::vec3f>> getDeformationGraph() override;
	std::vector<ml::vec3f> getFixedPositions() override;
public:
	NonRigidRegistration(const Mesh & points_a, const Mesh & points_b, std::vector<int> fixed_positions = std::vector<int>(), unsigned int number_of_deformation_nodes = 1000, std::shared_ptr<FileWriter> logger = nullptr);
};


class ARAPNonRigidRegistration : public IRegistration
{
private:
	//std::unique_ptr<AsRigidAsPossible> _as_rigid_as_possible;
	std::unique_ptr<AsRigidAsPossibleWithoutICP> _as_rigid_as_possible;
public:
	bool solve() override;
	Mesh getPointsA() override;
	Mesh getPointsB() override;
	std::vector<ml::vec3f> getPointsDeformationGraph() override;
	std::pair<std::vector<ml::vec3f>, std::vector<ml::vec3f>> getDeformationGraph() override;
	std::vector<ml::vec3f> getFixedPositions() override;
public:
	ARAPNonRigidRegistration(const Mesh & points_a, const Mesh & points_b, std::vector<int> fixed_positions = std::vector<int>(), unsigned int number_of_deformation_nodes = 1000, std::shared_ptr<FileWriter> logger = nullptr);
};



class NonRigidRegistrationFrames
{
private:
	std::vector<Mesh> _meshes;
	std::vector<Mesh> _deformed_meshes;
	std::vector<DeformationGraph<ED::Graph, ED::Node>> _deformation_graphs;
	std::vector<ml::vec6d> _transformation;
	size_t _current;
	unsigned int _number_of_deformation_nodes;
	std::unique_ptr<ED::EmbeddedDeformation> _embedded_deformation;
public:
	bool solve();
	bool finished();
	size_t getCurrent();
	Mesh getMesh(int frame);
	Mesh getDeformedMesh(int frame);
	DeformationGraph<ED::Graph, ED::Node> getDeformationGraph(int frame);
public:
	NonRigidRegistrationFrames();
	NonRigidRegistrationFrames(const std::vector<Mesh> & meshes, unsigned int number_of_deformation_nodes = 1000);
};

