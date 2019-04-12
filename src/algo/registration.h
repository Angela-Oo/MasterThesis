#pragma once
#include "algo/rigid_registration/icp.h"
#include "algo/non_rigid_registration/embedded_deformation.h"
#include "algo/non_rigid_registration/as_rigid_as_possible.h"

class IRegistration
{
public:
	virtual bool solve() = 0;
	virtual std::vector<ml::vec3f> getPointsA() = 0;
	virtual std::vector<ml::vec3f> getPointsB() = 0;
	virtual std::vector<ml::vec3f> getPointsDeformationGraph() = 0;
	virtual ~IRegistration() = default;
};

class RigidRegistration : public IRegistration
{
private:
	std::unique_ptr<ICP> _icp_nn;
	std::vector<ml::vec3f> _points_a;
	std::vector<ml::vec3f> _points_b;
	ml::mat4f _transformation;
public:
	bool solve() override;
	void icp_calc_nn_in_cost_function();
	std::vector<ml::vec3f> getPointsA() override;
	std::vector<ml::vec3f> getPointsB() override;
	std::vector<ml::vec3f> getPointsDeformationGraph() override;
public:
	RigidRegistration(const std::vector<ml::vec3f> & points_a, const std::vector<ml::vec3f> & points_b);
};

class NonRigidRegistration : public IRegistration
{
private:
	std::unique_ptr<ICP> _icp_nn;
	std::vector<ml::vec3f> _points_a;
	std::vector<ml::vec3f> _points_b;
	std::vector<ml::vec6d> _transformation;
	std::unique_ptr<EmbeddedDeformation> _embedded_deformation;
	std::unique_ptr<AsRigidAsPossible> _as_rigid_as_possible;
public:
	bool solve() override;
	std::vector<ml::vec3f> getPointsA() override;
	std::vector<ml::vec3f> getPointsB() override;
	std::vector<ml::vec3f> getPointsDeformationGraph() override;
public:
	NonRigidRegistration();
	NonRigidRegistration(const std::vector<ml::vec3f> & points_a, const std::vector<ml::vec3f> & points_b);
};
