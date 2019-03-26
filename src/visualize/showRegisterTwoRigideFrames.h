#pragma once
#include "../mLibInclude.h"
#include "i_showData.h"
#include "kinect/SensorDataWrapper.h"
#include "kinect/ImageReaderSensor.h"
#include "algo/icp.h"


class IRegistration
{
public:
	virtual void solve() = 0;
	virtual std::vector<ml::vec3f> getPointsA() = 0;
	virtual std::vector<ml::vec3f> getPointsB() = 0;
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
	void solve() override;
	void icp_calc_nn_in_cost_function();
	std::vector<ml::vec3f> getPointsA() override;
	std::vector<ml::vec3f> getPointsB() override;
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
public:
	void solve() override;
	std::vector<ml::vec3f> getPointsA() override;
	std::vector<ml::vec3f> getPointsB() override;
public:
	NonRigidRegistration();
};

class ShowTwoRigideRegisteredFrames : public IShowData
{
private:
	std::vector<ml::vec3f> _points_a;
	std::vector<ml::vec3f> _points_b;
	ml::D3D11TriMesh m_pointCloudFrameA;
	ml::D3D11TriMesh m_pointCloudFrameB;
	ml::D3D11ShaderManager m_shaderManager;
	ml::D3D11ConstantBuffer<ConstantBuffer> m_constants;
	std::unique_ptr<CalibrateSensorDataWrapper> _sensor_data;
	ImageReaderSensor _depth_sensor;
	ml::GraphicsDevice * _graphics;
	bool icp_active = false;
	std::unique_ptr<IRegistration> _registration;
private:
	void transform(std::vector<ml::vec3f>& points);
	void configImageReaderSensor(std::string filepath);
	void renderPoints();
	void initICP();
	void initNonRigidRegistration();
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;
	void key(UINT key) override;
public:
	ShowTwoRigideRegisteredFrames() {};
	~ShowTwoRigideRegisteredFrames() = default;
};