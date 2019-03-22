#pragma once
#include "../mLibInclude.h"
#include "i_showData.h"
#include "kinect/SensorDataWrapper.h"
#include "kinect/ImageReaderSensor.h"
#include "algo/icp.h"

class RigidRegistration
{
private:
	std::unique_ptr<ICP> _icp_nn;
	std::vector<ml::vec3f> _points_a;
	std::vector<ml::vec3f> _points_b;
	ml::mat4f _transformation;
private:
	std::vector<ml::vec3f> transform(std::vector<ml::vec3f> points);
public:
	void icp();
	void icp_calc_nn_in_cost_function();

	std::vector<ml::vec3f> getPointsA();
	std::vector<ml::vec3f> getPointsB();
public:
	RigidRegistration(const std::vector<ml::vec3f> & points_a, const std::vector<ml::vec3f> & points_b);

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
	std::unique_ptr<RigidRegistration> _rigid_registration;
private:
	void transform(std::vector<ml::vec3f>& points);
	void configImageReaderSensor(std::string filepath);
	void renderPoints();
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;
	void key(UINT key) override;
public:
	ShowTwoRigideRegisteredFrames() {};
	~ShowTwoRigideRegisteredFrames() = default;
};