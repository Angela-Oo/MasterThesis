#pragma once
#include "../mLibInclude.h"
#include "PointsFromDepthSensor.h"
#include "i_showData.h"
#include "kinect/PrimeSenseSensor.h"
#include <chrono>

class ShowKinectData : public IShowData
{
private:
	//void processFrame();
	void renderPoints(int frame);
	ml::mat4f getWorldTransformation();
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;
private:
	ml::D3D11TriMesh m_pointCloud;
	ml::D3D11ShaderManager m_shaderManager;
	ml::D3D11ConstantBuffer<ConstantBuffer> m_constants;
	ml::GraphicsDevice * _graphics;
	//std::unique_ptr<SensorDataWrapper> _sensor_data_wrapper;
	std::unique_ptr<CalibrateSensorDataWrapper> _sensor_data_wrapper;
	PrimeSenseSensor _depth_sensor;
	unsigned int _frame = 0;
	std::chrono::time_point<std::chrono::system_clock> _start_time;
	std::vector<ml::vec3f> _all_points;
};