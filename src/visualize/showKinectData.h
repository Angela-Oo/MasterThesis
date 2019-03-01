#pragma once
#include "../mLibInclude.h"
#include "PointsFromDepthSensor.h"
#include "i_showData.h"
#include "kinect/PrimeSenseSensor.h"


class ShowKinectData : public IShowData
{
private:
	void processFrame();
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;
private:
	ml::D3D11TriMesh m_pointCloud;
	ml::D3D11ShaderManager m_shaderManager;
	ml::D3D11ConstantBuffer<ConstantBuffer> m_constants;
	ml::GraphicsDevice * _graphics;
	std::unique_ptr<SensorDataWrapper> _sensor_data_wrapper;
	PrimeSenseSensor _depth_sensor;
};