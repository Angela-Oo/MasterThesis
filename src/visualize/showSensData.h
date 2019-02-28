#pragma once
#include "../mLibInclude.h"
#include "i_showData.h"
#include "ext-depthcamera/sensorData.h"
#include <chrono>

class ShowSensData : public IShowData
{
private:
	void showFrame();
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;
private:
	ml::D3D11TriMesh m_pointCloud;
	ml::D3D11ShaderManager m_shaderManager;
	ml::D3D11ConstantBuffer<ConstantBuffer> m_constants;
	SensorData _sensor_data;
	ml::GraphicsDevice * _graphics;
	unsigned int _frame = 0;
	std::chrono::time_point<std::chrono::system_clock> _start_time;
public:
	ShowSensData() = default;
	~ShowSensData() = default;
};