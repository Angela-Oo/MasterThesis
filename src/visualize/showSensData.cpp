#include "stdafx.h"
#include "showSensData.h"
#include <numeric>

using namespace ml;

void ShowSensData::showFrame()
{
	auto points = _sensor_data.computePointCloud(_frame).m_points;

	auto average = std::accumulate(points.begin(), points.end(), vec3f(0., 0., 0.)) / static_cast<float>(points.size());
	mat4f center = mat4f::translation(-average);
	float scale_factor = 1.;
	mat4f scale = mat4f::scale({ scale_factor, scale_factor, scale_factor });
	mat4f transform = mat4f::translation({ -0.5f, -2.f, 1.2f });
	mat4f translate = transform * scale * center;
	std::for_each(points.begin(), points.end(), [&translate](vec3f & p) { p = translate * p; });

	_frame++;
	if (!points.empty()) {
		auto point_template = ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.002f), points);
		m_pointCloud.init(*_graphics, point_template);
	}
}

void ShowSensData::init(ml::ApplicationData &app)
{
	_graphics = &app.graphics;
	_start_time = std::chrono::system_clock::now();

	_sensor_data.loadFromFile("D:/Studium/MasterThesis/input_data/scannet/scans/scene0000_00/scene0000_00.sens");
	
	m_shaderManager.init(app.graphics);
	m_shaderManager.registerShader("shaders/pointCloud.hlsl", "pointCloud");
	m_constants.init(app.graphics);
	showFrame();
}

void ShowSensData::render(ml::Cameraf& camera)
{
	auto end = std::chrono::system_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(end - _start_time).count();
	if (elapsed > 3) {
		showFrame();
		_start_time = std::chrono::system_clock::now();
	}
	ConstantBuffer constants;
	constants.worldViewProj = camera.getViewProj();

	m_constants.updateAndBind(constants, 0);
	m_shaderManager.bindShaders("pointCloud");
	m_constants.bind(0);
	m_pointCloud.render();
}


