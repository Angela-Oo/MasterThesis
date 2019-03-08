#include "showKinectData.h"
//#include "kinect/BinaryDumpReader.h"
#include "kinect/KinectSensor.h"

#include "ext-depthcamera/sensorData.h"
#include <numeric>

using namespace ml;

//void ShowKinectData::processFrame()
//{
//	//auto points = _sensor_data_wrapper->addFrame(2);
//	auto points = _sensor_data_wrapper->get3DPoints(2);
//
//	auto average = std::accumulate(points.begin(), points.end(), vec3f(0., 0., 0.)) / static_cast<float>(points.size());
//	mat4f center = mat4f::translation(-average);
//	float scale_factor = 1.;
//	//float scale_factor = 0.01;
//	mat4f scale = mat4f::scale({ scale_factor, scale_factor, scale_factor });
//	mat4f rotation = mat4f::rotationX(90.) * mat4f::rotationZ(-90.);
//	mat4f transform = mat4f::translation({ -0.5f, -2.f, 1.2f });
//	mat4f translate = transform * rotation * scale * center;
//	std::for_each(points.begin(), points.end(), [&translate](vec3f & p) { p = translate * p; });
//
//	//_all_points.insert(_all_points.end(), points.begin(), points.end());
//	m_pointCloud.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.002f), points /*_all_points*/));
//	_frame++;
//}


void ShowKinectData::renderPoints(int frame)
{
	auto points = _sensor_data_wrapper->getPoints(frame);

	//auto average = std::accumulate(points.begin(), points.end(), vec3f(0., 0., 0.)) / static_cast<float>(points.size());
	//mat4f center = mat4f::translation(-average);
	//std::for_each(points.begin(), points.end(), [&center](vec3f & p) { p = center * p; });
	
	m_pointCloud.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.002f), points /*_all_points*/));
}


void ShowKinectData::init(ml::ApplicationData &app)
{
	_graphics = &app.graphics;
	_start_time = std::chrono::system_clock::now();

	if (_depth_sensor.createFirstConnected() == S_OK)
	{
		auto intrinsic = _depth_sensor.getIntrinsics();
		//_depth_sensor.toggleNearMode();
		//_sensor_data_wrapper = std::make_unique<SensorDataWrapper>(_depth_sensor);// intrinsics ??
		auto depth_intrinsics = intrinsic.converToMatrix();
		auto color_intrinsics = intrinsic.converToMatrix();

		float scale_factor = 1.0;
		//float scale_factor = 0.01;
		mat4f scale = mat4f::scale({ scale_factor, scale_factor, scale_factor });
		mat4f rotation = mat4f::rotationY(180.) * mat4f::rotationX(90.);
		mat4f transform = mat4f::translation({ -0.5f, -1.f, 1.5f });
		mat4f depth_extrinsics = transform * rotation * scale;
		auto color_extrinsics = ml::mat4f::identity();

		_sensor_data_wrapper = std::make_unique<CalibrateSensorDataWrapper>(_depth_sensor,
																			depth_intrinsics, depth_extrinsics,
																			color_intrinsics, color_extrinsics);
	}
	else {
		std::cout << "could not connect to camera" << std::endl;
	}

	m_shaderManager.init(app.graphics);
	m_shaderManager.registerShader("shaders/pointCloud.hlsl", "pointCloud");
	m_constants.init(app.graphics);
}

void ShowKinectData::render(ml::Cameraf& camera)
{
	_sensor_data_wrapper->processFrame();
	auto end = std::chrono::system_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(end - _start_time).count();
	//if (elapsed > 1) {
		renderPoints(_frame);
		_frame++;
		_start_time = std::chrono::system_clock::now();
	//}	
	ConstantBuffer constants;
	constants.worldViewProj = camera.getViewProj();
	constants.modelColor = ml::vec4f(1.0f, 1.0f, 1.0f, 1.0f);

	m_constants.updateAndBind(constants, 0);
	m_shaderManager.bindShaders("pointCloud");
	m_constants.bind(0);
	m_pointCloud.render();
}

