#include "showKinectData.h"
//#include "kinect/BinaryDumpReader.h"
#include "kinect/KinectSensor.h"

#include "ext-depthcamera/sensorData.h"
#include <numeric>

void ShowKinectData::processFrame()
{
	//auto points = _sensor_data_wrapper->addFrame(2);
	auto points = _sensor_data_wrapper->get3DPoints(2);

	auto average = std::accumulate(points.begin(), points.end(), vec3f(0., 0., 0.)) / static_cast<float>(points.size());
	mat4f center = mat4f::translation(-average);
	float scale_factor = 1.;
	//float scale_factor = 0.01;
	mat4f scale = mat4f::scale({ scale_factor, scale_factor, scale_factor });
	mat4f rotation = mat4f::rotationX(90.) * mat4f::rotationZ(-90.);
	mat4f transform = mat4f::translation({ -0.5f, -2.f, 1.2f });
	mat4f translate = transform * rotation * scale * center;
	std::for_each(points.begin(), points.end(), [&translate](vec3f & p) { p = translate * p; });

	m_pointCloud.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.002f), points));
}



void ShowKinectData::init(ml::ApplicationData &app)
{
	_graphics = &app.graphics;
	if (_depth_sensor.createFirstConnected() == S_OK)
	{
		auto intrinsic = _depth_sensor.getIntrinsics();
		//_depth_sensor.toggleNearMode();

		_sensor_data_wrapper = std::make_unique<SensorDataWrapper>(_depth_sensor);// intrinsics ??
		processFrame();

		//reader.writeDepthDataToFile("C:/Users/Angela/Meins/Studium/MasterThesis/data/test_depth.png");
		//reader.writeColorDataToFile("C:/Users/Angela/Meins/Studium/MasterThesis/data/test_color.png");
		//reader.saveRecordedPointCloud("C:/Users/Angela/Meins/Studium/MasterThesis/data/test_point.pt");
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
	processFrame();
	ConstantBuffer constants;
	constants.worldViewProj = camera.getViewProj();
	constants.modelColor = ml::vec4f(1.0f, 1.0f, 1.0f, 1.0f);

	m_constants.updateAndBind(constants, 0);
	m_shaderManager.bindShaders("pointCloud");
	m_constants.bind(0);
	m_pointCloud.render();
}

