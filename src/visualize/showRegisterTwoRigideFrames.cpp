#include "showRegisterTwoRigideFrames.h"
#include <numeric>


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

void ShowTwoRigideRegisteredFrames::configImageReaderSensor(std::string filepath)
{
	_depth_sensor.setBaseFilePath(filepath);
	auto frame_number = [](unsigned int idx) {
		char frameNumber_c[10];
		sprintf_s(frameNumber_c, "%06d", idx);
		return std::string(frameNumber_c);
	};
	_depth_sensor.setDepthFileName([&frame_number](unsigned int idx) { return "frame-" + frame_number(idx) + ".depth.png"; });
	_depth_sensor.setColorFileName([&frame_number](unsigned int idx) { return "frame-" + frame_number(idx) + ".color.png"; });
	_depth_sensor.setDepthIntrinsicsFileName("depthIntrinsics.txt");
	_depth_sensor.setColorIntrinsicsFileName("colorIntrinsics.txt");

	_depth_sensor.createFirstConnected();
	_depth_sensor.setNumFrames(56);
	_depth_sensor.toggleNearMode();
}


std::vector<vec3f> ShowTwoRigideRegisteredFrames::processFrame()
{
	auto points = _rgbd_frame_to_point_cloud->addFrame();

	auto average = std::accumulate(points.begin(), points.end(), vec3f(0., 0., 0.)) / static_cast<float>(points.size());
	mat4f center = mat4f::translation(-average);
	float scale_factor = 5.;
	mat4f scale = mat4f::scale({ scale_factor, scale_factor, scale_factor });
	mat4f rotation = mat4f::rotationX(90.) * mat4f::rotationY(180.);
	mat4f transform = mat4f::translation({ -0.5f, -2.f, 1.2f });
	mat4f translate = transform * rotation * scale * center;
	std::for_each(points.begin(), points.end(), [&translate](vec3f & p) { p = translate * p; });

	_depth_sensor.recordFrame();
	return points;
}

void ShowTwoRigideRegisteredFrames::init(ml::ApplicationData &app)
{
	_graphics = &app.graphics;
	configImageReaderSensor("D:/Studium/MasterThesis/input_data/sokrates-ps/");	
	_rgbd_frame_to_point_cloud = std::make_unique<SensorDataWrapper>(_depth_sensor, _depth_sensor.getColorIntrinsics(), _depth_sensor.getDepthIntrinsics());

	auto points_frame_A = processFrame();
	std::vector<vec4f> color_frame_A(points_frame_A.size());
	std::fill(color_frame_A.begin(), color_frame_A.end(), RGBColor::Orange);
	m_pointCloudFrameA.init(app.graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.001f), points_frame_A, color_frame_A));

	for (int i = 0; i < 9; i++) {
		_depth_sensor.processDepth();
		_depth_sensor.processColor();
	}

	auto points_frame_B = processFrame();
	std::vector<vec4f> color_frame_B(points_frame_B.size());
	std::fill(color_frame_B.begin(), color_frame_B.end(), RGBColor::Green);
	m_pointCloudFrameB.init(app.graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.001f), points_frame_B, color_frame_B));

	m_shaderManager.init(app.graphics);
	m_shaderManager.registerShader("shaders/pointCloud.hlsl", "pointCloud");
	m_constants.init(app.graphics);
}

void ShowTwoRigideRegisteredFrames::render(ml::Cameraf& camera)
{
	ConstantBuffer constants;
	constants.worldViewProj = camera.getViewProj();

	m_constants.updateAndBind(constants, 0);
	m_shaderManager.bindShaders("pointCloud");
	m_constants.bind(0);
	m_pointCloudFrameA.render();
	m_pointCloudFrameB.render();
}


