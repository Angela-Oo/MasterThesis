#include "showRegisterTwoRigideFrames.h"
#include <numeric>
#include "algo/icp-ceres.h"
#include "algo/eigen_quaternion.h"
#include "ext-depthcamera/calibratedSensorData.h"
using namespace Eigen;

Vector3d vec3f_to_vector3d(const ml::vec3f &vec)
{
	return Vector3d(vec.x, vec.y, vec.z);
}

ml::vec3f vector3d_to_vec3f(const Vector3d &vec)
{
	return ml::vec3f(vec[0], vec[1], vec[2]);
}

std::vector<Vector3d> vector_vec3f_to_vector_vector3d(std::vector<ml::vec3f> & vec)
{
	std::vector<Vector3d> converted_vec;
	std::transform(vec.begin(), vec.end(),
				   std::back_inserter(converted_vec), &vec3f_to_vector3d);
	return converted_vec;
}

std::vector<ml::vec3f> vector_vector3d_to_vector_vec3f(std::vector<Vector3d> & vec)
{
	std::vector<ml::vec3f> converted_vec;
	std::transform(vec.begin(), vec.end(),
				   std::back_inserter(converted_vec), &vector3d_to_vec3f);
	return converted_vec;
}

Vector4d to_vector4d(Vector3d vec)
{
	return Vector4d(vec[0], vec[1], vec[2], 1);
}
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


std::vector<ml::vec3f> ShowTwoRigideRegisteredFrames::processFrame()
{
	auto points = _rgbd_frame_to_point_cloud->addFrame(5);

	auto average = std::accumulate(points.begin(), points.end(), ml::vec3f(0., 0., 0.)) / static_cast<float>(points.size());
	ml::mat4f center = ml::mat4f::translation(-average);
	float scale_factor = 5.;
	ml::mat4f scale = ml::mat4f::scale({ scale_factor, scale_factor, scale_factor });
	ml::mat4f rotation = ml::mat4f::rotationX(90.) * ml::mat4f::rotationY(180.);
	ml::mat4f transform = ml::mat4f::translation({ -0.5f, -2.f, 1.2f });
	ml::mat4f translate = transform * rotation * scale * center;
	std::for_each(points.begin(), points.end(), [&translate](ml::vec3f & p) { p = translate * p; });

	_depth_sensor.recordFrame();
	return points;
}

void ShowTwoRigideRegisteredFrames::renderPoints(std::vector<ml::vec3f> & points_frame_A, std::vector<ml::vec3f> & points_frame_B)
{
	std::vector<ml::vec4f> color_frame_A(points_frame_A.size());
	std::fill(color_frame_A.begin(), color_frame_A.end(), ml::RGBColor::Orange);
	m_pointCloudFrameA.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.001f), points_frame_A, color_frame_A));

	std::vector<ml::vec4f> color_frame_B(points_frame_B.size());
	std::fill(color_frame_B.begin(), color_frame_B.end(), ml::RGBColor::Green);
	m_pointCloudFrameB.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.001f), points_frame_B, color_frame_B));
}

void ShowTwoRigideRegisteredFrames::init(ml::ApplicationData &app)
{
	_graphics = &app.graphics;
	m_shaderManager.init(app.graphics);
	m_shaderManager.registerShader("shaders/pointCloud.hlsl", "pointCloud");
	m_constants.init(app.graphics);
	configImageReaderSensor("C:/Users/Angela/Meins/Studium/MasterThesis/data/sokrates-ps/");

	float scale_factor = 0.004;
	ml::mat4f scale = ml::mat4f::scale({ scale_factor, scale_factor, scale_factor });
	ml::mat4f rotation = ml::mat4f::rotationX(90.) * ml::mat4f::rotationY(180.);
	//ml::mat4f transform = ml::mat4f::translation({ -0.5f, -2.f, 1.2f });
	ml::mat4f depth_extrinsics = /*transform */ rotation * scale;
	//auto depth_extrinsics = ml::mat4f::identity();
	auto sensor_data = std::make_unique<CalibrateSensorDataWrapper>(_depth_sensor, 
																	_depth_sensor.getDepthIntrinsics(), depth_extrinsics,
																	_depth_sensor.getColorIntrinsics(), ml::mat4f::identity());
	
	for(int i = 0; i < 10; i++)
		sensor_data->processFrame();
	auto points_frame_A = sensor_data->getPoints(0);
	auto points_frame_B = sensor_data->getPoints(2);


	//_rgbd_frame_to_point_cloud = std::make_unique<SensorDataWrapper>(_depth_sensor, _depth_sensor.getColorIntrinsics(), _depth_sensor.getDepthIntrinsics());
	//
	//auto points_frame_A = processFrame();
	//for (int i = 0; i < 2; i++) {
	//	_depth_sensor.processDepth();
	//	_depth_sensor.processColor();
	//}
	//auto points_frame_B = processFrame();

	auto points_A = vector_vec3f_to_vector_vector3d(points_frame_A);
	auto points_B = vector_vec3f_to_vector_vector3d(points_frame_B);
	auto icp = ICP_Ceres::pointToPoint_SophusSE3(points_A, points_B);

	Eigen::Matrix4d transform = icp.matrix();
	std::for_each(points_B.begin(), points_B.end(), [&](Vector3d & p) { auto x = transform * to_vector4d(p); p = x.head<3>();  });
	points_frame_A = vector_vector3d_to_vector_vec3f(points_A);
	points_frame_B = vector_vector3d_to_vector_vec3f(points_B);

	renderPoints(points_frame_A, points_frame_B);
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


