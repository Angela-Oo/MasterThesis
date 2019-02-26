#include "showKinectData.h"
//#include "kinect/BinaryDumpReader.h"
#include "kinect/KinectSensor.h"
#include "kinect/PrimeSenseSensor.h"
#include "ext-depthcamera/sensorData.h"
#include <numeric>


void ShowKinectData::initKinectPoints(ml::GraphicsDevice & graphics)
{
	PrimeSenseSensor reader;
	try {
		reader.createFirstConnected();

		auto intrinsic = reader.getIntrinsics();
		reader.toggleNearMode();

		PointsFromDepthData pointGenerator(reader); // intrinsics ??
		auto point_cloud = pointGenerator.getPoints();
		auto points = point_cloud.m_points;
		m_pointCloud.init(graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.01f), points));
		//data.loadFromFile();

		//reader.writeDepthDataToFile("C:/Users/Angela/Meins/Studium/MasterThesis/data/test_depth.png");
		//reader.writeColorDataToFile("C:/Users/Angela/Meins/Studium/MasterThesis/data/test_color.png");
		//reader.saveRecordedPointCloud("C:/Users/Angela/Meins/Studium/MasterThesis/data/test_point.pt");
	}
	catch (...)
	{
		std::cout << "could not load file" << std::endl;
	}
}

void ShowKinectData::init(ml::ApplicationData &app)
{
	initKinectPoints(app.graphics);

	m_shaderManager.init(app.graphics);
	m_shaderManager.registerShader("shaders/pointCloud.hlsl", "pointCloud");
	m_constants.init(app.graphics);
}

void ShowKinectData::render(ml::Cameraf& camera)
{
	ConstantBuffer constants;
	constants.worldViewProj = camera.getViewProj();
	constants.modelColor = ml::vec4f(1.0f, 1.0f, 1.0f, 1.0f);

	m_constants.updateAndBind(constants, 0);
	m_shaderManager.bindShaders("pointCloud");
	m_constants.bind(0);
	m_pointCloud.render();
}

