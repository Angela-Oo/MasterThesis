#include "showKinectData.h"
#include "kinect/BinaryDumpReader.h"
#include "kinect/ImageReaderSensor.h"
#include "kinect/KinectSensor.h"
#include "kinect/PrimeSenseSensor.h"
#include "ext-depthcamera/sensorData.h"
#include <numeric>


void ShowKinectData::initMesh(ml::GraphicsDevice & graphics)
{
	MeshDataf meshData = MeshIOf::loadFromFile("scans/gates381.ply");
	ml::TriMeshf triMesh(meshData);

	std::vector<ml::TriMeshf> meshes;
	meshes.push_back(triMesh);

	ml::TriMeshf unifiedMesh = ml::meshutil::createUnifiedMesh(meshes);
	m_mesh.init(graphics, unifiedMesh);
}

void ShowKinectData::initPoints(ml::GraphicsDevice & graphics)
{
	auto lambdaPoints = [=](ml::vec3f& v) { v = ml::vec3f(-2.f*(float)rand() / RAND_MAX, -2.f*(float)rand() / RAND_MAX, (float)rand() / RAND_MAX); };
	std::vector<ml::vec3f> points(5000);
	std::for_each(points.begin(), points.end(), lambdaPoints);

	m_pointCloud.init(graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.01f), points));
}

void ShowKinectData::initKinectPoints(ml::GraphicsDevice & graphics)
{
	PrimeSenseSensor reader;
	try {
		reader.createFirstConnected();

		auto intrinsic = reader.getIntrinsics();

		reader.toggleNearMode();

		for (int i = 0; i < 10; i++) {
			reader.processDepth();
			reader.processColor();

			reader.recordFrame();
			reader.recordPointCloud();
		}
		

	

		std::vector<float> depth_data;
		std::vector<vec3f> skeleton;
		for (unsigned int i = 0; i < reader.getDepthHeight(); i++) {
			for (unsigned int j = 0; j < reader.getDepthWidth(); j++) {
				depth_data.push_back(reader.getDepth(j, i));
				skeleton.push_back(reader.depthToSkeleton(j, i));
			}
		}

		DepthImage32 depth_image(reader.getDepthWidth(), reader.getDepthHeight(), depth_data.data());

		SensorData::CalibrationData calibrationColor;
		SensorData::CalibrationData calibrationDepth;
		SensorData data;
		data.initDefault(reader.getColorWidth(), reader.getColorHeight(),
						 reader.getDepthWidth(), reader.getDepthHeight(),
						 calibrationColor, calibrationDepth);
		auto color_rgbx = reader.getColorRGBX();
		std::vector<vec3uc> color_data;
		for (unsigned int i = 0; i < reader.getColorHeight(); i++) {
			for (unsigned int j = 0; j < reader.getColorWidth(); j++) {
				const unsigned int idx = (i * reader.getColorWidth() + j) * 4;	//4 bytes per entry
				//vec4ui c = vec4ui(color_rgbx[idx + 0], color_rgbx[idx + 1], color_rgbx[idx + 2], color_rgbx[idx + 3]);
				color_data.push_back(vec3uc(color_rgbx[idx + 0], color_rgbx[idx + 1], color_rgbx[idx + 2]));
			}
		}

		data.addFrame(color_data.data(), reader.getDepthD16());// , intrinsic.converToMatrix());
		auto point_cloud = data.computePointCloud(0);

		auto points = point_cloud.m_points;

		//auto max = std::max_element(points.begin(), points.end(), [](vec3f& a, vec3f & b) {return b > a; });
		//auto min = std::min(points.begin(), points.end());
		//vec3f range = vec3f(*max);// max->x - min->x, max->y - min->y, max->z - min->z);
		////range.z = 1.;
		//std::for_each(points.begin(), points.end(), [&range](vec3f & p) { p = vec3f(p.x / range.x, p.y / range.y, p.z / range.z); });

		//auto average = std::accumulate(points.begin(), points.end(), vec3f(0., 0., 0.)) / points.size();
		//std::for_each(points.begin(), points.end(), [&average](vec3f & p) { p = p - average; });

		m_pointCloud.init(graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.01f), points));
		//data.loadFromFile();

		reader.writeDepthDataToFile("C:/Users/Angela/Meins/Studium/MasterThesis/data/test_depth.png");
		reader.writeColorDataToFile("C:/Users/Angela/Meins/Studium/MasterThesis/data/test_color.png");
		reader.saveRecordedPointCloud("C:/Users/Angela/Meins/Studium/MasterThesis/data/test_point.pt");

	}
	catch (...)
	{
		std::cout << "could not load file" << std::endl;
	}
}



void ShowKinectData::initSokratesPoints(ml::GraphicsDevice & graphics)
{
	ImageReaderSensor reader;
	reader.setBaseFilePath("D:/Studium/MasterThesis/input_data/sokrates-ps/");

	auto frame_number = [](unsigned int idx) {
		char frameNumber_c[10];
		sprintf_s(frameNumber_c, "%06d", idx);
		return std::string(frameNumber_c);
	};
	reader.setDepthFileName([&frame_number](unsigned int idx) { return "frame-" + frame_number(idx) + ".depth.png";});
	reader.setColorFileName([&frame_number](unsigned int idx) { return "frame-" + frame_number(idx) + ".color.png";});
	reader.setDepthIntrinsicsFileName("depthIntrinsics.txt");
	reader.setColorIntrinsicsFileName("colorIntrinsics.txt");

	try {
		reader.createFirstConnected();
		reader.setNumFrames(98);

		reader.processDepth();
		reader.processColor();

		mat4f depth_intrinsics_inv = reader.getDepthIntrinsics();
		depth_intrinsics_inv.invert();

		std::vector<ml::vec3f> points;
		for (unsigned int i = 0; i < reader.getDepthHeight(); i++) {
			for (unsigned int j = 0; j < reader.getDepthWidth(); j++) {
				float depth = reader.getDepth(j, i);
				//if (depth != 0.) {
				vec3f p(static_cast<float>(j), static_cast<float>(i), 1.);
				p = depth_intrinsics_inv * p;
				depth = (200. - depth) / 20.;
				p = p * depth;
				points.push_back(p);
				//}
			}
		}
		mat3f scale({ 0.5, 0., 0. }, { 0., 0.5, 0. }, { 0., 0., 0.5 });
		std::for_each(points.begin(), points.end(), [&scale](vec3f & p) { p = scale * p; });

		m_pointCloud.init(graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.01f), points));
	}
	catch (...)
	{
		std::cout << "could not load file" << std::endl;
	}
}

void ShowKinectData::initImagePoints(ml::GraphicsDevice & graphics)
{
	ImageReaderSensor reader;
	reader.setBaseFilePath("D:/Studium/MasterThesis/input_data/sokrates-ps/");
	auto frame_number = [](unsigned int idx) {
		char frameNumber_c[10];
		sprintf_s(frameNumber_c, "%06d", idx);
		return std::string(frameNumber_c);
	};
	reader.setDepthFileName([&frame_number](unsigned int idx) { return "frame-" + frame_number(idx) + ".depth.png"; });
	reader.setColorFileName([&frame_number](unsigned int idx) { return "frame-" + frame_number(idx) + ".color.png"; });
	reader.setDepthIntrinsicsFileName("depthIntrinsics.txt");
	reader.setColorIntrinsicsFileName("colorIntrinsics.txt");

	try {
		reader.createFirstConnected();
		reader.setNumFrames(98);
		
		reader.processDepth();
		reader.processColor();
		
		std::vector<unsigned short> depth_data;
		for (unsigned int i = 0; i < reader.getDepthHeight(); i++) {
			for (unsigned int j = 0; j < reader.getDepthWidth(); j++) {
				float depth = reader.getDepth(j, i);
				if (depth != 0.)
					depth = 200 - depth;
				depth_data.push_back(static_cast<unsigned short>(depth));				
			}
		}

		//DepthImage32 depth_image(reader.getDepthWidth(), reader.getDepthHeight(), depth_data.data());
		auto color_rgbx = reader.getColorRGBX();
		std::vector<vec3uc> color_data;
		for (unsigned int i = 0; i < reader.getColorHeight(); i++) {
			for (unsigned int j = 0; j < reader.getColorWidth(); j++) {
				const unsigned int idx = (i * reader.getColorWidth() + j) * 4;	//4 bytes per entry
				color_data.push_back(vec3uc(color_rgbx[idx + 0], color_rgbx[idx + 1], color_rgbx[idx + 2]));
			}
		}

		SensorData::CalibrationData calibrationColor(reader.getColorIntrinsics());
		SensorData::CalibrationData calibrationDepth(reader.getDepthIntrinsics());
		SensorData data;
		data.initDefault(reader.getColorWidth(), reader.getColorHeight(),
						 reader.getDepthWidth(), reader.getDepthHeight(),
						 calibrationColor, calibrationDepth);

		data.addFrame(color_data.data(), depth_data.data());
		auto point_cloud = data.computePointCloud(0);

		auto points = point_cloud.m_points;		


		auto average = std::accumulate(points.begin(), points.end(), vec3f(0., 0., 0.)) / points.size();
		mat4f center = mat4f::translation(-average);
		std::for_each(points.begin(), points.end(), [&center](vec3f & p) { p = center * p; });

		float scale_factor = 10.;
		mat3f scale({ scale_factor, 0., 0. }, { 0., scale_factor, 0. }, { 0., 0., scale_factor });
		std::for_each(points.begin(), points.end(), [&scale](vec3f & p) { p = scale * p; });

		mat4f rotation = mat4f::rotationX(90.) * mat4f::rotationY(180.);
		std::for_each(points.begin(), points.end(), [&rotation](vec3f & p) { p = rotation * p; });

		mat4f transform = mat4f::translation({ -2., -2., 1. });
		std::for_each(points.begin(), points.end(), [&transform](vec3f & p) { p = transform * p; });
		
		m_pointCloud.init(graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.001f), points));

		//reader.recordFrame();
	}
	catch (...)
	{
		std::cout << "could not load file" << std::endl;
	}
}

void ShowKinectData::init(ml::ApplicationData &app)
{
	//initMesh(app.graphics);

	//initPoints(app.graphics);

	//initKinectPoints(app.graphics);
	initImagePoints(app.graphics);

	m_shaderManager.init(app.graphics);
	m_shaderManager.registerShader("shaders/pointCloud.hlsl", "pointCloud");
	m_shaderManager.registerShaderWithGS("shaders/test.hlsl", "geometryShaderTest");

	m_constants.init(app.graphics);

	std::vector<vec4f> bufferData(m_mesh.getTriMesh().getVertices().size());
	for (size_t i = 0; i < m_mesh.getTriMesh().getVertices().size(); i++) {
		bufferData[i] = m_mesh.getTriMesh().getVertices()[i].color;
	}
	m_buffer.init(app.graphics, bufferData);

	D3D11Buffer<vec4f> tmp = m_buffer;
}

void ShowKinectData::render(ml::Cameraf& camera)
{
	ConstantBuffer constants;
	constants.worldViewProj = camera.getViewProj();
	constants.modelColor = ml::vec4f(1.0f, 1.0f, 1.0f, 1.0f);

	m_constants.updateAndBind(constants, 0);
	//app.graphics.castD3D11().getShaderManager().bindShaders("defaultBasic");
	m_shaderManager.bindShaders("geometryShaderTest");

	m_buffer.bindSRV(0);
	m_mesh.render();
	m_buffer.unbindSRV(0);

	m_shaderManager.bindShaders("pointCloud");
	m_constants.bind(0);
	m_pointCloud.render();

}

