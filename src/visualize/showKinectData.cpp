#include "showKinectData.h"
#include "kinect/BinaryDumpReader.h"
#include "kinect/ImageReaderSensor.h"
#include "kinect/DX11DepthSensor.h"


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
	ImageReaderSensor reader;
	reader.setBaseFilePath("D:/Studium/MasterThesis/input_data/desk_1/rgbd-scenes/desk/desk_1/");
	reader.setDepthFileName([](unsigned int idx) { return "desk_1_" + std::to_string(idx) + "_depth.png"; });
	reader.setColorFileName([](unsigned int idx) { return "desk_1_" + std::to_string(idx) + ".png"; });

	try {
		reader.createFirstConnected();
		reader.setNumFrames(98);

		//DX11Sensor sensor;
		//sensor.OnD3D11CreateDevice(graphics, &reader);

		//sensor.OnD3D11DestroyDevice();
		auto intrinsic = reader.getIntrinsics();
		auto dh = reader.getDepthHeight();
		reader.processDepth();
		reader.processColor();		
	}
	catch (...)
	{
		std::cout << "could not load file" << std::endl;
	}
}

void ShowKinectData::init(ml::ApplicationData &app)
{
	initMesh(app.graphics);

	initPoints(app.graphics);

	initKinectPoints(app.graphics);

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

