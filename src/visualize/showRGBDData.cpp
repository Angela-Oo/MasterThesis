#include "showRGBDData.h"
#include <numeric>


void RenderMesh::init(ml::ApplicationData &app)
{
	MeshDataf meshData = MeshIOf::loadFromFile("scans/gates381.ply");
	ml::TriMeshf triMesh(meshData);

	std::vector<ml::TriMeshf> meshes;
	meshes.push_back(triMesh);

	ml::TriMeshf unifiedMesh = ml::meshutil::createUnifiedMesh(meshes);
	m_mesh.init(app.graphics, unifiedMesh);

	m_shaderManager.init(app.graphics);
	m_shaderManager.registerShaderWithGS("shaders/test.hlsl", "geometryShaderTest");

	m_constants.init(app.graphics);

	std::vector<vec4f> bufferData(m_mesh.getTriMesh().getVertices().size());
	for (size_t i = 0; i < m_mesh.getTriMesh().getVertices().size(); i++) {
		bufferData[i] = m_mesh.getTriMesh().getVertices()[i].color;
	}
	m_buffer.init(app.graphics, bufferData);

	D3D11Buffer<vec4f> tmp = m_buffer;
}

void RenderMesh::render(ml::Cameraf& camera)
{
	ConstantBuffer constants;
	constants.worldViewProj = camera.getViewProj();
	constants.modelColor = ml::vec4f(1.0f, 1.0f, 1.0f, 1.0f);

	m_constants.updateAndBind(constants, 0);
	m_shaderManager.bindShaders("geometryShaderTest");

	m_buffer.bindSRV(0);
	m_mesh.render();
	m_buffer.unbindSRV(0);
}


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

void configImageReaderSensor(ImageReaderSensor & reader, std::string filepath)
{
	reader.setBaseFilePath(filepath);
	auto frame_number = [](unsigned int idx) {
		char frameNumber_c[10];
		sprintf_s(frameNumber_c, "%06d", idx);
		return std::string(frameNumber_c);
	};
	reader.setDepthFileName([&frame_number](unsigned int idx) { return "frame-" + frame_number(idx) + ".depth.png"; });
	reader.setColorFileName([&frame_number](unsigned int idx) { return "frame-" + frame_number(idx) + ".color.png"; });
	reader.setDepthIntrinsicsFileName("depthIntrinsics.txt");
	reader.setColorIntrinsicsFileName("colorIntrinsics.txt");
}


std::vector<vec3f> ShowRGBDImageData::initSokratesPoints()
{
	configImageReaderSensor(_reader, { "D:/Studium/MasterThesis/input_data/sokrates-ps/" });
	_reader.createFirstConnected();
	_reader.setNumFrames(98);

	PointsFromDepthData pointGenerator(_reader, _reader.getColorIntrinsics(), _reader.getDepthIntrinsics());
	auto points = pointGenerator.getPointsWithoutSensorData();

	mat3f scale({ 0.5, 0., 0. }, { 0., 0.5, 0. }, { 0., 0., 0.5 });
	std::for_each(points.begin(), points.end(), [&scale](vec3f & p) { p = scale * p; });

	return points;
	//m_pointCloud.init(graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.01f), points));
}


std::vector<vec3f> ShowRGBDImageData::initImagePoints()
{
	configImageReaderSensor(_reader, { "D:/Studium/MasterThesis/input_data/sokrates-ps/" });
	_reader.createFirstConnected();
	_reader.setNumFrames(98);
	_reader.toggleNearMode();
	return processFrame();
	//reader.recordFrame();
}

std::vector<vec3f> ShowRGBDImageData::processFrame()
{
	_rgbd_frame_to_point_cloud = std::make_unique<PointsFromDepthData>(_reader, _reader.getColorIntrinsics(), _reader.getDepthIntrinsics());
	auto points = _rgbd_frame_to_point_cloud->getPoints().m_points;

	auto average = std::accumulate(points.begin(), points.end(), vec3f(0., 0., 0.)) / points.size();
	mat4f center = mat4f::translation(-average);
	float scale_factor = 5.;
	mat3f scale({ scale_factor, 0., 0. }, { 0., scale_factor, 0. }, { 0., 0., scale_factor });
	mat4f rotation = mat4f::rotationX(90.) * mat4f::rotationY(180.);
	mat4f transform = mat4f::translation({ -0.5, -2., 1.2 });
	mat4f translate = transform * rotation * scale * center;
	std::for_each(points.begin(), points.end(), [&translate](vec3f & p) { p = translate * p; });

	return points;
}

void ShowRGBDImageData::init(ml::ApplicationData &app)
{
	//initKinectPoints(app.graphics);
	m_pointCloud.init(app.graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.001f), initImagePoints()));

	m_shaderManager.init(app.graphics);
	m_shaderManager.registerShader("shaders/pointCloud.hlsl", "pointCloud");
	m_constants.init(app.graphics);
}

void ShowRGBDImageData::render(ml::Cameraf& camera)
{
	//auto points = processFrame();
	//m_pointCloud.init(_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.001f), points));

	ConstantBuffer constants;
	constants.worldViewProj = camera.getViewProj();
	constants.modelColor = ml::vec4f(1.0f, 1.0f, 1.0f, 1.0f);

	m_constants.updateAndBind(constants, 0);
	m_shaderManager.bindShaders("pointCloud");
	m_constants.bind(0);
	m_pointCloud.render();
}

