#include "showRGBDData.h"
#include <numeric>
#include "input_reader/depth_image_reader.h"

using namespace ml;

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


void ShowRGBDImageData::init(ml::ApplicationData &app)
{
	_point_renderer = std::make_unique<PointsRenderer>(app);

	float scale_factor = 0.004;
	ml::mat4f scale = ml::mat4f::scale({ scale_factor, scale_factor, scale_factor });
	ml::mat4f rotation = ml::mat4f::rotationX(90.);
	ml::mat4f transform = ml::mat4f::translation({ -0.5f, -0.7f, 1.6f });
	ml::mat4f transformation_camera_extrinsics = transform * rotation * scale;
	_reader = std::make_unique<DepthImageReader>("D:/Studium/MasterThesis/input_data/sokrates-ps/", transformation_camera_extrinsics);

	_reader->processFrame();
	auto points = _reader->getPoints(0);
	_point_renderer->insertPoints("points", points, ml::RGBColor::Red);
}

void ShowRGBDImageData::render(ml::Cameraf& camera)
{
	_reader->processFrame();
	auto points = _reader->getPoints(_reader->frame());
	_point_renderer->insertPoints("points", points, ml::RGBColor::Red);
	_point_renderer->render(camera);
}


