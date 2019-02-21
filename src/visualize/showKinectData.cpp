#include "showKinectData.h"



void ShowKinectData::init(ml::ApplicationData &app)
{
	MeshDataf meshData = MeshIOf::loadFromFile("scans/gates381.ply");
	ml::TriMeshf triMesh(meshData);

	std::vector<ml::TriMeshf> meshes;
	meshes.push_back(triMesh);

	ml::TriMeshf unifiedMesh = ml::meshutil::createUnifiedMesh(meshes);

	m_mesh.init(app.graphics, unifiedMesh);

	auto lambdaPoints = [=](ml::vec3f& v) { v = ml::vec3f(-2.f*(float)rand() / RAND_MAX, -2.f*(float)rand() / RAND_MAX, (float)rand() / RAND_MAX); };
	std::vector<ml::vec3f> points(5000);
	std::for_each(points.begin(), points.end(), lambdaPoints);

	m_pointCloud.init(app.graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.01f), points));

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

