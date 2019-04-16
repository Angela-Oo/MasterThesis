#include "meshRenderer.h"

void MeshRenderer::render(ml::Cameraf& camera)
{
	ConstantBuffer constants;
	constants.worldViewProj = camera.getViewProj();
	constants.modelColor = ml::vec4f(1.0f, 1.0f, 1.0f, 1.0f);

	_constants.updateAndBind(constants, 0);
	_shaderManager.bindShaders("geometryShaderTest");

	_buffer.bindSRV(0);
	_mesh.render();
	_buffer.unbindSRV(0); 
}

void MeshRenderer::insertMesh(std::string id, ml::TriMeshf mesh)
{
	_meshes[id] = mesh;

	std::vector<ml::TriMeshf> meshes;
	for(auto & m : _meshes)
		meshes.push_back(m.second);

	ml::TriMeshf unifiedMesh = ml::meshutil::createUnifiedMesh(meshes);
	_mesh.init(*_graphics, unifiedMesh);

	std::vector<ml::vec4f> bufferData(_mesh.getTriMesh().getVertices().size());
	for (size_t i = 0; i < _mesh.getTriMesh().getVertices().size(); i++) {
		bufferData[i] =_mesh.getTriMesh().getVertices()[i].color;
	}
	_buffer.init(*_graphics, bufferData);
}

MeshRenderer::MeshRenderer(ml::ApplicationData &app)
{
	_graphics = &app.graphics;
	_shaderManager.init(app.graphics);
	_shaderManager.registerShaderWithGS("shaders/test.hlsl", "geometryShaderTest");
	_constants.init(app.graphics);
}

