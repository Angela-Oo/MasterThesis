#include "meshRenderer.h"


ml::vec4f NormalShader::operator()(ml::vec3f normal, ml::vec4f color)
{
	return ml::vec4f(normal);
}

ml::vec4f PhongShader::operator()(ml::vec3f normal, ml::vec4f color)
{
	ml::vec4f diffuse = _diffuse * (ml::vec3f::dot(normal, _incomming_light_direction.getNormalized())) * color;
	ml::vec4f ambient = _ambient * color;
	ml::vec4f phong_color = ambient + diffuse;
	return ml::vec4f(phong_color);
}


void MeshRenderer::render(ml::Cameraf& camera)
{
	ConstantBuffer constants;
	constants.worldViewProj = camera.getViewProj();
	constants.modelColor = ml::vec4f(1.0f, 1.0f, 1.0f, 1.0f);

	_constants.updateAndBind(constants, 0);
	_shaderManager.bindShaders("geometryShaderTest");

	for (auto & m : _meshes) {
		m.second.buffer.bindSRV(0);
		m.second.mesh.render();
		m.second.buffer.unbindSRV(0);
	}
}

void MeshRenderer::insertMesh(std::string id, const ml::TriMeshf& mesh, ml::vec4f color)
{
	_meshes[id] = D3D11MeshAndBuffer();

	std::vector<ml::vec4f> bufferData(mesh.getVertices().size());
	for (size_t i = 0; i < mesh.getVertices().size(); i++) {
		bufferData[i] = _shader(mesh.getVertices()[i].normal, color);
	}
	_meshes[id].mesh.init(*_graphics, mesh);
	_meshes[id].buffer.init(*_graphics, bufferData);
}

void MeshRenderer::removeMesh(std::string id)
{
	_meshes.erase(id);
}

void MeshRenderer::clear()
{
	_meshes.clear();
}

MeshRenderer::MeshRenderer(ml::ApplicationData &app, std::function<ml::vec4f(ml::vec3f, ml::vec4f)> shader)
	: _shader(shader)
{
	_graphics = &app.graphics;
	_shaderManager.init(app.graphics);
	_shaderManager.registerShaderWithGS("shaders/test.hlsl", "geometryShaderTest");
	_constants.init(app.graphics);
}

