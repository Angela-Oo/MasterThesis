#include "meshRenderer.h"


ml::vec4f NormalShader::operator()(ml::vec3f normal)
{
	return ml::vec4f(normal);
}

ml::vec4f PhongShader::operator()(ml::vec3f normal)
{
	ml::vec3f diffuse = _diffuse * (ml::vec3f::dot(normal, _incomming_light_direction.getNormalized())) * _incomming_light_color;
	ml::vec3f ambient = _ambient * _incomming_light_color;
	ml::vec3f color = ambient + diffuse;
	return ml::vec4f(color);
}


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

void MeshRenderer::insertMesh(std::string id, const ml::TriMeshf& mesh)
{
	_meshes[id] = mesh;

	std::vector<ml::TriMeshf> meshes;
	for(auto & m : _meshes)
		meshes.push_back(m.second);

	ml::TriMeshf unifiedMesh = ml::meshutil::createUnifiedMesh(meshes);
	_mesh.init(*_graphics, unifiedMesh);

	std::vector<ml::vec4f> bufferData(_mesh.getTriMesh().getVertices().size());
	for (size_t i = 0; i < _mesh.getTriMesh().getVertices().size(); i++) {
		bufferData[i] = _shader(_mesh.getTriMesh().getVertices()[i].normal);// { 1., 0., 0., 0.5 };// _mesh.getTriMesh().getVertices()[i].color;
	}
	_buffer.init(*_graphics, bufferData);
}

MeshRenderer::MeshRenderer(ml::ApplicationData &app, std::function<ml::vec4f(ml::vec3f)> shader)
	: _shader(shader)
{
	_graphics = &app.graphics;
	_shaderManager.init(app.graphics);
	_shaderManager.registerShaderWithGS("shaders/test.hlsl", "geometryShaderTest");
	_constants.init(app.graphics);
}

