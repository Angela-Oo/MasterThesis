#include "stdafx.h"
#include "meshRenderer.h"
#include "algo/surface_mesh/mesh_convertion.h"
#include "ext-freeimage/freeImageWrapper.h"

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

void MeshRenderer::saveCurrentWindowAsImage()
{
	auto image = _graphics->captureBackBufferColor();
	auto color = _graphics->castD3D11().captureBackBufferColor();
	auto depth = _graphics->castD3D11().captureBackBufferDepth();
	ml::FreeImageWrapper::saveImage("screenshot_color.png", color);
	ml::FreeImageWrapper::saveImage("screenshot_depth.png", ml::ColorImageR32G32B32A32(depth));/*
	FreeImageWrapper:: void saveImage(const std::string &filename, const Image& image, bool debugPrint = false) {
	image.saveAsBinaryMImage("test_image.mbinRGB");*/
}

void MeshRenderer::insertMesh(std::string id, const SurfaceMesh & mesh, ml::vec4f color, bool override)
{
	if (override || !keyExists(id)) {
		insertMesh(id, convertToTriMesh(mesh), color);
	}
}


void MeshRenderer::insertMesh(std::string id, const ml::TriMeshf& mesh, ml::vec4f color)
{
	_meshes[id] = D3D11MeshAndBuffer();
	
	std::vector<ml::vec4f> bufferData(mesh.getVertices().size());
	for (size_t i = 0; i < mesh.getVertices().size(); i++) {
		bufferData[i] = color;
	}
	_meshes[id].mesh.init(*_graphics, mesh);
	_meshes[id].buffer.init(*_graphics, bufferData);
}


void MeshRenderer::insertMesh(std::string id, const ml::TriMeshf& mesh)
{
	_meshes[id] = D3D11MeshAndBuffer();

	std::vector<ml::vec4f> bufferData(mesh.getVertices().size());
	for (size_t i = 0; i < mesh.getVertices().size(); i++) {
		bufferData[i] = mesh.getVertices()[i].color;
	}
	_meshes[id].mesh.init(*_graphics, mesh);
	_meshes[id].buffer.init(*_graphics, bufferData);
}

bool MeshRenderer::keyExists(std::string id)
{
	return (_meshes.find(id) != _meshes.end());
}

void MeshRenderer::removeMesh(std::string id)
{
	_meshes.erase(id);
}

void MeshRenderer::clear()
{
	_meshes.clear();
}

MeshRenderer::MeshRenderer(ml::GraphicsDevice * graphics)
{
	_graphics = graphics;
	_shaderManager.init(*graphics);
	_shaderManager.registerShaderWithGS("shaders/test.hlsl", "geometryShaderTest");
	//_shaderManager.registerShaderWithGS("shaders/phongShader.hlsl", "geometryShaderTest");
	_constants.init(*graphics);
}

