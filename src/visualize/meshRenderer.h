#pragma once
#include "../mLibInclude.h"
#include "visualize/constantBuffer.h"


struct D3D11MeshAndBuffer
{
	ml::D3D11TriMesh mesh;
	ml::D3D11Buffer<ml::vec4f> buffer;
};

class MeshRenderer
{
private:
	std::map<std::string, D3D11MeshAndBuffer> _meshes;
	ml::D3D11ShaderManager _shaderManager;
	ml::D3D11ConstantBuffer<ConstantBuffer> _constants;
	ml::GraphicsDevice * _graphics;
public:
	void render(ml::Cameraf& camera);
	void insertMesh(std::string id, const ml::TriMeshf& mesh, ml::vec4f color = { 1., 1., 1., 1. });
	bool keyExists(std::string id);
	void removeMesh(std::string id);
	void clear();
public:
	MeshRenderer(ml::ApplicationData &app);
};