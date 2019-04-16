#pragma once
#include "../mLibInclude.h"
#include "i_showData.h"

class MeshRenderer
{
private:
	std::map<std::string, ml::TriMeshf> _meshes;
	ml::D3D11TriMesh _mesh;
	ml::D3D11ShaderManager _shaderManager;
	ml::D3D11ConstantBuffer<ConstantBuffer> _constants;
	ml::D3D11Buffer<ml::vec4f> _buffer;
	ml::GraphicsDevice * _graphics;
public:
	void render(ml::Cameraf& camera);
	void insertMesh(std::string id, ml::TriMeshf mesh);
public:
	MeshRenderer(ml::ApplicationData &app);
};