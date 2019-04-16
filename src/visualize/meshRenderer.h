#pragma once
#include "../mLibInclude.h"
#include "i_showData.h"



class NormalShader{
public:
	ml::vec4f operator()(ml::vec3f normal);
};


class PhongShader{
	float _diffuse = 0.3f;
	float _specular = 0.4f;
	float _ambient = 0.3f;
	ml::vec3f _incomming_light_direction = { 3., -1., 5. };
	ml::vec3f _incomming_light_color = {1., 1., 1. };
public:
	ml::vec4f operator()(ml::vec3f normal);
};


class MeshRenderer
{
private:
	std::map<std::string, ml::TriMeshf> _meshes;
	ml::D3D11TriMesh _mesh;
	ml::D3D11ShaderManager _shaderManager;
	ml::D3D11ConstantBuffer<ConstantBuffer> _constants;
	ml::D3D11Buffer<ml::vec4f> _buffer;
	ml::GraphicsDevice * _graphics;
	std::function<ml::vec4f(ml::vec3f)> _shader;
public:
	void render(ml::Cameraf& camera);
	void insertMesh(std::string id, const ml::TriMeshf& mesh);
public:
	MeshRenderer(ml::ApplicationData &app, std::function<ml::vec4f(ml::vec3f)> shader);
};