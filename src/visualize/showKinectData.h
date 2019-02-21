#pragma once
#include "../mLibInclude.h"

struct ConstantBuffer
{
	ml::mat4f worldViewProj;
	ml::vec4f modelColor;
};

class ShowKinectData
{
private:
	void initMesh(ml::GraphicsDevice & graphics);
	void initPoints(ml::GraphicsDevice & graphics);
public:
	void init(ml::ApplicationData &app);
	void render(ml::Cameraf& camera);
private:
	ml::D3D11TriMesh m_mesh, m_pointCloud;

	ml::D3D11ShaderManager m_shaderManager;

	ml::D3D11ConstantBuffer<ConstantBuffer> m_constants;

	ml::D3D11Buffer<vec4f> m_buffer;
};