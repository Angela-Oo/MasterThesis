#pragma once
#include "../mLibInclude.h"
#include "i_showData.h"

class PointsRenderer
{
private:
	std::map<std::string, ml::D3D11TriMesh> _pointClouds;
	ml::D3D11ShaderManager _shaderManager;
	ml::D3D11ConstantBuffer<ConstantBuffer> _constants;
	ml::GraphicsDevice * _graphics;
public:
	void render(ml::Cameraf& camera);
	void insertPoints(std::string id, std::vector<ml::vec3f> points, ml::RGBColor color, float point_size = 0.001f);
	void insertLine(std::string id, std::vector<ml::vec3f> points1, std::vector<ml::vec3f> points2, ml::RGBColor color, float point_size = 0.001f);
	void insertPoints(std::string id, const ml::TriMeshf & points, ml::RGBColor color, float point_size = 0.001f);
	void insertLine(std::string id, const ml::TriMeshf & points1, const ml::TriMeshf & points2, ml::RGBColor color, float point_size = 0.001f);
	void removePoints(std::string id);
	void clear();
public:
	PointsRenderer(ml::ApplicationData &app);
};