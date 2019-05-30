#pragma once
#include "../mLibInclude.h"
#include "visualize/constantBuffer.h"
#include "algo/registration/i_registration.h"

#include "algo/mesh_simplification/mesh_simplification.h"

ml::vec3f PointToVec3f(const Point & p);

class PointsRenderer
{
private:
	std::map<std::string, ml::D3D11TriMesh> _pointClouds;
	ml::D3D11ShaderManager _shaderManager;
	ml::D3D11ConstantBuffer<ConstantBuffer> _constants;
	ml::GraphicsDevice * _graphics;
private:
	ml::TriMeshf createPointTriMesh(const SurfaceMesh & mesh, float point_size = 0.01f);
	ml::TriMeshf createPointTriMesh(const SurfaceMesh & mesh, ml::RGBColor color, float point_size = 0.01f);
	ml::TriMeshf createLineTriMesh(const SurfaceMesh & mesh, float point_size = 0.01f);
	ml::TriMeshf createLineTriMesh(const SurfaceMesh & mesh, ml::RGBColor color, float point_size = 0.01f);
	ml::TriMeshf createNormalTriMesh(const SurfaceMesh & mesh, float point_size = 0.01f);
	ml::TriMeshf createNormalTriMesh(const SurfaceMesh & mesh, ml::RGBColor color, float point_size = 0.01f);
public:
	void render(ml::Cameraf& camera);
	void insertMesh(std::string id, const SurfaceMesh & mesh, float point_size = 0.001f, bool draw_normals = false, bool override = true);
	void insertMesh(std::string id, const SurfaceMesh & mesh, ml::RGBColor color, float point_size = 0.001f, bool draw_normals = false, bool override = true);
	void insertPoints(std::string id, const SurfaceMesh & mesh, float point_size = 0.001f);
	void insertPoints(std::string id, const SurfaceMesh & mesh, ml::RGBColor color, float point_size = 0.001f);

	void insertPoints(std::string id, std::vector<Point> points, ml::RGBColor color, float point_size = 0.001f);
	void insertPoints(std::string id, std::vector<ml::vec3f> points, ml::RGBColor color, float point_size = 0.001f);
	void insertPoints(std::string id, const ml::TriMeshf & points, float point_size = 0.001f, bool draw_normals = false);
	void insertPoints(std::string id, const ml::TriMeshf & points, ml::RGBColor color, float point_size = 0.001f, bool draw_normals = false);
	//void insertLine(std::string id, std::vector<ml::vec3f> points1, std::vector<ml::vec3f> points2, ml::RGBColor color, float point_size = 0.001f);
	//void insertLine(std::string id, std::vector<Edge> edges, float point_size = 0.001f);	
	void insertLine(std::string id, const ml::TriMeshf & points1, const ml::TriMeshf & points2, ml::RGBColor color, float point_size = 0.001f);
	bool keyExists(std::string id);
	void removePoints(std::string id);
	void clear();
public:
	PointsRenderer(ml::ApplicationData &app);
};