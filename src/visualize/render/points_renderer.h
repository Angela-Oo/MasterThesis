#pragma once
#include "mLibInclude.h"
#include "algo/registration/interface/i_registration.h"
#include "algo/remeshing/mesh_simplification.h"
#include "visualize/constantBuffer.h"

ml::vec3f PointToVec3f(const Point & p);

class PointsRenderer
{
private:
	std::map<std::string, ml::D3D11TriMesh> _pointClouds;
	ml::D3D11ShaderManager _shaderManager;
	ml::D3D11ConstantBuffer<ConstantBuffer> _constants;
	ml::GraphicsDevice * _graphics;
private:
	ml::TriMeshf createPointTriMesh(const SurfaceMesh & mesh, float point_size = 0.001f);
	ml::TriMeshf createPointTriMesh(const SurfaceMesh & mesh, ml::RGBColor color, float point_size = 0.001f);
	ml::TriMeshf createLineTriMesh(const SurfaceMesh & mesh, float point_size = 0.001f);
	ml::TriMeshf createLineTriMesh(const SurfaceMesh & mesh, ml::RGBColor color, float point_size = 0.001f);
	ml::TriMeshf createNormalTriMesh(const SurfaceMesh & mesh, float point_size = 0.001f);
	ml::TriMeshf createNormalTriMesh(const SurfaceMesh & mesh, ml::RGBColor color, float point_size = 0.001f);
public:
	void render(ml::Cameraf& camera);
	
	void insertMesh(std::string id, const SurfaceMesh & mesh, bool replace, float point_size = 0.001f, bool draw_normals = false);
	void insertMesh(std::string id, const SurfaceMesh & mesh, ml::RGBColor color, bool replace, float point_size = 0.001f, bool draw_normals = false);
	
	void insertPoints(std::string id, const SurfaceMesh & mesh, bool replace, float point_size = 0.001f);
	void insertPoints(std::string id, const SurfaceMesh & mesh, ml::RGBColor color, bool replace, float point_size = 0.001f);

	void insertPoints(std::string id, const std::vector<Point> & points, ml::RGBColor color, float point_size = 0.001f);
	void insertPoints(std::string id, const std::vector<ml::vec3f> & points, ml::RGBColor color, float point_size = 0.001f);
	
	bool keyExists(std::string id);
	void removePoints(std::string id);
	void clear();
public:
	PointsRenderer(ml::GraphicsDevice * graphics);
};