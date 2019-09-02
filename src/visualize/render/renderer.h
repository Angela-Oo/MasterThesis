#pragma once

#include "mLibInclude.h"
#include "pointsRenderer.h"
#include "meshRenderer.h"
#include <memory>
#include <set>

enum class RenderMode
{
	MESH,
	EDGE,
	POINT
};

class Renderer {
	std::unique_ptr<PointsRenderer> _point_renderer;
	std::unique_ptr<MeshRenderer> _mesh_renderer;

	std::set<std::string> _point_ids;
	std::set<std::string> _mesh_ids;
public:
	void saveCurrentWindowAsImage(std::string folder, std::string filename);
public:
	void insert(std::string id, const SurfaceMesh & mesh, RenderMode mode, ml::RGBColor color, bool replace, float thickness = 0.005f);
	void insert(std::string id, const SurfaceMesh & mesh, RenderMode mode, bool replace, float thickness = 0.005f);
	void remove(std::string id);

	void render(ml::Cameraf& camera);
public:
	Renderer(ml::GraphicsDevice * graphics);
};
