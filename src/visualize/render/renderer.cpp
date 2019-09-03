#include "renderer.h"

void Renderer::render(ml::Cameraf& camera)
{
	_mesh_renderer->render(camera);
	_point_renderer->render(camera);
}

void Renderer::saveCurrentWindowAsImage(std::string folder, std::string filename)
{
	try {
		_mesh_renderer->saveCurrentWindowAsImage(folder, filename);
	}
	catch (...) {
		std::cout << "could not save image " << filename << std::endl;
	}
}

void Renderer::insert(std::string id, const SurfaceMesh & mesh, RenderMode mode, ml::RGBColor color, bool replace, float thickness)
{
	if (mode == RenderMode::MESH) {
		_mesh_renderer->insertMesh(id, mesh, color.toVec4f(), replace);
		_mesh_ids.insert(id);
	}
	else if (mode == RenderMode::EDGE) {
		_point_renderer->insertMesh(id, mesh, color, thickness, false, replace);
		_point_ids.insert(id);
	}
	else {
		_point_renderer->insertPoints(id, mesh, color, thickness, replace);
		_point_ids.insert(id);
	}
}

void Renderer::insert(std::string id, const SurfaceMesh & mesh, RenderMode mode, bool replace, float thickness)
{
	if (mode == RenderMode::MESH) {
		_mesh_renderer->insertMesh(id, mesh);
		_mesh_ids.insert(id);
	}
	else if (mode == RenderMode::EDGE) {
		_point_renderer->insertMesh(id, mesh, thickness, replace);
		_point_ids.insert(id);
	}
	else {
		_point_renderer->insertPoints(id, mesh, thickness, replace);
		_point_ids.insert(id);
	}
}

void Renderer::remove(std::string id)
{
	if (_point_ids.find(id) != _point_ids.end()) {
		_point_renderer->removePoints(id);
		_point_ids.erase(id);
	}
	else if (_mesh_ids.find(id) != _mesh_ids.end()) {
		_mesh_renderer->removeMesh(id);
		_mesh_ids.erase(id);
	}
}


Renderer::Renderer(ml::GraphicsDevice * graphics)
{
	_mesh_renderer = std::make_unique<MeshRenderer>(graphics);
	_point_renderer = std::make_unique<PointsRenderer>(graphics);
}