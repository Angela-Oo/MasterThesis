#include "stdafx.h"
#include "points_renderer.h"
#include "algo/registration/util/hsv_to_rgb.h"
using namespace ml;

ml::vec3f PointToVec3f(const Point & p)
{
	return ml::vec3f(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
}

void PointsRenderer::render(ml::Cameraf& camera)
{
	ConstantBuffer constants;
	constants.worldViewProj = camera.getViewProj();
	constants.modelColor = ml::vec4f(1.0f, 1.0f, 1.0f, 1.0f);

	_constants.updateAndBind(constants, 0);
	_shaderManager.bindShaders("pointCloud");
	_constants.bind(0);
	for (auto& point_cloud : _pointClouds) {
		point_cloud.second.render();
	}
}


TriMeshf PointsRenderer::createPointTriMesh(const SurfaceMesh & mesh, float point_size)
{
	auto vertex_colors = mesh.property_map<vertex_descriptor, ml::vec4f>("v:color").first;
	std::vector<ml::vec3f> positions;
	std::vector<ml::vec4f> color_frame;
	for (auto & v : mesh.vertices()) {
		auto p = mesh.point(v);
		positions.push_back(PointToVec3f(p));
		color_frame.push_back(vertex_colors[v]);
	}
	return ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(point_size), positions, color_frame);
}

TriMeshf PointsRenderer::createPointTriMesh(const SurfaceMesh & mesh, ml::RGBColor color, float point_size)
{
	std::vector<ml::vec3f> positions;
	for (auto & v : mesh.vertices()) {
		positions.push_back(PointToVec3f(mesh.point(v)));
	}
	std::vector<ml::vec4f> color_frame(mesh.number_of_vertices());
	std::fill(color_frame.begin(), color_frame.end(), color);
	return ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(point_size), positions, color_frame);
}


TriMeshf PointsRenderer::createLineTriMesh(const SurfaceMesh & mesh, float point_size)
{
	std::vector<TriMeshf> meshes;
	auto edge_colors = mesh.property_map<edge_descriptor, ml::vec4f>("e:color").first;
	for (auto & e : mesh.edges()) {
		auto he = mesh.halfedge(e);
		meshes.push_back(ml::Shapesf::line(PointToVec3f(mesh.point(mesh.source(he))), 
										   PointToVec3f(mesh.point(mesh.target(he))),
										   edge_colors[e], point_size));
	}
	return ml::meshutil::createUnifiedMesh(meshes);
}

TriMeshf PointsRenderer::createLineTriMesh(const SurfaceMesh & mesh, ml::RGBColor color, float point_size)
{
	std::vector<TriMeshf> meshes;
	for (auto & e : mesh.edges()) {
		auto he = mesh.halfedge(e);
		meshes.push_back(ml::Shapesf::line(PointToVec3f(mesh.point(mesh.source(he))),
										   PointToVec3f(mesh.point(mesh.target(he))), 
										   color, point_size));
	}
	return ml::meshutil::createUnifiedMesh(meshes);
}


TriMeshf PointsRenderer::createNormalTriMesh(const SurfaceMesh & mesh, float point_size)
{
	std::vector<TriMeshf> meshes;
	auto vertex_colors = mesh.property_map<vertex_descriptor, ml::vec4f>("v:color").first;
	auto normals = mesh.property_map<vertex_descriptor, Vector>("v:normal").first;
	std::vector<ml::vec3f> normal_positions;
	for (auto & v : mesh.vertices()) {
		auto p = mesh.point(v);
		auto normal = PointToVec3f(p + (normals[v] * 0.05));
		meshes.push_back(ml::Shapesf::line(PointToVec3f(p), normal, vertex_colors[v], point_size));
	}
	return ml::meshutil::createUnifiedMesh(meshes);
}

TriMeshf PointsRenderer::createNormalTriMesh(const SurfaceMesh & mesh, ml::RGBColor color, float point_size)
{
	std::vector<TriMeshf> meshes;
	auto normals = mesh.property_map<vertex_descriptor, Vector>("v:normal").first;
	std::vector<ml::vec3f> normal_positions;
	for (auto & v : mesh.vertices()) {
		auto p = mesh.point(v);
		auto normal = PointToVec3f(p + (normals[v] * 0.05));
		meshes.push_back(ml::Shapesf::line(PointToVec3f(p), normal, color, point_size));
	}
	return ml::meshutil::createUnifiedMesh(meshes);
}

void PointsRenderer::insertMesh(std::string id, const SurfaceMesh & mesh, bool replace, float point_size, bool draw_normals)
{
	if (replace || !keyExists(id)) {
		std::vector<TriMeshf> meshes;
		// draw edges
		meshes.push_back(createLineTriMesh(mesh, point_size));

		// draw vertices
		meshes.push_back(createPointTriMesh(mesh, point_size * 10.f));

		// draw normals
		if (draw_normals) {
			meshes.push_back(createNormalTriMesh(mesh, point_size));
		}
		_pointClouds[id].init(*_graphics, ml::meshutil::createUnifiedMesh(meshes));
	}
}

void PointsRenderer::insertMesh(std::string id, const SurfaceMesh & mesh, ml::RGBColor color, bool replace,	float point_size, bool draw_normals)
{
	if (replace || !keyExists(id)) {
		std::vector<TriMeshf> meshes;
		// draw edges
		meshes.push_back(createLineTriMesh(mesh, color, point_size));
		// draw normals
		if (draw_normals) {
			meshes.push_back(createNormalTriMesh(mesh, color, point_size));
		}
		_pointClouds[id].init(*_graphics, ml::meshutil::createUnifiedMesh(meshes));
	}
}

void PointsRenderer::insertPoints(std::string id, const SurfaceMesh & mesh, bool replace, float point_size)
{
	if (replace || !keyExists(id)) {
		_pointClouds[id].init(*_graphics, createPointTriMesh(mesh, point_size));
	}
}

void PointsRenderer::insertPoints(std::string id, const SurfaceMesh & mesh, ml::RGBColor color, bool replace, float point_size)
{
	if (replace || !keyExists(id)) {
		_pointClouds[id].init(*_graphics, createPointTriMesh(mesh, color, point_size));
	}
}

void PointsRenderer::insertPoints(std::string id, const std::vector<Point> & points, ml::RGBColor color, float point_size)
{
	std::vector<ml::vec3f> positions;
	for (auto & p : points) {
		positions.push_back(PointToVec3f(p));
	}
	std::vector<ml::vec4f> color_frame(points.size());
	std::fill(color_frame.begin(), color_frame.end(), color);
	_pointClouds[id].init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(point_size), positions, color_frame));
}

void PointsRenderer::insertPoints(std::string id, const std::vector<ml::vec3f> & points, ml::RGBColor color, float point_size)
{
	// render point clouds
	std::vector<ml::vec4f> color_frame(points.size());
	std::fill(color_frame.begin(), color_frame.end(), color);
	_pointClouds[id].init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(point_size), points, color_frame));
}

bool PointsRenderer::keyExists(std::string id)
{
	return (_pointClouds.find(id) != _pointClouds.end());
}

void PointsRenderer::removePoints(std::string id)
{
	_pointClouds.erase(id);
}

void PointsRenderer::clear()
{
	_pointClouds.clear();
}

PointsRenderer::PointsRenderer(ml::GraphicsDevice * graphics)
{
	_graphics = graphics;
	_shaderManager.init(*graphics);
	_shaderManager.registerShader("shaders/pointCloud.hlsl", "pointCloud");
	_constants.init(*graphics);
}