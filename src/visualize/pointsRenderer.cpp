#include "stdafx.h"
#include "pointsRenderer.h"
#include "algo/registration/hsv_to_rgb.h"
using namespace ml;

ml::vec3f PointToVec3f(const Point & p)
{
	return ml::vec3f(p.x(), p.y(), p.z());
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
	auto normals = mesh.property_map<vertex_descriptor, Direction>("v:normal").first;
	std::vector<ml::vec3f> normal_positions;
	for (auto & v : mesh.vertices()) {
		auto p = mesh.point(v);
		auto normal = PointToVec3f(p + (normals[v].vector() * 0.1));
		meshes.push_back(ml::Shapesf::line(PointToVec3f(p), normal, vertex_colors[v], point_size));
	}
	return ml::meshutil::createUnifiedMesh(meshes);
}

TriMeshf PointsRenderer::createNormalTriMesh(const SurfaceMesh & mesh, ml::RGBColor color, float point_size)
{
	std::vector<TriMeshf> meshes;
	auto normals = mesh.property_map<vertex_descriptor, Direction>("v:normal").first;
	std::vector<ml::vec3f> normal_positions;
	for (auto & v : mesh.vertices()) {
		auto p = mesh.point(v);
		auto normal = PointToVec3f(p + (normals[v].vector() * 0.1));
		meshes.push_back(ml::Shapesf::line(PointToVec3f(p), normal, color, point_size));
	}
	return ml::meshutil::createUnifiedMesh(meshes);
}

void PointsRenderer::insertMesh(std::string id, const SurfaceMesh & mesh, float point_size, bool draw_normals, bool override)
{
	if (override || !keyExists(id)) {
		std::vector<TriMeshf> meshes;
		// draw edges
		meshes.push_back(createLineTriMesh(mesh, point_size));

		// draw vertices
		meshes.push_back(createPointTriMesh(mesh, point_size * 10.));

		// draw normals
		if (draw_normals) {
			meshes.push_back(createNormalTriMesh(mesh, point_size));
		}
		_pointClouds[id].init(*_graphics, ml::meshutil::createUnifiedMesh(meshes));
	}
}

void PointsRenderer::insertMesh(std::string id,
								const SurfaceMesh & mesh,
								ml::RGBColor color,
								float point_size, 
								bool draw_normals,
								bool override)
{
	if (override || !keyExists(id)) {
		std::vector<TriMeshf> meshes;
		// draw edges
		meshes.push_back(createLineTriMesh(mesh, color, point_size));
		// draw normals
		if (draw_normals) {
			meshes.push_back(createNormalTriMesh(mesh, point_size));
		}
		_pointClouds[id].init(*_graphics, ml::meshutil::createUnifiedMesh(meshes));
	}
}

void PointsRenderer::insertPoints(std::string id, const SurfaceMesh & mesh, float point_size)
{
	_pointClouds[id].init(*_graphics, createPointTriMesh(mesh, point_size));
}

void PointsRenderer::insertPoints(std::string id, const SurfaceMesh & mesh, ml::RGBColor color, float point_size)
{
	_pointClouds[id].init(*_graphics, createPointTriMesh(mesh, color, point_size));
}

void PointsRenderer::insertPoints(std::string id, std::vector<Point> points, ml::RGBColor color, float point_size)
{
	std::vector<ml::vec3f> positions;
	for (auto & p : points) {
		positions.push_back(ml::vec3f(p.x(), p.y(), p.z()));
	}
	std::vector<ml::vec4f> color_frame(points.size());
	std::fill(color_frame.begin(), color_frame.end(), color);
	_pointClouds[id].init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(point_size), positions, color_frame));
}

void PointsRenderer::insertPoints(std::string id, std::vector<ml::vec3f> points, ml::RGBColor color, float point_size)
{
	// render point clouds
	std::vector<ml::vec4f> color_frame(points.size());
	std::fill(color_frame.begin(), color_frame.end(), color);
	_pointClouds[id].init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(point_size), points, color_frame));
}
//
//void PointsRenderer::insertLine(std::string id, std::vector<ml::vec3f> points1, std::vector<ml::vec3f> points2, ml::RGBColor color, float point_size)
//{
//	// render point clouds
//	std::vector<TriMeshf> meshes;
//	for (int i = 0; i < points1.size(); i++) {
//		meshes.push_back(ml::Shapesf::line(points1[i], points2[i], color, point_size));
//	}
//	_pointClouds[id].init(*_graphics, ml::meshutil::createUnifiedMesh(meshes));
//}
//
//void PointsRenderer::insertLine(std::string id, std::vector<Edge> edges, float point_size)
//{
//	std::vector<TriMeshf> meshes;
//	for (auto & e : edges) {
//		auto cost = e.cost;
//		auto color = errorToRGB(cost);
//		meshes.push_back(ml::Shapesf::line(e.source_point, e.target_point, color, point_size));
//	}
//	_pointClouds[id].init(*_graphics, ml::meshutil::createUnifiedMesh(meshes));
//}

void PointsRenderer::insertPoints(std::string id, const TriMeshf & points, float point_size, bool draw_normals)
{
	std::vector<ml::vec4f> color_frame;
	std::vector<ml::vec3f> vertices;
	for (auto & p : points.m_vertices)
	{
		color_frame.push_back(p.color);
		vertices.push_back(p.position);
	}	

	if (!draw_normals) {
		_pointClouds[id].init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(point_size), vertices, color_frame));
	}
	else {
		auto mesh = ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(point_size), vertices, color_frame);

		std::vector<TriMeshf> meshes;
		meshes.push_back(mesh);
		std::vector<ml::vec3f> normals;
		for (auto & p : points.getVertices()) {
			meshes.push_back(ml::Shapesf::line(p.position, p.position + p.normal * 0.02, p.color, point_size * 0.2));
		}
		_pointClouds[id].init(*_graphics, ml::meshutil::createUnifiedMesh(meshes));
	}
}

void PointsRenderer::insertPoints(std::string id, const ml::TriMeshf & points, ml::RGBColor color, float point_size, bool draw_normals)
{
	std::vector<ml::vec4f> color_frame;
	std::vector<ml::vec3f> vertices;
	for (auto & p : points.m_vertices)
	{
		color_frame.push_back(color);
		vertices.push_back(p.position);
	}
	
	_pointClouds[id].init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(point_size), vertices, color_frame));
}

void PointsRenderer::insertLine(std::string id, const TriMeshf & points1, const TriMeshf & points2, ml::RGBColor color, float point_size)
{
	// render point clouds
	std::vector<TriMeshf> meshes;
	for (int i = 0; i < points1.getVertices().size(); i++) {
		meshes.push_back(ml::Shapesf::line(points1.getVertices()[i].position, points2.getVertices()[i].position, color, point_size));
	}
	_pointClouds[id].init(*_graphics, ml::meshutil::createUnifiedMesh(meshes));
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

PointsRenderer::PointsRenderer(ml::ApplicationData &app)
{
	_graphics = &app.graphics;
	_shaderManager.init(app.graphics);
	_shaderManager.registerShader("shaders/pointCloud.hlsl", "pointCloud");
	_constants.init(app.graphics);
}