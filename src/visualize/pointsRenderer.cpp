#include "stdafx.h"
#include "pointsRenderer.h"
#include "algo/registration/hsv_to_rgb.h"
using namespace ml;


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

void PointsRenderer::insertPoints(std::string id, std::vector<ml::vec3f> points, ml::RGBColor color, float point_size)
{
	// render point clouds
	std::vector<ml::vec4f> color_frame(points.size());
	std::fill(color_frame.begin(), color_frame.end(), color);
	_pointClouds[id].init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(point_size), points, color_frame));
}

void PointsRenderer::insertLine(std::string id, std::vector<ml::vec3f> points1, std::vector<ml::vec3f> points2, ml::RGBColor color, float point_size)
{
	// render point clouds
	std::vector<TriMeshf> meshes;
	for (int i = 0; i < points1.size(); i++) {
		meshes.push_back(ml::Shapesf::line(points1[i], points2[i], color, point_size));
	}
	_pointClouds[id].init(*_graphics, ml::meshutil::createUnifiedMesh(meshes));
}

void PointsRenderer::insertLine(std::string id, std::vector<Edge> edges, float point_size)
{
	std::vector<TriMeshf> meshes;
	double max_cost = 0.;
	std::for_each(edges.begin(), edges.end(), [&max_cost](const Edge & e) { if (e.cost > max_cost) max_cost = e.cost; });
	std::cout << "max cost " << max_cost << std::endl;
	for (auto & e : edges) {
		auto cost = e.cost;
		if (max_cost > 0.)
			cost = e.cost / max_cost;
		auto color = errorToRGB(cost);
		meshes.push_back(ml::Shapesf::line(e.source_point, e.target_point, color, point_size));
	}
	_pointClouds[id].init(*_graphics, ml::meshutil::createUnifiedMesh(meshes));
}

void PointsRenderer::insertPoints(std::string id, const TriMeshf & points, ml::RGBColor color, float point_size, bool draw_normals)
{
	// render point clouds
	std::vector<ml::vec4f> color_frame;// (points.getVertices().size());
	//std::fill(color_frame.begin(), color_frame.end(), color);

	for (auto & p : points.m_vertices)
	{
		color_frame.push_back(p.color);
	}
	std::vector<ml::vec3f> vertices;
	for (auto & p : points.getVertices())
		vertices.push_back(p.position);

	if (!draw_normals) {
		_pointClouds[id].init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(point_size), vertices, color_frame));
	}
	else {
		auto mesh = ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(point_size), vertices, color_frame);

		std::vector<TriMeshf> meshes;
		meshes.push_back(mesh);
		std::vector<ml::vec3f> normals;
		for (auto & p : points.getVertices()) {
			meshes.push_back(ml::Shapesf::line(p.position, p.position + p.normal * 0.05, p.color, point_size * 0.2));
		}
		_pointClouds[id].init(*_graphics, ml::meshutil::createUnifiedMesh(meshes));
	}
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