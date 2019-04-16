#include "pointsRenderer.h"
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

PointsRenderer::PointsRenderer(ml::ApplicationData &app)
{
	_graphics = &app.graphics;
	_shaderManager.init(app.graphics);
	_shaderManager.registerShader("shaders/pointCloud.hlsl", "pointCloud");
	_constants.init(app.graphics);
}