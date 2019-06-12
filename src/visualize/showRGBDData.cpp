#include "stdafx.h"
#include "showRGBDData.h"
#include <numeric>
#include "input_reader/depth_image_reader.h"


void ShowRGBDImageData::init(ml::ApplicationData &app)
{
	_point_renderer = std::make_unique<PointsRenderer>(&app.graphics);

	float scale_factor = 0.004;
	ml::mat4f scale = ml::mat4f::scale({ scale_factor, scale_factor, scale_factor });
	ml::mat4f rotation = ml::mat4f::rotationX(90.);
	ml::mat4f transform = ml::mat4f::translation({ -0.5f, -0.7f, 1.6f });
	ml::mat4f transformation_camera_extrinsics = transform * rotation * scale;
	_reader = std::make_unique<DepthImageReader>("D:/Studium/MasterThesis/input_data/sokrates-ps/", transformation_camera_extrinsics);

	_reader->processFrame();
	auto points = _reader->getPoints(0);
	_point_renderer->insertPoints("points", points, ml::RGBColor::Red);
}

void ShowRGBDImageData::render(ml::Cameraf& camera)
{
	_reader->processFrame();
	auto points = _reader->getPoints(_reader->frame());
	_point_renderer->insertPoints("points", points, ml::RGBColor::Red);
	_point_renderer->render(camera);
}


