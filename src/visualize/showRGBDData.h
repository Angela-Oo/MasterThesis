#pragma once
#include "../mLibInclude.h"
#include "i_showData.h"
#include "kinect/SensorDataWrapper.h"
#include "kinect/ImageReaderSensor.h"
#include "input_reader/i_reader.h"
#include "render/pointsRenderer.h"


class ShowRGBDImageData : public IShowData
{
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;
	void key(UINT key) override {};
private:
	std::unique_ptr<IReader> _reader;
	std::unique_ptr<PointsRenderer> _point_renderer;
public:
	ShowRGBDImageData() = default;
	~ShowRGBDImageData() = default;
};