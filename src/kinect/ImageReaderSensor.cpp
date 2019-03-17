#include "stdafx.h"

#include "ImageReaderSensor.h"
#include "core-base/baseImage.h"

using namespace ml;

mat4f loadIntrinsicFromFile(std::string filename)
{
	mat4f data = mat4f::identity();
	std::ifstream file(filename);
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			float value;
			file >> value;
			data(i, j) = value;
		}
	}
	return data;
}

ImageReaderSensor::ImageReaderSensor()
	: m_BasePath("")
{
	unsigned int windowWidth = 1280;
	unsigned int windowHeight = 1024;
	//init(windowWidth, windowHeight, 640, 480, 640, 480);
	init(windowWidth, windowHeight, 640, 480, 1280, 1024);

	//default path should be actually overwritten
	//m_BaseFilename = "../stanfordData/copyroom_png/";
	m_NumFrames = 0;
}

ImageReaderSensor::~ImageReaderSensor()
{

}


ml::mat4f ImageReaderSensor::getColorIntrinsics()
{
	return loadIntrinsicFromFile(m_BasePath + m_ColorIntrinsicFileName);
}

ml::mat4f ImageReaderSensor::getDepthIntrinsics()
{
	return loadIntrinsicFromFile(m_BasePath + m_DepthIntrinsicFileName);
}

HRESULT ImageReaderSensor::createFirstConnected()
{
	HRESULT hr = S_OK;

	mat4f intrinsics = loadIntrinsicFromFile(m_BasePath + m_DepthIntrinsicFileName);
	initializeIntrinsics(intrinsics(0, 0), intrinsics(1, 1), intrinsics(0, 2), intrinsics(1, 2));

	//what Qian-Yi / Vladlen tell us
	//float focalLengthX = 525.0f;
	//float focalLengthY = 525.0f;
	////float cx = 319.5f;
	////float cy = 239.5f;

	////what the oni framework gives us
	////float focalLengthX = 570.34f;
	////float focalLengthY = 570.34f;
	//float cx = 320.0f;
	//float cy = 240.0f;
	//initializeIntrinsics(focalLengthX, focalLengthY, cx, cy);

	m_CurrentFrameNumberColor = 0;
	m_CurrentFrameNumberDepth = 0;
	return hr;
}

HRESULT ImageReaderSensor::processDepth()
{
	HRESULT hr = S_OK;
	if (m_CurrentFrameNumberDepth >= m_NumFrames) {
		return S_FALSE;
	}
	std::cout << "Processing Depth Frame " << m_CurrentFrameNumberDepth << std::endl;
	std::string currFileName = m_BasePath + m_create_depth_file_name(m_CurrentFrameNumberDepth + 1);
	ml::DepthImage16 image;
	FreeImageWrapper::loadImage(currFileName, image);
	image.flipY();
	for (UINT i = 0; i < getDepthWidth() * getDepthHeight(); i++) {
		unsigned short depth = image.getData()[i];
		m_depthD16[i] = depth * 1000u;
	}
	m_CurrentFrameNumberDepth++;
	return hr;
}

HRESULT ImageReaderSensor::processColor()
{
	HRESULT hr = S_OK;
	if (m_CurrentFrameNumberColor >= m_NumFrames) {
		return S_FALSE;
	}

	bool readColor = false;

	if (readColor) {
		std::string currFileName = m_BasePath + m_create_color_file_name(m_CurrentFrameNumberDepth + 1);
		ml::ColorImageR32G32B32 image;
		FreeImageWrapper::loadImage(currFileName, image);
		image.flipY();
		for (UINT i = 0; i < getDepthWidth() * getDepthHeight(); i++) {
			vec3f c = image.getData()[i];
			c = 255.0f*c;

			m_colorRGBX[4*i+0] = (BYTE)c.x;
			m_colorRGBX[4*i+1] = (BYTE)c.y; 
			m_colorRGBX[4*i+2] = (BYTE)c.z;
			m_colorRGBX[4*i+3] = 255; 
		}
	}
	m_CurrentFrameNumberColor++;
	return hr;
}
