#pragma once

/************************************************************************/
/* Reads data from a set of image sequence                              */
/************************************************************************/

#include "DepthSensor.h"

#include <string>

class ImageReaderSensor : public DepthSensor
{
public:
	ImageReaderSensor();
	~ImageReaderSensor();

	HRESULT createFirstConnected();

	HRESULT processDepth();

	HRESULT processColor();

	HRESULT toggleAutoWhiteBalance()
	{
		HRESULT hr = S_OK;

		return hr;
	}

	void setBaseFilePath(const std::string& basePath) {
		m_BasePath = basePath;
	}

	void setNumFrames(unsigned int i) {
		m_NumFrames = i;
	}

	unsigned int getNumFrames() {
		return m_NumFrames;
	}

	void setDepthFileName(const std::function<std::string(unsigned int)> create_depth_file_name) {
		m_create_depth_file_name = create_depth_file_name;
	}

	void setColorFileName(const std::function<std::string(unsigned int)> create_depth_file_name) {
		m_create_color_file_name = create_depth_file_name;
	}

private:
	std::string m_BasePath;
	std::function<std::string(unsigned int)> m_create_depth_file_name;
	std::function<std::string(unsigned int)> m_create_color_file_name;
	unsigned int m_CurrentFrameNumberDepth;
	unsigned int m_CurrentFrameNumberColor;
	unsigned int m_NumFrames;


};

