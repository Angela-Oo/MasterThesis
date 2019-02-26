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

	void setDepthIntrinsicsFileName(const std::string depth_intrinsics_file_name) {
		m_DepthIntrinsicFileName = depth_intrinsics_file_name;
	}

	void setColorIntrinsicsFileName(const std::string color_intrinsics_file_name) {
		m_ColorIntrinsicFileName = color_intrinsics_file_name;
	}

	ml::mat4f getColorIntrinsics();
	ml::mat4f getDepthIntrinsics();

private:
	std::string m_BasePath;
	std::string m_ColorIntrinsicFileName;
	std::string m_DepthIntrinsicFileName;
	std::function<std::string(unsigned int)> m_create_depth_file_name;
	std::function<std::string(unsigned int)> m_create_color_file_name;
	unsigned int m_CurrentFrameNumberDepth;
	unsigned int m_CurrentFrameNumberColor;
	unsigned int m_NumFrames;


};

