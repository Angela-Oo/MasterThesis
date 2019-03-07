#pragma once

/************************************************************************/
/* Reads binary dump data from .sensor files                            */
/************************************************************************/

#include "DepthSensor.h"
#include "ext-depthcamera/calibratedSensorData.h"

class BinaryDumpReader : public DepthSensor
{
public:

	//! Constructor
	BinaryDumpReader(std::string filename, unsigned int windowWidth, unsigned int windowHeight);

	//! Destructor; releases allocated ressources
	~BinaryDumpReader();

	//! initializes the sensor
	HRESULT createFirstConnected();

	//! reads the next depth frame
	HRESULT processDepth();

	HRESULT processColor()	{
		//everything done in process depth since order is relevant (color must be read first)
		return S_OK;
	}

	HRESULT BinaryDumpReader::toggleNearMode()	{
		return S_OK;
	}

	//! Toggle enable auto white balance
	HRESULT toggleAutoWhiteBalance() {
		return S_OK;
	}

	bool isKinect4Windows()	{
		return true;
	}

	ml::mat4f getRigidTransform() const {
		if (m_CurrFrame-1 >= m_data.m_trajectory.size()) throw MLIB_EXCEPTION("invalid trajectory index " + std::to_string(m_CurrFrame-1));
		return m_data.m_trajectory[m_CurrFrame-1];
	}
private:
	//! deletes all allocated data
	void releaseData();

	ml::CalibratedSensorData m_data;
	
	unsigned int	m_NumFrames;
	unsigned int	m_CurrFrame;

	std::string m_filename;
	unsigned int m_windowWidth;
	unsigned int m_windowHeight;
};
