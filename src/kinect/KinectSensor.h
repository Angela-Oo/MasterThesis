#pragma once

/************************************************************************/
/* Kinect Sensor (the old version of a Kinect)                          */
/************************************************************************/

#include "DepthSensor.h"
#include <NuiApi.h>
#include <NuiSkeleton.h>

class KinectSensor : public DepthSensor
{
public:
	KinectSensor(unsigned int windowWidth, unsigned int windowHeight);
	//! Constructor; allocates CPU memory and creates handles

	//! Destructor; releases allocated ressources
	~KinectSensor();

	//! Initializes the sensor
	HRESULT createFirstConnected();

	//! gets the next depth frame
	HRESULT processDepth();

	//! maps the color to depth data and copies depth and color data to the GPU
	HRESULT processColor();

	//! toggles near mode if possible (only available on a windows Kinect)
	HRESULT toggleNearMode();

	//! Toggle enable auto white balance
	HRESULT toggleAutoWhiteBalance();
	

private:
	INuiSensor*		m_pNuiSensor;

	static const NUI_IMAGE_RESOLUTION   cDepthResolution = NUI_IMAGE_RESOLUTION_640x480;
	static const NUI_IMAGE_RESOLUTION   cColorResolution = NUI_IMAGE_RESOLUTION_640x480;

	HANDLE			m_hNextDepthFrameEvent;
	HANDLE			m_pDepthStreamHandle;
	HANDLE			m_hNextColorFrameEvent;
	HANDLE			m_pColorStreamHandle;

	LONG*			m_colorCoordinates;		// for mapping depth to color

	LONG			m_colorToDepthDivisor;

	// to prevent drawing until we have data for both streams
	bool			m_bDepthReceived;
	bool			m_bColorReceived;

	bool			m_bDepthImageIsUpdated;
	bool			m_bDepthImageCameraIsUpdated;
	bool			m_bNormalImageCameraIsUpdated;

	bool			m_kinect4Windows;
	bool			m_bNearMode;
};
