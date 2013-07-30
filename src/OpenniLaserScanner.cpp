/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
#include "OpenniLaserScanner.h"

using namespace openni;

namespace OpenniLaser
{

class StreamListener;

struct LaserScanMsgInternal
{
	LaserScanMsgInternal(OpenniLaserScanner* pLaserScanner) :
		m_pDevice(NULL), m_pDepthStream(NULL), m_pListener(NULL), m_pStreamListener(NULL), m_pOpenniLaserScanner(pLaserScanner)
		{}

	void Raise()
	{
		if (m_pListener != NULL)
			m_pListener->readyForNextData(m_pOpenniLaserScanner);
	}
	bool m_oniOwner;
	Device* m_pDevice;
	VideoStream* m_pDepthStream;

	OpenniLaserScanner::Listener* m_pListener;

	StreamListener* m_pStreamListener;

	OpenniLaserScanner* m_pOpenniLaserScanner;
};

class StreamListener : public VideoStream::NewFrameListener
{
public:
	StreamListener(LaserScanMsgInternal* pClosestPoint) : m_pClosestPoint(pClosestPoint)
	{}
	virtual void onNewFrame(VideoStream& stream)
	{
		m_pClosestPoint->Raise();
	}
private:
	LaserScanMsgInternal* m_pClosestPoint;
};

OpenniLaserScanner::OpenniLaserScanner(const char* uri)
{
	m_pInternal = new LaserScanMsgInternal(this);

	m_pInternal->m_pDevice = new Device;
	m_pInternal->m_oniOwner = true;

	OpenNI::initialize();
	Status rc = m_pInternal->m_pDevice->open(uri);
	if (rc != STATUS_OK)
	{
		printf("Open device failed:\n%s\n", OpenNI::getExtendedError());
		return;
	}
	initialize();
}

OpenniLaserScanner::OpenniLaserScanner(openni::Device* pDevice)
{
	m_pInternal = new LaserScanMsgInternal(this);

	m_pInternal->m_pDevice = pDevice;
	m_pInternal->m_oniOwner = false;

	OpenNI::initialize();

	if (pDevice != NULL)
	{
		initialize();
	}
}

void OpenniLaserScanner::initialize()
{
	m_pInternal->m_pStreamListener = NULL;
	m_pInternal->m_pListener = NULL;

	m_pInternal->m_pDepthStream = new VideoStream;
	Status rc = m_pInternal->m_pDepthStream->create(*m_pInternal->m_pDevice, SENSOR_DEPTH);
	if (rc != STATUS_OK)
	{
		printf("Created failed\n%s\n", OpenNI::getExtendedError());
		return;
	}

	m_pInternal->m_pStreamListener = new StreamListener(m_pInternal);

	rc = m_pInternal->m_pDepthStream->start();
	if (rc != STATUS_OK)
	{
		printf("Start failed:\n%s\n", OpenNI::getExtendedError());
	}

	m_pInternal->m_pDepthStream->addNewFrameListener(m_pInternal->m_pStreamListener);
}

OpenniLaserScanner::~OpenniLaserScanner()
{
	if (m_pInternal->m_pDepthStream != NULL)
	{
		m_pInternal->m_pDepthStream->removeNewFrameListener(m_pInternal->m_pStreamListener);

		m_pInternal->m_pDepthStream->stop();
		m_pInternal->m_pDepthStream->destroy();

		delete m_pInternal->m_pDepthStream;
	}

	if (m_pInternal->m_pStreamListener != NULL)
	{
		delete m_pInternal->m_pStreamListener;
	}

	if (m_pInternal->m_oniOwner)
	{
		if (m_pInternal->m_pDevice != NULL)
		{
			m_pInternal->m_pDevice->close();

			delete m_pInternal->m_pDevice;
		}
	}

	OpenNI::shutdown();


	delete m_pInternal;
}

bool OpenniLaserScanner::isValid() const
{
	if (m_pInternal == NULL)
		return false;
	if (m_pInternal->m_pDevice == NULL)
		return false;
	if (m_pInternal->m_pDepthStream == NULL)
		return false;
	if (!m_pInternal->m_pDepthStream->isValid())
		return false;

	return true;
}

Status OpenniLaserScanner::setListener(Listener& listener)
{
	m_pInternal->m_pListener = &listener;
	return STATUS_OK;
}
void OpenniLaserScanner::resetListener()
{
	m_pInternal->m_pListener = NULL;
}

sensor_msgs::LaserScan
OpenniLaserScanner::getNextData(VideoFrameRef& rawFrame)
{
	Status rc = m_pInternal->m_pDepthStream->readFrame(&rawFrame);
	if (rc != STATUS_OK)
	{
		printf("readFrame failed\n%s\n", OpenNI::getExtendedError());
	}

	int width = rawFrame.getWidth();
	int height = rawFrame.getHeight();

	unsigned int num_readings = width;
	double laser_frequency = 30;
	double ranges[num_readings];
	double intensities[num_readings];

	DepthPixel* pDepth = (DepthPixel*)rawFrame.getData();

	//populate the LaserScan message
	//Xtion 58°
	sensor_msgs::LaserScan scan;
	scan.header.stamp = ros::Time::now();
	scan.header.frame_id = "openni2_laserscan";
	scan.angle_min = -0.505;
	scan.angle_max = 0.505;
	scan.angle_increment = (scan.angle_max - scan.angle_min) / width;
	scan.time_increment = (1 / laser_frequency) / (num_readings);
	scan.range_min = 0.0;
	scan.range_max = 10.0;
	scan.ranges.resize(num_readings);
	//scan.intensities.resize(num_readings);

	int start_pos = (width*height)/2;

	for(unsigned int i = 0; i < num_readings/2; ++i){
		int indexRev = num_readings -1 -i;

		float angle_radius_fix = std::cos( scan.angle_max - scan.angle_increment*i );

		if(i == num_readings/2 -1){

			scan.ranges[i] = (((float)pDepth[start_pos+i])/1000);
			scan.ranges[indexRev] = (((float)pDepth[start_pos + indexRev])/1000);
		}
		else{
			scan.ranges[i] = (((float)pDepth[start_pos+i])/1000) / angle_radius_fix;
			scan.ranges[indexRev] = (((float)pDepth[start_pos + indexRev])/1000) / angle_radius_fix;
		}

	}
	//printf("Openni Laser Scan L: %f C: %f R: %f \n",scan.ranges[20], scan.ranges[width/2], scan.ranges[num_readings-1]);
	return scan;
}

}


