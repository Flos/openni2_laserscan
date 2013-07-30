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
#ifndef _MW_OPENNI_LASER_H_
#define _MW_OPENNI_LASER_H_

#include <OpenNI.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>

#ifdef _OPENNI_LASER_H_
#define MW_CP_API ONI_API_EXPORT
#else
#define MW_CP_API ONI_API_IMPORT
#endif


namespace openni
{
	class Device;
}

namespace OpenniLaser
{

struct LaserScanMsgInternal;

class MW_CP_API OpenniLaserScanner
{
public:
	class Listener
	{
	public:
		virtual void readyForNextData(OpenniLaserScanner*) = 0;
	};

	OpenniLaserScanner(const char* uri = NULL);
	OpenniLaserScanner(openni::Device* pDevice);
	~OpenniLaserScanner();

	bool isValid() const;

	openni::Status setListener(Listener& listener);
	void resetListener();

	sensor_msgs::LaserScan getNextData(openni::VideoFrameRef& rawFrame);
private:
	void initialize();

	LaserScanMsgInternal* m_pInternal;
};

}

#endif // _MW_CLOSEST_POINT_H_
