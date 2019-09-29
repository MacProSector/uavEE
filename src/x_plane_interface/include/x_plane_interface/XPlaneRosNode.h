////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavAP.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
/**
 * @file	XPlaneRosNode.h
 * @author	Mirco Theile, mircot@illinois.edu
 * @date	[DD.MM.YYYY] 27.3.2018
 * @brief	X-Plane ROS Interface Node
 */

#ifndef XPLANEROSNODE_H
#define XPLANEROSNODE_H

#include <boost/property_tree/ptree.hpp>
#include <ros/ros.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <simulation_interface/actuation.h>

#include "xPlane/CHeaders/XPLM/XPLMDataAccess.h"

class IScheduler;

class XPlaneRosNode: public IAggregatableObject, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "ros_node";

	XPlaneRosNode();

	~XPlaneRosNode();

	static std::shared_ptr<XPlaneRosNode>
	create(const boost::property_tree::ptree& config);

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	bool
	run(RunStage stage) override;

	void
	enableAutopilot();

	void
	disableAutopilot();

private:

	void
	setDataRefs();

	void
	setActuationData(const simulation_interface::actuation& actuation);

	void
	publishSensorData();

	ObjectHandle<IScheduler> scheduler_;

	ros::Publisher sensorDataPublisher_;
	ros::Subscriber actuationSubscriber_;

	XPLMDataRef positionRefs_[3];
	XPLMDataRef velocityRefs_[3];
	XPLMDataRef airSpeedRef_;
	XPLMDataRef accelerationRefs_[3];
	XPLMDataRef attitudeRefs_[3];
	XPLMDataRef angleOfAttackRef_;
	XPLMDataRef angleOfSideslipRef_;
	XPLMDataRef angularRateRefs_[3];
	XPLMDataRef gpsFixRef_;
	XPLMDataRef autopilotActiveRef_[2];
	XPLMDataRef batteryVoltageRef_;
	XPLMDataRef batteryCurrentRef_;
	XPLMDataRef motorSpeedRef_;
	XPLMDataRef aileronLevelRef_;
	XPLMDataRef elevatorLevelRef_;
	XPLMDataRef rudderLevelRef_;
	XPLMDataRef throttleLevelRef_;
	XPLMDataRef windAltitudeRef_[3];
	XPLMDataRef windDirectionRef_[3];
	XPLMDataRef windSpeedRef_[3];
	XPLMDataRef windTurbulenceRef_[3];
	XPLMDataRef windShearDirectionRef_[3];
	XPLMDataRef windShearSpeedRef_[3];
	XPLMDataRef activeWindDirectionRef_;
	XPLMDataRef activeWindSpeedRef_;
	XPLMDataRef actuationRef_[3];

	unsigned int sequenceNr_;
	int sensorFrequency_;

	bool autopilotActive_;
};

#endif /* XPLANEROSNODE_H */
