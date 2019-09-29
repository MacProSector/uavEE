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
 * @file	XPlaneRosNode.cpp
 * @author	Mirco Theile, mircot@illinois.edu
 * @date	[DD.MM.YYYY] 27.3.2018
 * @brief	X-Plane ROS Interface Node
 */

#include <cmath>
#include <uavAP/API/ap_ext/latLongToUTM.h>
#include <uavAP/Core/LinearAlgebra.h>
#include <uavAP/Core/Scheduler/IScheduler.h>
#include <uavAP/Core/Logging/APLogger.h>
#include <simulation_interface/sensor_data.h>

#include "xPlane/CHeaders/XPLM/XPLMDefs.h"
#include "xPlane/CHeaders/XPLM/XPLMDataAccess.h"
#include "xPlane/CHeaders/XPLM/XPLMGraphics.h"
#include "x_plane_interface/XPlaneRosNode.h"

XPlaneRosNode::XPlaneRosNode() :
		sequenceNr_(0), sensorFrequency_(100), autopilotActive_(false)
{
	setDataRefs();
}

XPlaneRosNode::~XPlaneRosNode()
{
}

std::shared_ptr<XPlaneRosNode>
XPlaneRosNode::create(const boost::property_tree::ptree& config)
{
	return std::make_shared<XPlaneRosNode>();
}

void
XPlaneRosNode::notifyAggregationOnUpdate(const Aggregator& agg)
{
	scheduler_.setFromAggregationIfNotSet(agg);
}

bool
XPlaneRosNode::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!scheduler_.isSet())
		{
			APLOG_ERROR << "XPlaneRosNode: Scheduler Missing.";
			return true;
		}

		ros::VP_string config;
		ros::init(config, "xplane_interface");

		break;
	}
	case RunStage::NORMAL:
	{
		ros::NodeHandle nh;

		sensorDataPublisher_ = nh.advertise<simulation_interface::sensor_data>("sensor_data", 20);
		actuationSubscriber_ = nh.subscribe("actuation", 20, &XPlaneRosNode::setActuationData,
				this);

		auto scheduler = scheduler_.get();

		if (!scheduler)
		{
			APLOG_ERROR << "XPlaneRosNode: Scheduler Missing.";
			return true;
		}

		scheduler->schedule(std::bind(&XPlaneRosNode::publishSensorData, this), Milliseconds(0),
				Milliseconds(1000 / sensorFrequency_));

		break;
	}
	default:
	{
		break;
	}
	}

	return false;
}

void
XPlaneRosNode::enableAutopilot()
{
	if (!autopilotActive_)
	{
		autopilotActive_ = true;
		XPLMSetDatai(autopilotActiveRef_[0], 1);
		XPLMSetDatai(autopilotActiveRef_[1], 1);

		std::cout << "XPlaneRosNode: Autopilot On." << std::endl;
	}
}

void
XPlaneRosNode::disableAutopilot()
{
	if (autopilotActive_)
	{
		autopilotActive_ = false;
		XPLMSetDatai(autopilotActiveRef_[0], 0);
		XPLMSetDatai(autopilotActiveRef_[1], 0);

		std::cout << "XPlaneRosNode: Autopilot Off." << std::endl;
	}

	/* TEST */
	double x, y, z;
	XPLMWorldToLocal(40.05, -88.50, 2000, &x, &y, &z);
	XPLMSetDatad(XPLMFindDataRef("sim/flightmodel/position/local_x"), x);
	XPLMSetDatad(XPLMFindDataRef("sim/flightmodel/position/local_y"), y);
	XPLMSetDatad(XPLMFindDataRef("sim/flightmodel/position/local_z"), z);
	/* TEST */
}

void
XPlaneRosNode::setDataRefs()
{
	positionRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/latitude");
	positionRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/longitude");
	positionRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/elevation");

	velocityRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/local_vx");
	velocityRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/local_vy");
	velocityRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/local_vz");

	airSpeedRef_ = XPLMFindDataRef("sim/flightmodel/position/true_airspeed");

	accelerationRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/local_ax");
	accelerationRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/local_ay");
	accelerationRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/local_az");

	attitudeRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/phi");
	attitudeRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/theta");
	attitudeRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/psi");

	angleOfAttackRef_ = XPLMFindDataRef("sim/flightmodel2/misc/AoA_angle_degrees");
	angleOfSideslipRef_ = XPLMFindDataRef("sim/flightmodel/position/beta");

	angularRateRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/P");
	angularRateRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/Q");
	angularRateRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/R");

	gpsFixRef_ = XPLMFindDataRef("sim/cockpit2/radios/actuators/gps_power");
	autopilotActiveRef_[0] = XPLMFindDataRef("sim/operation/override/override_joystick");
	autopilotActiveRef_[1] = XPLMFindDataRef("sim/operation/override/override_throttles");

	batteryVoltageRef_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_bat_volt");
	batteryCurrentRef_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_bat_amp");

	motorSpeedRef_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_tacrad");

	aileronLevelRef_ = XPLMFindDataRef("sim/flightmodel/controls/wing1l_ail1def");
	elevatorLevelRef_ = XPLMFindDataRef("sim/flightmodel/controls/hstab1_elv1def");
	rudderLevelRef_ = XPLMFindDataRef("sim/flightmodel/controls/vstab1_rud1def");
	throttleLevelRef_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_thro_use");

	actuationRef_[0] = XPLMFindDataRef("sim/joystick/yoke_roll_ratio");
	actuationRef_[1] = XPLMFindDataRef("sim/joystick/yoke_pitch_ratio");
	actuationRef_[2] = XPLMFindDataRef("sim/joystick/yoke_heading_ratio");
}

void
XPlaneRosNode::publishSensorData()
{
	double yaw_angle_unbounded = 0;
	double latitude = XPLMGetDatad(positionRefs_[0]);
	double longitude = XPLMGetDatad(positionRefs_[1]);
	double altitude = XPLMGetDatad(positionRefs_[2]);
	double northing = 0;
	double easting = 0;
	int zone = 0;
	char hemi = 'N';
	float batteryVoltage[8];
	float batteryCurrent[8];
	float motorSpeed[8];
	float throttleLevel[8];
	simulation_interface::sensor_data sensorData;
	Vector3 accelerationInertial;
	Vector3 accelerationBody;
	Eigen::Matrix3d rotationMatrix;

	latLongToUTM(latitude, longitude, northing, easting, zone, hemi);

	sensorData.sequenceNr = sequenceNr_++;
	sensorData.header.stamp = ros::Time::now();

	sensorData.position.x = easting;
	sensorData.position.y = northing;
	sensorData.position.z = altitude;

	sensorData.velocity.linear.x = static_cast<double>(XPLMGetDataf(velocityRefs_[0]));
	sensorData.velocity.linear.y = static_cast<double>(XPLMGetDataf(velocityRefs_[2])) * -1;
	sensorData.velocity.linear.z = static_cast<double>(XPLMGetDataf(velocityRefs_[1]));

	sensorData.air_speed = static_cast<double>(XPLMGetDataf(airSpeedRef_));
	sensorData.ground_speed = sqrt(
			pow(sensorData.velocity.linear.x, 2) + pow(sensorData.velocity.linear.y, 2)
					+ pow(sensorData.velocity.linear.z, 2));

	accelerationInertial[0] = static_cast<double>(XPLMGetDataf(accelerationRefs_[0]));
	accelerationInertial[1] = static_cast<double>(XPLMGetDataf(accelerationRefs_[2]));
	accelerationInertial[2] = static_cast<double>(XPLMGetDataf(accelerationRefs_[1]));
	rotationMatrix = Eigen::AngleAxisd(sensorData.attitude.x, Vector3::UnitX() * -1)
			* Eigen::AngleAxisd(sensorData.attitude.y, Vector3::UnitY() * -1)
			* Eigen::AngleAxisd(sensorData.attitude.z, Vector3::UnitZ() * -1);
	accelerationBody = rotationMatrix * accelerationInertial;
	sensorData.acceleration.linear.x = accelerationBody[0];
	sensorData.acceleration.linear.y = accelerationBody[1];
	sensorData.acceleration.linear.z = accelerationBody[2];

	yaw_angle_unbounded = degToRad(static_cast<double>(XPLMGetDataf(attitudeRefs_[2])));
	sensorData.attitude.x = degToRad(static_cast<double>(XPLMGetDataf(attitudeRefs_[0])));
	sensorData.attitude.y = degToRad(static_cast<double>(XPLMGetDataf(attitudeRefs_[1])));
	sensorData.attitude.z = boundAngleRad((yaw_angle_unbounded - M_PI / 2) * -1);

	sensorData.angle_of_attack = degToRad(static_cast<double>(XPLMGetDataf(angleOfAttackRef_)));
	sensorData.angle_of_sideslip = degToRad(static_cast<double>(XPLMGetDataf(angleOfSideslipRef_)));

	sensorData.velocity.angular.x = degToRad(
			static_cast<double>(XPLMGetDataf(angularRateRefs_[0])));
	sensorData.velocity.angular.y = degToRad(
			static_cast<double>(XPLMGetDataf(angularRateRefs_[1])) * -1);
	sensorData.velocity.angular.z = degToRad(
			static_cast<double>(XPLMGetDataf(angularRateRefs_[2])));

	sensorData.gps_fix = static_cast<bool>(XPLMGetDatai(gpsFixRef_));
	sensorData.autopilot_active = autopilotActive_;

	XPLMGetDatavf(batteryVoltageRef_, batteryVoltage, 0, 8);
	XPLMGetDatavf(batteryCurrentRef_, batteryCurrent, 0, 8);
	sensorData.battery_voltage = batteryVoltage[0];
	sensorData.battery_current = batteryCurrent[0] + 24;

	XPLMGetDatavf(motorSpeedRef_, motorSpeed, 0, 8);
	motorSpeed[0] = motorSpeed[0] * 60 / M_PI / 2; // Radians per second to revolutions per minute
	sensorData.motor_speed = motorSpeed[0];

	XPLMGetDatavf(throttleLevelRef_, throttleLevel, 0, 8);
	sensorData.aileron_level = static_cast<double>(XPLMGetDataf(aileronLevelRef_));
	sensorData.elevator_level = static_cast<double>(XPLMGetDataf(elevatorLevelRef_));
	sensorData.rudder_level = static_cast<double>(XPLMGetDataf(rudderLevelRef_));
	sensorData.throttle_level = throttleLevel[0];

	sensorDataPublisher_.publish(sensorData);

	ros::spinOnce();
}

void
XPlaneRosNode::setActuationData(const simulation_interface::actuation& actuation)
{
	if (!autopilotActive_)
	{
		return;
	}

	float roll = static_cast<float>(actuation.rollOutput);
	float pitch = static_cast<float>(actuation.pitchOutput);
	float yaw = static_cast<float>(actuation.yawOutput);
	float throttleValue = (static_cast<float>(actuation.throttleOutput) + 1) / 2;
	float throttle[] =
	{ throttleValue, throttleValue, throttleValue, throttleValue, throttleValue, throttleValue,
			throttleValue, throttleValue };

	XPLMSetDataf(actuationRef_[0], roll);
	XPLMSetDataf(actuationRef_[1], pitch);
	XPLMSetDataf(actuationRef_[2], yaw);
	XPLMSetDatavf(throttleLevelRef_, throttle, 0, 8);
}
