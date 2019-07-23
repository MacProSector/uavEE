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
 * @file XPlaneRosNode.cpp
 * @author Mirco Theile, mircot@illinois.edu
 * @date [DD.MM.YYYY] 27.3.2018
 * @brief
 */

#include <cmath>
#include <simulation_interface/sensor_data.h>
#include <uavAP/Core/LinearAlgebra.h>
#include <uavAP/Core/Scheduler/IScheduler.h>
#include <uavAP/Core/Logging/APLogger.h>
#include <uavAP/API/ap_ext/latLongToUTM.h>

#include "xPlane/CHeaders/XPLM/XPLMDefs.h"
#include "xPlane/CHeaders/XPLM/XPLMDataAccess.h"
#include "xPlane/CHeaders/XPLM/XPLMGraphics.h"
#include "x_plane_interface/XPlaneRosNode.h"

XPlaneRosNode::XPlaneRosNode() :
		sensorFrequency_(100), sequenceNr_(0), autopilotActive_(false)
{


	positionRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/latitude");
	positionRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/longitude");
	positionRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/elevation");

	velocityRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/local_vx");
	velocityRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/local_vy");
	velocityRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/local_vz");

	accelerationRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/local_ax");
	accelerationRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/local_ay");
	accelerationRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/local_az");

	attitudeRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/phi");
	attitudeRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/theta");
	attitudeRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/psi");

	angularRateRefs_[0] = XPLMFindDataRef("sim/flightmodel/position/P");
	angularRateRefs_[1] = XPLMFindDataRef("sim/flightmodel/position/Q");
	angularRateRefs_[2] = XPLMFindDataRef("sim/flightmodel/position/R");

	trueAirSpeedRef_ = XPLMFindDataRef("sim/flightmodel/position/true_airspeed");

	gpsPowerRef_ = XPLMFindDataRef("sim/cockpit2/radios/actuators/gps_power");

	angleOfAttackRef_ = XPLMFindDataRef("sim/flightmodel2/misc/AoA_angle_degrees");
	angleOfSideslipRef_ = XPLMFindDataRef("sim/flightmodel/position/beta");

	batteryVoltageRef_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_bat_volt");
	batteryCurrentRef_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_bat_amp");

	aileronRef_ = XPLMFindDataRef("sim/flightmodel/controls/wing1l_ail1def");
	elevatorRef_ = XPLMFindDataRef("sim/flightmodel/controls/hstab1_elv1def");
	rudderRef_ = XPLMFindDataRef("sim/flightmodel/controls/vstab1_rud1def");
	throttleRef_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_thro_use");
	rpmRef_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_tacrad");

	overridesRef_[0] = XPLMFindDataRef("sim/operation/override/override_joystick");
	overridesRef_[1] = XPLMFindDataRef("sim/operation/override/override_throttles");

	joystickAttitudeRef_[0] = XPLMFindDataRef("sim/joystick/yoke_roll_ratio");
	joystickAttitudeRef_[1] = XPLMFindDataRef("sim/joystick/yoke_pitch_ratio");
	joystickAttitudeRef_[2] = XPLMFindDataRef("sim/joystick/yoke_heading_ratio");

//    double x, y, z;
//    XPLMWorldToLocal(40.0594, -88.5514, 206, &x, &y, &z);
//    XPLMSetDatad(XPLMFindDataRef("sim/flightmodel/position/local_x"), x);
//    XPLMSetDatad(XPLMFindDataRef("sim/flightmodel/position/local_y"), y);
//    XPLMSetDatad(XPLMFindDataRef("sim/flightmodel/position/local_z"), z);
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
			APLOG_ERROR << "Scheduler not set.";
			return true;
		}
		ros::VP_string config;
		ros::init(config, "xplane_interface");
		break;
	}
	case RunStage::NORMAL:
	{
		ros::NodeHandle nh;
		sensorDataPublisher_ = nh.advertise<simulation_interface::sensor_data>(
				"sensor_data", 20);

		actuationSubscriber_ = nh.subscribe("actuation", 20, &XPlaneRosNode::actuate,
				this);
		auto scheduler = scheduler_.get();

		scheduler->schedule(std::bind(&XPlaneRosNode::getSensorData, this), Milliseconds(0),
				Milliseconds(1000 / sensorFrequency_));
		break;
	}
	default:
		break;
	}

	return false;
}

void
XPlaneRosNode::enableAutopilot()
{
	if (!autopilotActive_)
	{
		autopilotActive_ = true;
		XPLMSetDatai(overridesRef_[0], 1);
		XPLMSetDatai(overridesRef_[1], 1);
	}
}

void
XPlaneRosNode::disableAutopilot()
{
	if (autopilotActive_)
	{
		autopilotActive_ = false;
		XPLMSetDatai(overridesRef_[0], 0);
		XPLMSetDatai(overridesRef_[1], 0);
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
XPlaneRosNode::getSensorData()
{
	double lat = XPLMGetDatad(positionRefs_[0]);
	double lon = XPLMGetDatad(positionRefs_[1]);
	double alt = XPLMGetDatad(positionRefs_[2]);

	double north = 0;
	double east = 0;
	double deg2rad = M_PI / 180.0;
	int zone = 0;
	char hemi = 'N';

	latLongToUTM(lat, lon, north, east, zone, hemi);
	std::cout << lat << " " << lon << std::endl;

	simulation_interface::sensor_data sd;

	sd.header.stamp = ros::Time::now();

	sd.position.x = east;
	sd.position.y = north;
	sd.position.z = alt;

	sd.velocity.linear.x = static_cast<double>(XPLMGetDataf(velocityRefs_[0]));
	sd.velocity.linear.y = -static_cast<double>(XPLMGetDataf(velocityRefs_[2]));
	sd.velocity.linear.z = static_cast<double>(XPLMGetDataf(velocityRefs_[1]));

	sd.velocity.angular.x = static_cast<double>(XPLMGetDataf(angularRateRefs_[0])) * deg2rad;
	sd.velocity.angular.y = -static_cast<double>(XPLMGetDataf(angularRateRefs_[1])) * deg2rad;
	sd.velocity.angular.z = static_cast<double>(XPLMGetDataf(angularRateRefs_[2])) * deg2rad;

	Vector3 accelerationInertial;
	accelerationInertial[0] = static_cast<double>(XPLMGetDataf(accelerationRefs_[0]));
	accelerationInertial[1] = static_cast<double>(XPLMGetDataf(accelerationRefs_[2]));
	accelerationInertial[2] = static_cast<double>(XPLMGetDataf(accelerationRefs_[1]));

	Eigen::Matrix3d m;
	m = Eigen::AngleAxisd(-sd.attitude.x, Vector3::UnitX())
			* Eigen::AngleAxisd(-sd.attitude.y, Vector3::UnitY())
			* Eigen::AngleAxisd(-sd.attitude.z, Vector3::UnitZ());

	Vector3 accelerationBody = m * accelerationInertial;

	sd.acceleration.linear.x = accelerationBody[0];
	sd.acceleration.linear.y = accelerationBody[1];
	sd.acceleration.linear.z = accelerationBody[2];

	double yaw_unbounded = static_cast<double>(XPLMGetDataf(attitudeRefs_[2])) * deg2rad;

	sd.attitude.x = static_cast<double>(XPLMGetDataf(attitudeRefs_[0])) * deg2rad;
	sd.attitude.y = static_cast<double>(XPLMGetDataf(attitudeRefs_[1])) * deg2rad;
	sd.attitude.z = boundAngleRad(-(yaw_unbounded - M_PI/2));

	sd.air_speed = static_cast<double>(XPLMGetDataf(trueAirSpeedRef_));
	sd.ground_speed = sqrt(pow(sd.velocity.linear.x,2) + pow(sd.velocity.linear.y,2) + pow(sd.velocity.linear.z,2));

	sd.sequenceNr = sequenceNr_++;

	sd.has_gps_fix = static_cast<bool>(XPLMGetDatai(gpsPowerRef_));
	sd.autopilot_active = autopilotActive_;

	sd.angle_of_attack = static_cast<double>(XPLMGetDataf(angleOfAttackRef_)) * deg2rad;
	sd.angle_of_sideslip = static_cast<double>(XPLMGetDataf(angleOfSideslipRef_)) * deg2rad;

	float batteryVoltage[8];
	XPLMGetDatavf(batteryVoltageRef_, batteryVoltage, 0, 8);
	sd.battery_voltage = batteryVoltage[0];

	float batteryCurrent[8];
	XPLMGetDatavf(batteryCurrentRef_, batteryCurrent, 0, 8);
	sd.battery_current = batteryCurrent[0];

	sd.aileron = static_cast<double>(XPLMGetDataf(aileronRef_));
	sd.elevator = static_cast<double>(XPLMGetDataf(elevatorRef_));
	sd.rudder = static_cast<double>(XPLMGetDataf(rudderRef_));

	float throttle[8];
	XPLMGetDatavf(throttleRef_, throttle, 0, 8);
	sd.throttle = throttle[0];

	float rpm[8];
	XPLMGetDatavf(rpmRef_, rpm, 0, 8);
	rpm[0] = rpm[0] * 60 / M_PI / 2; // Radians Per Second to Revolution Per Minute
	sd.rpm = rpm[0];

	sensorDataPublisher_.publish(sd);

	ros::spinOnce();
}

void
XPlaneRosNode::actuate(const simulation_interface::actuation& act)
{
	if (!autopilotActive_)
		return;

	std::cout << "Override" << std::endl;

	float throt = (static_cast<float>(act.throttleOutput) + 1) / 2;
	float throttle[] =
	{ throt, throt, throt, throt, throt, throt, throt, throt };
	float roll = static_cast<float>(act.rollOutput);
	float pitch = static_cast<float>(act.pitchOutput);
	float yaw = static_cast<float>(act.yawOutput);
	XPLMSetDatavf(throttleRef_, throttle, 0, 8);
	XPLMSetDataf(joystickAttitudeRef_[0], roll);
	XPLMSetDataf(joystickAttitudeRef_[1], pitch);
	XPLMSetDataf(joystickAttitudeRef_[2], yaw);

}
