////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavEE.
//
// uavEE is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavEE is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////

#include <ctime>
#include <uavAP/Core/Frames/InertialFrame.h>
#include <uavAP/Core/LinearAlgebra.h>
#include <uavAP/Core/Logging/APLogger.h>
#include <uavAP/Core/SensorData.h>
#include <uavAP/Core/Time.h>
#include <autopilot_interface/detail/uavAPConversions.h>

#include "ui_WidgetXPlane.h"
#include "ground_station/Widgets/WidgetXPlane.h"
#include "ground_station/IDataSignals.h"
#include "ground_station/IWidgetInterface.h"

WidgetXPlane::WidgetXPlane(QWidget* parent) :
		QWidget(parent), ui(new Ui::WidgetXPlane), gpsFix_(false), autopilotActive_(false), sensorDataActive_(
				false), edit_(true)
{
	ui->setupUi(this);

	disabledStyle_ =
			":disabled { color: white; border-color: rgb(49, 54, 59); padding-left: 0; padding-right: 0 }";
	enabledStyle_ = "padding-left: 0; padding-right: 0";
	enabledStyleButton_ = "background-color: rgb(35, 38, 41); padding-left: 0; padding-right: 0";

	std::unique_lock<std::mutex> lock(editMutex_);
	setEdit(false);
	lock.unlock();
}

WidgetXPlane::~WidgetXPlane()
{
	APLOG_DEBUG << "WidgetXPlane: Widget Deleted.";
	delete ui;
}

void
WidgetXPlane::onLocalFrame(const VehicleOneFrame& localFrame)
{
	localFrame_ = localFrame;
}

void
WidgetXPlane::on_gpsFixValue_clicked()
{
	gpsFix_ = !gpsFix_;

	if (gpsFix_)
	{
		ui->gpsFixValue->setText(QString("Acquired"));
	}
	else
	{
		ui->gpsFixValue->setText(QString("Lost"));
	}
}

void
WidgetXPlane::on_autopilotValue_clicked()
{
	autopilotActive_ = !autopilotActive_;

	if (autopilotActive_)
	{
		ui->autopilotValue->setText(QString("On"));
	}
	else
	{
		ui->autopilotValue->setText(QString("Off"));
	}
}

void
WidgetXPlane::on_edit_clicked()
{
	std::unique_lock<std::mutex> lock(editMutex_);
	setEdit(true);
	lock.unlock();
}

void
WidgetXPlane::on_cancel_clicked()
{
	std::unique_lock<std::mutex> lock(editMutex_);
	setEdit(false);
	lock.unlock();
}

void
WidgetXPlane::on_apply_clicked()
{
	std::unique_lock<std::mutex> lock(editMutex_);
	if (edit_)
	{
		setEdit(false);
	}
	lock.unlock();
}

void
WidgetXPlane::onSensorData(const simulation_interface::sensor_data& sensorData)
{
	std::unique_lock<std::mutex> lock(editMutex_);
	if (edit_)
	{
		return;
	}
	lock.unlock();

	if (!sensorDataActive_)
	{
		sensorDataActive_ = true;
	}

	std::string printFormat = "%10.5f";
	QString string;
	SensorData sensorDataAP = rosToAp(sensorData);

	if (ui->frameValue->currentIndex() == 1)
	{
		changeFrame(localFrame_, InertialFrame(), sensorDataAP);
	}

	string = QString::fromStdString(boost::posix_time::to_simple_string(sensorData.header.stamp.toBoost()));
	ui->timeValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorDataAP.position.x());
	ui->positionEValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorDataAP.position.y());
	ui->positionNValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorDataAP.position.z());
	ui->positionUValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorDataAP.velocity.x());
	ui->velocityEValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorDataAP.velocity.y());
	ui->velocityNValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorDataAP.velocity.z());
	ui->velocityUValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.air_speed);
	ui->airSpeedValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.ground_speed);
	ui->groundSpeedValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.acceleration.linear.x);
	ui->accelerationUValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.acceleration.linear.y);
	ui->accelerationVValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.acceleration.linear.z);
	ui->accelerationWValue->setText(string);

	string.sprintf(printFormat.c_str(), radToDeg(sensorDataAP.attitude.x()));
	ui->rollAngleValue->setText(string);

	string.sprintf(printFormat.c_str(), radToDeg(sensorDataAP.attitude.y()));
	ui->pitchAngleValue->setText(string);

	string.sprintf(printFormat.c_str(), radToDeg(sensorDataAP.attitude.z()));
	ui->yawAngleValue->setText(string);

	string.sprintf(printFormat.c_str(), radToDeg(sensorData.angle_of_attack));
	ui->attackAngleValue->setText(string);

	string.sprintf(printFormat.c_str(), radToDeg(sensorData.angle_of_sideslip));
	ui->SideslipAngleValue->setText(string);

	string.sprintf(printFormat.c_str(), radToDeg(sensorData.velocity.angular.x));
	ui->rollRateValue->setText(string);

	string.sprintf(printFormat.c_str(), radToDeg(sensorData.velocity.angular.y));
	ui->pitchRateValue->setText(string);

	string.sprintf(printFormat.c_str(), radToDeg(sensorData.velocity.angular.z));
	ui->yawRateValue->setText(string);

	if (sensorDataAP.hasGPSFix)
	{
		ui->gpsFixValue->setText("Acquired");
	}
	else
	{
		ui->gpsFixValue->setText("Lost");
	}

	if (sensorDataAP.autopilotActive)
	{
		ui->autopilotValue->setText("On");
	}
	else
	{
		ui->autopilotValue->setText("Off");
	}

	string.sprintf(printFormat.c_str(), sensorData.battery_voltage);
	ui->batteryVoltageValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.battery_current);
	ui->batteryCurrentValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.aileron_level);
	ui->aileronLevelValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.elevator_level);
	ui->elevatorLevelValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.rudder_level);
	ui->rudderLevelValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.throttle_level * 100);
	ui->throttleLevelValue->setText(string);

	string.sprintf(printFormat.c_str(), sensorData.motor_speed);
	ui->motorSpeedValue->setText(string);

	update();
}

void
WidgetXPlane::connectInterface(std::shared_ptr<IWidgetInterface> interface)
{
	if (!interface)
	{
		APLOG_WARN << "WidgetXPlane: Interface Missing.";
		return;
	}
	if (auto ds = interface->getIDataSignals().get())
	{
		QObject::connect(ds.get(), SIGNAL(onSensorData(const simulation_interface::sensor_data&)),
				this, SLOT(onSensorData(const simulation_interface::sensor_data&)));
		QObject::connect(ds.get(), SIGNAL(onLocalFrame(const VehicleOneFrame&)), this,
				SLOT(onLocalFrame(const VehicleOneFrame&)));
	}
	else
		APLOG_ERROR << "WidgetXPlane: IDataSignals Missing.";
}

void
WidgetXPlane::setText(const QString& text)
{
	ui->positionEValue->setText(text);
	ui->positionNValue->setText(text);
	ui->positionUValue->setText(text);

	ui->velocityEValue->setText(text);
	ui->velocityNValue->setText(text);
	ui->velocityUValue->setText(text);

	ui->airSpeedValue->setText(text);
	ui->groundSpeedValue->setText(text);

	ui->accelerationUValue->setText(text);
	ui->accelerationVValue->setText(text);
	ui->accelerationWValue->setText(text);

	ui->rollAngleValue->setText(text);
	ui->pitchAngleValue->setText(text);
	ui->yawAngleValue->setText(text);

	ui->attackAngleValue->setText(text);
	ui->SideslipAngleValue->setText(text);

	ui->rollRateValue->setText(text);
	ui->pitchRateValue->setText(text);
	ui->yawRateValue->setText(text);

	ui->gpsFixValue->setText(text);
	ui->autopilotValue->setText(text);

	ui->batteryVoltageValue->setText(text);
	ui->batteryCurrentValue->setText(text);

	ui->motorSpeedValue->setText(text);

	ui->aileronLevelValue->setText(text);
	ui->elevatorLevelValue->setText(text);
	ui->rudderLevelValue->setText(text);
	ui->throttleLevelValue->setText(text);

	ui->windAltitudeValue->setText(text);
	ui->windDirectionValue->setText(text);
	ui->windSpeedValue->setText(text);
	ui->windTurbulenceValue->setText(text);

	ui->windShearDirectionValue->setText(text);
	ui->windShearSpeedValue->setText(text);

	ui->activeWindDirectionValue->setText(text);
	ui->activeWindSpeedValue->setText(text);
}

void
WidgetXPlane::setStyle(const bool& edit)
{
	QString style = "";

	if (edit)
	{
		style = enabledStyle_;
		clear();
	}
	else
	{
		style = disabledStyle_;

		if (sensorDataActive_)
		{
			setText("-");
		}
		else
		{
			setText("N/A");
		}
	}

	ui->positionEValue->setStyleSheet(style);
	ui->positionNValue->setStyleSheet(style);
	ui->positionUValue->setStyleSheet(style);

	ui->velocityEValue->setStyleSheet(style);
	ui->velocityNValue->setStyleSheet(style);
	ui->velocityUValue->setStyleSheet(style);

	ui->airSpeedValue->setStyleSheet(style);
	ui->groundSpeedValue->setStyleSheet(style);

	ui->accelerationUValue->setStyleSheet(style);
	ui->accelerationVValue->setStyleSheet(style);
	ui->accelerationWValue->setStyleSheet(style);

	ui->rollAngleValue->setStyleSheet(style);
	ui->pitchAngleValue->setStyleSheet(style);
	ui->yawAngleValue->setStyleSheet(style);

	ui->attackAngleValue->setStyleSheet(style);
	ui->SideslipAngleValue->setStyleSheet(style);

	ui->rollRateValue->setStyleSheet(style);
	ui->pitchRateValue->setStyleSheet(style);
	ui->yawRateValue->setStyleSheet(style);

	if (edit)
	{
		ui->gpsFixValue->setStyleSheet(enabledStyleButton_);
		ui->autopilotValue->setStyleSheet(enabledStyleButton_);
	}
	else
	{
		ui->gpsFixValue->setStyleSheet(style);
		ui->autopilotValue->setStyleSheet(style);
	}

	ui->batteryVoltageValue->setStyleSheet(style);
	ui->batteryCurrentValue->setStyleSheet(style);

	ui->motorSpeedValue->setStyleSheet(style);

	ui->aileronLevelValue->setStyleSheet(style);
	ui->elevatorLevelValue->setStyleSheet(style);
	ui->rudderLevelValue->setStyleSheet(style);
	ui->throttleLevelValue->setStyleSheet(style);

	ui->windAltitudeValue->setStyleSheet(style);
	ui->windDirectionValue->setStyleSheet(style);
	ui->windSpeedValue->setStyleSheet(style);
	ui->windTurbulenceValue->setStyleSheet(style);

	ui->windShearDirectionValue->setStyleSheet(style);
	ui->windShearSpeedValue->setStyleSheet(style);

	ui->activeWindDirectionValue->setStyleSheet(style);
	ui->activeWindSpeedValue->setStyleSheet(style);
}

void
WidgetXPlane::setEdit(const bool& edit)
{
	if (edit == edit_)
	{
		return;
	}

	edit_ = edit;

	setStyle(edit);

	ui->positionEValue->setEnabled(edit);
	ui->positionNValue->setEnabled(edit);
	ui->positionUValue->setEnabled(edit);

	ui->velocityEValue->setEnabled(edit);
	ui->velocityNValue->setEnabled(edit);
	ui->velocityUValue->setEnabled(edit);

	ui->airSpeedValue->setEnabled(false);
	ui->groundSpeedValue->setEnabled(false);

	ui->accelerationUValue->setEnabled(edit);
	ui->accelerationVValue->setEnabled(edit);
	ui->accelerationWValue->setEnabled(edit);

	ui->rollAngleValue->setEnabled(edit);
	ui->pitchAngleValue->setEnabled(edit);
	ui->yawAngleValue->setEnabled(edit);

	ui->attackAngleValue->setEnabled(false);
	ui->SideslipAngleValue->setEnabled(false);

	ui->rollRateValue->setEnabled(edit);
	ui->pitchRateValue->setEnabled(edit);
	ui->yawRateValue->setEnabled(edit);

	ui->gpsFixValue->setEnabled(edit);
	ui->autopilotValue->setEnabled(edit);

	ui->batteryVoltageValue->setEnabled(edit);
	ui->batteryCurrentValue->setEnabled(edit);

	ui->motorSpeedValue->setEnabled(edit);

	ui->aileronLevelValue->setEnabled(edit);
	ui->elevatorLevelValue->setEnabled(edit);
	ui->rudderLevelValue->setEnabled(edit);
	ui->throttleLevelValue->setEnabled(edit);

	ui->windAltitudeValue->setEnabled(edit);
	ui->windDirectionValue->setEnabled(edit);
	ui->windSpeedValue->setEnabled(edit);
	ui->windTurbulenceValue->setEnabled(edit);

	ui->windShearDirectionValue->setEnabled(edit);
	ui->windShearSpeedValue->setEnabled(edit);

	ui->activeWindDirectionValue->setEnabled(false);
	ui->activeWindSpeedValue->setEnabled(false);
}

void
WidgetXPlane::clear()
{
	setText("");

	gpsFix_ = false;
	autopilotActive_ = false;
}
