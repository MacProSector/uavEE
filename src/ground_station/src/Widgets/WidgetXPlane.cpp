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
		QWidget(parent), ui(new Ui::WidgetXPlane), gpsFix_(false), autopilotActive_(false)
{
	ui->setupUi(this);

	setEdit();

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

	QString string;
	SensorData sensorDataAP = rosToAp(sensorData);

	if (ui->frameValue->currentIndex() == 1)
	{
		changeFrame(localFrame_, InertialFrame(), sensorDataAP);
	}

	string = QString::fromStdString(boost::posix_time::to_simple_string(sensorDataAP.timestamp));
	ui->timeValue->setText(string);

	string.sprintf("%10.2f", sensorDataAP.position.x());
	ui->positionEValue->setText(string);

	string.sprintf("%10.2f", sensorDataAP.position.y());
	ui->positionNValue->setText(string);

	string.sprintf("%10.2f", sensorDataAP.position.z());
	ui->positionUValue->setText(string);

	string.sprintf("%10.2f", sensorDataAP.velocity.x());
	ui->velocityEValue->setText(string);

	string.sprintf("%10.2f", sensorDataAP.velocity.y());
	ui->velocityNValue->setText(string);

	string.sprintf("%10.2f", sensorDataAP.velocity.z());
	ui->velocityUValue->setText(string);

	string.sprintf("%10.2f", sensorDataAP.airSpeed);
	ui->airSpeedValue->setText(string);

	string.sprintf("%10.2f", sensorDataAP.groundSpeed);
	ui->groundSpeedValue->setText(string);

	string.sprintf("%10.2f", sensorDataAP.acceleration.x());
	ui->accelerationUValue->setText(string);

	string.sprintf("%10.2f", sensorDataAP.acceleration.y());
	ui->accelerationVValue->setText(string);

	string.sprintf("%10.2f", sensorDataAP.acceleration.z());
	ui->accelerationWValue->setText(string);

	string.sprintf("%10.2f", radToDeg(sensorDataAP.attitude.x()));
	ui->rollAngleValue->setText(string);

	string.sprintf("%10.2f", radToDeg(sensorDataAP.attitude.y()));
	ui->pitchAngleValue->setText(string);

	string.sprintf("%10.2f", radToDeg(sensorDataAP.attitude.z()));
	ui->yawAngleValue->setText(string);

	string.sprintf("%10.2f", radToDeg(sensorDataAP.angleOfAttack));
	ui->attackAngleValue->setText(string);

	string.sprintf("%10.2f", radToDeg(sensorDataAP.angleOfSideslip));
	ui->SideslipAngleValue->setText(string);

	string.sprintf("%10.2f", radToDeg(sensorDataAP.angularRate.x()));
	ui->rollRateValue->setText(string);

	string.sprintf("%10.2f", radToDeg(sensorDataAP.angularRate.y()));
	ui->pitchRateValue->setText(string);

	string.sprintf("%10.2f", radToDeg(sensorDataAP.angularRate.z()));
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

	string.sprintf("%10.2f", sensorDataAP.batteryVoltage);
	ui->batteryVoltageValue->setText(string);

	string.sprintf("%10.2f", sensorDataAP.batteryCurrent);
	ui->batteryCurrentValue->setText(string);

	string.sprintf("%10.2f", sensorDataAP.aileron);
	ui->aileronLevelValue->setText(string);

	string.sprintf("%10.2f", sensorDataAP.elevator);
	ui->elevatorLevelValue->setText(string);

	string.sprintf("%10.2f", sensorDataAP.rudder);
	ui->rudderLevelValue->setText(string);

	string.sprintf("%10.2f", sensorDataAP.throttle * 100);
	ui->throttleLevelValue->setText(string);

	string.sprintf("%10.2f", sensorDataAP.rpm);
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
WidgetXPlane::clear()
{
	ui->positionEValue->setText(QString(""));
	ui->positionNValue->setText(QString(""));
	ui->positionUValue->setText(QString(""));

	ui->velocityEValue->setText(QString(""));
	ui->velocityNValue->setText(QString(""));
	ui->velocityUValue->setText(QString(""));

	ui->airSpeedValue->setText(QString(""));
	ui->groundSpeedValue->setText(QString(""));

	ui->accelerationUValue->setText(QString(""));
	ui->accelerationVValue->setText(QString(""));
	ui->accelerationWValue->setText(QString(""));

	ui->rollAngleValue->setText(QString(""));
	ui->pitchAngleValue->setText(QString(""));
	ui->yawAngleValue->setText(QString(""));

	ui->attackAngleValue->setText(QString(""));
	ui->SideslipAngleValue->setText(QString(""));

	ui->rollRateValue->setText(QString(""));
	ui->pitchRateValue->setText(QString(""));
	ui->yawRateValue->setText(QString(""));

	ui->gpsFixValue->setText(QString(""));
	ui->autopilotValue->setText(QString(""));

	ui->batteryVoltageValue->setText(QString(""));
	ui->batteryCurrentValue->setText(QString(""));

	ui->motorSpeedValue->setText(QString(""));

	ui->aileronLevelValue->setText(QString(""));
	ui->elevatorLevelValue->setText(QString(""));
	ui->rudderLevelValue->setText(QString(""));
	ui->throttleLevelValue->setText(QString(""));

	ui->windAltitudeValue->setText(QString(""));
	ui->windDirectionValue->setText(QString(""));
	ui->windSpeedValue->setText(QString(""));
	ui->windTurbulenceValue->setText(QString(""));

	ui->windShearDirectionValue->setText(QString(""));
	ui->windShearSpeedValue->setText(QString(""));

	ui->activeWindDirectionValue->setText(QString(""));
	ui->activeWindSpeedValue->setText(QString(""));

	gpsFix_ = false;
	autopilotActive_ = false;
}

void
WidgetXPlane::setEdit()
{
	ui->airSpeedValue->setStyleSheet(":disabled { color: white }");
	ui->groundSpeedValue->setStyleSheet(":disabled { color: white }");

	ui->attackAngleValue->setStyleSheet(":disabled { color: white }");
	ui->SideslipAngleValue->setStyleSheet(":disabled { color: white }");

	ui->activeWindDirectionValue->setStyleSheet(":disabled { color: white }");
	ui->activeWindSpeedValue->setStyleSheet(":disabled { color: white }");

	ui->airSpeedValue->setEnabled(false);
	ui->groundSpeedValue->setEnabled(false);

	ui->attackAngleValue->setEnabled(false);
	ui->SideslipAngleValue->setEnabled(false);

	ui->activeWindDirectionValue->setEnabled(false);
	ui->activeWindSpeedValue->setEnabled(false);
}

void
WidgetXPlane::setEdit(const bool& edit)
{
	edit_ = edit;

	ui->positionEValue->setEnabled(edit);
	ui->positionNValue->setEnabled(edit);
	ui->positionUValue->setEnabled(edit);

	ui->velocityEValue->setEnabled(edit);
	ui->velocityNValue->setEnabled(edit);
	ui->velocityUValue->setEnabled(edit);

	ui->accelerationUValue->setEnabled(edit);
	ui->accelerationVValue->setEnabled(edit);
	ui->accelerationWValue->setEnabled(edit);

	ui->rollAngleValue->setEnabled(edit);
	ui->pitchAngleValue->setEnabled(edit);
	ui->yawAngleValue->setEnabled(edit);

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

	if (edit)
	{
		clear();

		ui->positionEValue->setStyleSheet("");
		ui->positionNValue->setStyleSheet("");
		ui->positionUValue->setStyleSheet("");

		ui->velocityEValue->setStyleSheet("");
		ui->velocityNValue->setStyleSheet("");
		ui->velocityUValue->setStyleSheet("");

		ui->accelerationUValue->setStyleSheet("");
		ui->accelerationVValue->setStyleSheet("");
		ui->accelerationWValue->setStyleSheet("");

		ui->rollAngleValue->setStyleSheet("");
		ui->pitchAngleValue->setStyleSheet("");
		ui->yawAngleValue->setStyleSheet("");

		ui->rollRateValue->setStyleSheet("");
		ui->pitchRateValue->setStyleSheet("");
		ui->yawRateValue->setStyleSheet("");

		ui->gpsFixValue->setStyleSheet("");
		ui->autopilotValue->setStyleSheet("");

		ui->gpsFixValue->setStyleSheet("background-color: rgb(35, 38, 41);");
		ui->autopilotValue->setStyleSheet("background-color: rgb(35, 38, 41);");

		ui->batteryVoltageValue->setStyleSheet("");
		ui->batteryCurrentValue->setStyleSheet("");

		ui->motorSpeedValue->setStyleSheet("");

		ui->aileronLevelValue->setStyleSheet("");
		ui->elevatorLevelValue->setStyleSheet("");
		ui->rudderLevelValue->setStyleSheet("");
		ui->throttleLevelValue->setStyleSheet("");

		ui->windAltitudeValue->setStyleSheet("");
		ui->windDirectionValue->setStyleSheet("");
		ui->windSpeedValue->setStyleSheet("");
		ui->windTurbulenceValue->setStyleSheet("");

		ui->windShearDirectionValue->setStyleSheet("");
		ui->windShearSpeedValue->setStyleSheet("");
	}
	else
	{
		ui->positionEValue->setStyleSheet(":disabled { color: white }");
		ui->positionNValue->setStyleSheet(":disabled { color: white }");
		ui->positionUValue->setStyleSheet(":disabled { color: white }");

		ui->velocityEValue->setStyleSheet(":disabled { color: white }");
		ui->velocityNValue->setStyleSheet(":disabled { color: white }");
		ui->velocityUValue->setStyleSheet(":disabled { color: white }");

		ui->accelerationUValue->setStyleSheet(":disabled { color: white }");
		ui->accelerationVValue->setStyleSheet(":disabled { color: white }");
		ui->accelerationWValue->setStyleSheet(":disabled { color: white }");

		ui->rollAngleValue->setStyleSheet(":disabled { color: white }");
		ui->pitchAngleValue->setStyleSheet(":disabled { color: white }");
		ui->yawAngleValue->setStyleSheet(":disabled { color: white }");

		ui->rollRateValue->setStyleSheet(":disabled { color: white }");
		ui->pitchRateValue->setStyleSheet(":disabled { color: white }");
		ui->yawRateValue->setStyleSheet(":disabled { color: white }");

		ui->gpsFixValue->setStyleSheet(":disabled { color: white }");
		ui->autopilotValue->setStyleSheet(":disabled { color: white }");

		ui->batteryVoltageValue->setStyleSheet(":disabled { color: white }");
		ui->batteryCurrentValue->setStyleSheet(":disabled { color: white }");

		ui->motorSpeedValue->setStyleSheet(":disabled { color: white }");

		ui->aileronLevelValue->setStyleSheet(":disabled { color: white }");
		ui->elevatorLevelValue->setStyleSheet(":disabled { color: white }");
		ui->rudderLevelValue->setStyleSheet(":disabled { color: white }");
		ui->throttleLevelValue->setStyleSheet(":disabled { color: white }");

		ui->windAltitudeValue->setStyleSheet(":disabled { color: white }");
		ui->windDirectionValue->setStyleSheet(":disabled { color: white }");
		ui->windSpeedValue->setStyleSheet(":disabled { color: white }");
		ui->windTurbulenceValue->setStyleSheet(":disabled { color: white }");

		ui->windShearDirectionValue->setStyleSheet(":disabled { color: white }");
		ui->windShearSpeedValue->setStyleSheet(":disabled { color: white }");
	}
}
