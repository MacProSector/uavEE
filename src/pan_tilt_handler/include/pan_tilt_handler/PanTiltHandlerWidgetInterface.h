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
﻿/**
 *   @author Richard Nai, rnai2@illinois.edu
 *   @file PanTiltHandlerWidgetInterface.h
 *   @date [DD/MM/YY] 24/3/2018
 *   @brief
 */

#ifndef PANTILTHANDLERWIDGETINTERFACE_H
#define PANTILTHANDLERWIDGETINTERFACE_H

#include <ground_station/IWidgetInterface.h>
#include <ground_station/IDataSignals.h>
#include <ground_station/IPIDConfigurator.h>
#include <ground_station/MapLogic.h>
#include <ground_station/IConfigManager.h>

class PanTiltHandlerWidgetInterface: public IWidgetInterface, public IAggregatableObject
{
public:

	//PanTiltHandlerWidgetInterface() = default;

	static constexpr TypeId typeId = "pan_tilt_widget_interface";

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	ObjectHandle<IDataSignals>
	getIDataSignals() const override;

	ObjectHandle<MapLogic>
	getMapLogic() const override;

	ObjectHandle<IConfigManager>
	getConfigManager() const override;

	ObjectHandle<IPIDConfigurator>
	getPIDConfigurator() const override;

private:
	ObjectHandle<IDataSignals> dataSignals_;
	ObjectHandle<IPIDConfigurator> pidConfigurator_;
};

#endif // PANTILTHANDLERWIDGETINTERFACE_H
