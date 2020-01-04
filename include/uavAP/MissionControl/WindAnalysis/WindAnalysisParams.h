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
/*
 * WindAnalysisParams.h
 *
 *  Created on: Oct 21, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_MISSIONCONTROL_WINDANALYSIS_WINDANALYSISPARAMS_H_
#define UAVAP_MISSIONCONTROL_WINDANALYSIS_WINDANALYSISPARAMS_H_

#include <cpsCore/Configuration/Parameter.hpp>

struct WindAnalysisParams
{
	Parameter<bool> useAlpha = {false, "use_alpha", false};
	Parameter<bool> useBeta = {false, "use_beta", false};

	template <typename Config>
	void
	configure(Config& c)
	{
		c & useAlpha;
		c & useBeta;
	}
};

#endif /* UAVAP_MISSIONCONTROL_WINDANALYSIS_WINDANALYSISPARAMS_H_ */
