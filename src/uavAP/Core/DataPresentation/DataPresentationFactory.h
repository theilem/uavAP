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
 * @file DataPresentationFactory.h
 * @brief Factory to create data presentations.
 *
 * The factory is templated and based on different implementations of Content and Target enums.
 * @date 30 July 2017
 * @author Mirco Theile, mircot@illinois.edu
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_DATAPRESENTATIONFACTORY_H_
#define UAVAP_CORE_DATAPRESENTATION_DATAPRESENTATIONFACTORY_H_

#include "uavAP/Core/DataPresentation/APDataPresentation/APDataPresentation.h"
#include "uavAP/Core/Framework/Factory.h"
#include "uavAP/Core/DataPresentation/IDataPresentation.h"

template <typename Content, typename Target>
class DataPresentationFactory : public Factory<IDataPresentation<Content, Target>>
{
public:
	/**
	 * @brief Constructor defining possible data presentations.
	 */
	DataPresentationFactory()
	{
		this->addCreator("ap", &APDataPresentation<Content,Target>::create);

		this->setDefault("ap");
	}
};

#endif /* UAVAP_CORE_DATAPRESENTATION_DATAPRESENTATIONFACTORY_H_ */
