/*
 * DataFilterFactory.h
 *
 *  Created on: Mar 13, 2019
 *      Author: simonyu
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_DATAFILTER_DATAFILTERFACTORY_H_
#define UAVAP_CORE_DATAPRESENTATION_DATAFILTER_DATAFILTERFACTORY_H_

#include "uavAP/Core/Framework/Factory.h"
#include "uavAP/Core/DataPresentation/DataFilter/LowPassDataFilter/LowPassDataFilter.h"

class DataFilterFactory : public Factory<IDataFilter>
{
public:

	DataFilterFactory()
	{
		addCreator<LowPassDataFilter>();
	}
};

#endif /* UAVAP_CORE_DATAPRESENTATION_DATAFILTER_DATAFILTERFACTORY_H_ */
