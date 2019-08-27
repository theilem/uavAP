/**
 * @file ArchiveOptions.h
 * @date Nov 12, 2018
 * @author Mirco Theile, mirco.theile@tum.de
 * @brief
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_ARCHIVEOPTIONS_H_
#define UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_ARCHIVEOPTIONS_H_
#include <uavAP/Core/PropertyMapper/Parameter.h>

struct ArchiveOptions
{
	Parameter<bool> compressDouble = {false, "compress_double", false};

	template <typename Config>
	inline void
	configure(Config& c)
	{
		c & compressDouble;
	}
};

#endif /* UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_ARCHIVEOPTIONS_H_ */
