/**
 * @file ArchiveOptions.cpp
 * @date Nov 12, 2018
 * @author Mirco Theile, mirco.theile@tum.de
 * @brief
 */
#include "uavAP/Core/DataPresentation/APDataPresentation/ArchiveOptions.h"

ArchiveOptions&
ArchiveOptions::compressDouble(bool compress)
{
	compressDouble_ = compress;
	return *this;
}
