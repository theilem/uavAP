/**
 * @file FileFromArchive.cpp
 * @date Nov 13, 2018
 * @author Mirco Theile, mirco.theile@tum.de
 * @brief
 */
#include "uavAP/Core/DataPresentation/APDataPresentation/FileFromArchive.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/BasicSerialization.h"
#include <iostream>
#include <sstream>
#include <fstream>

FileFromArchive::FileFromArchive(std::ifstream& file, const ArchiveOptions& opts) :
		options_(opts), file_(file)
{
}

void
FileFromArchive::setOptions(const ArchiveOptions& opts)
{
	options_ = opts;
}

FileFromArchive&
FileFromArchive::operator >>(double& doub)
{
	if (options_.compressDouble_)
	{
		float flo;
		dp::load(*this, reinterpret_cast<char*>(&flo), sizeof(float));
		doub = static_cast<double>(flo);
	}
	else
	{
		dp::load(*this, reinterpret_cast<char*>(&doub), sizeof(double));
	}
	return *this;
}

void
FileFromArchive::read(char* val, unsigned long bytes)
{
	file_.read(val, bytes);
}
