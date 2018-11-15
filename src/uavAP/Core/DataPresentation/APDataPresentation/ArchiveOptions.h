/**
 * @file ArchiveOptions.h
 * @date Nov 12, 2018
 * @author Mirco Theile, mirco.theile@tum.de
 * @brief
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_ARCHIVEOPTIONS_H_
#define UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_ARCHIVEOPTIONS_H_

struct ArchiveOptions
{
	static constexpr bool COMPRESS_DOUBLE_DEFAULT = false;
	bool compressDouble_ = COMPRESS_DOUBLE_DEFAULT;

	ArchiveOptions&
	compressDouble(bool compress);
};

#endif /* UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_ARCHIVEOPTIONS_H_ */
