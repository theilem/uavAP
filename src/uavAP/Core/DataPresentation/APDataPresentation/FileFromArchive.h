/**
 * @file FileFromArchive.h
 * @date Nov 12, 2018
 * @author Mirco Theile, mirco.theile@tum.de
 * @brief
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_FILEFROMARCHIVE_H_
#define UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_FILEFROMARCHIVE_H_
#include <google/protobuf/message.h>
#include "uavAP/Core/DataPresentation/APDataPresentation/ArchiveOptions.h"
#include <iostream>
#include <type_traits>


class FileFromArchive
{

public:

	FileFromArchive(std::ifstream& file, const ArchiveOptions& opts = ArchiveOptions());

	void
	setOptions(const ArchiveOptions& opts);

	void
	read(char* val, unsigned long bytes);

	template<class Type>
	typename std::enable_if<!std::is_base_of<google::protobuf::Message, Type>::value,
	FileFromArchive>::type&
	operator >>(Type& val);

	template<class Type>
	typename std::enable_if<std::is_base_of<google::protobuf::Message, Type>::value,
			FileFromArchive>::type&
	operator >>(Type& message);

	FileFromArchive&
	operator >>(double& doub);

	template<class Type>
	void
	operator &(Type& val);

	template<class Type>
	FileFromArchive&
	operator <<(Type& val);

private:

	ArchiveOptions options_;

	std::ifstream& file_;
};

#endif /* UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_FILEFROMARCHIVE_H_ */
