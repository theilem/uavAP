/**
 * @file FileFromArchiveImpl.hpp
 * @date Nov 13, 2018
 * @author Mirco Theile, mirco.theile@tum.de
 * @brief
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_DETAIL_FILEFROMARCHIVEIMPL_HPP_
#define UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_DETAIL_FILEFROMARCHIVEIMPL_HPP_
#include "uavAP/Core/DataPresentation/APDataPresentation/FileFromArchive.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/BasicSerialization.h"

template<class Type>
typename std::enable_if<!std::is_base_of<google::protobuf::Message, Type>::value,
		FileFromArchive>::type&
FileFromArchive::operator >>(Type& val)
{
	dp::serialize<FileFromArchive, Type>(*this, val);
	return *this;
}

template<class Type>
typename std::enable_if<std::is_base_of<google::protobuf::Message, Type>::value,
		FileFromArchive>::type&
FileFromArchive::operator >>(Type& message)
{
	uint16_t size;
	*this >> size;
	message.ParseFromIstream(file_);
	return *this;
}

template<class Type>
void
FileFromArchive::operator &(Type& val)
{
	*this >> val;
}

template<class Type>
FileFromArchive&
FileFromArchive::operator <<(Type& val)
{
	return *this;
}



#endif /* UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_DETAIL_FILEFROMARCHIVEIMPL_HPP_ */
