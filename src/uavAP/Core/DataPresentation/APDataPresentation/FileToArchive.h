/**
 * @file FileToArchive.h
 * @date Nov 13, 2018
 * @author Mirco Theile, mirco.theile@tum.de
 * @brief
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_FILETOARCHIVE_H_
#define UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_FILETOARCHIVE_H_
#include <google/protobuf/message.h>
#include <uavAP/Core/DataPresentation/APDataPresentation/ArchiveOptions.h>
#include <string>

class FileToArchive
{
public:
	/**
	 * @brief Constructor wrapping around a string
	 * @param str String to be wrapped around and filled with serialization
	 */
	FileToArchive(std::ofstream& str, const ArchiveOptions& opts = ArchiveOptions());

	void
	setOptions(const ArchiveOptions& opts);

	/**
	 * @brief Append the string with length characters from c
	 * @param c Characters to be appended
	 * @param length Number of characters to be appended
	 */
	void
	append(const char* c, size_t length);

	/**
	 * @brief Flush in operator for non protobuf objects.
	 *
	 * Serializes the object that is not a protobuf object.
	 * Uses serialization from BasicSerialization.h and CustomSerialization.h
	 * @param cval Data to be serialized
	 * @return The archive itself
	 */
	template<typename Type>
	typename std::enable_if<!std::is_base_of<google::protobuf::Message, Type>::value, FileToArchive>::type&
	operator <<(const Type& cval);

	/**
	 * @brief Flush in operator for protobuf objects.
	 *
	 * Uses the serialization function of a protobuf object to create a string from that object.
	 * Then appends the current string with the serialization of the object.
	 * @param message Protobuf object
	 * @return The archive itself
	 */
	template<class Type>
	typename std::enable_if<std::is_base_of<google::protobuf::Message, Type>::value, FileToArchive>::type&
	operator <<(const Type& message);

	/**
	 * @brief Handle double according to setting compressDouble_
	 * @param doub
	 * @return
	 */
	FileToArchive&
	operator <<(const double& doub);

	/**
	 * @brief Operator & for flush in
	 * @param val Data to be flushed in
	 */
	template<class Type>
	void
	operator &(const Type& val);

	/**
	 * @brief Flush out operator. Does nothing
	 * @param val Data to be flushed out.
	 * @return The archive itself
	 */
	template<class Type>
	FileToArchive&
	operator >>(const Type& val);

private:

	ArchiveOptions options_;

	std::ofstream& file_;
};

#endif /* UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_FILETOARCHIVE_H_ */
