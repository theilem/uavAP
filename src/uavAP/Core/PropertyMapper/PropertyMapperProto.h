/*
 * PropertyMapperProto.h
 *
 *  Created on: Feb 4, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_PROPERTYMAPPER_PROPERTYMAPPERPROTO_H_
#define UAVAP_CORE_PROPERTYMAPPER_PROPERTYMAPPERPROTO_H_
#include <boost/property_tree/ptree.hpp>
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"

namespace google
{
namespace protobuf
{
class Message;
}
}

class PropertyMapperProto: public PropertyMapper<boost::property_tree::ptree>
{
public:

	PropertyMapperProto(const boost::property_tree::ptree& p);

	bool
	addProto(const std::string& key, google::protobuf::Message& val, bool mandatory);
//
//	template<typename TYPE>
//	bool
//	add(const std::string& key,
//			typename std::enable_if<!std::is_base_of<google::protobuf::Message, TYPE>::value, TYPE>::type & t,
//			bool mandatory);

	bool
	configure(google::protobuf::Message& message, bool mandatory);

	static bool
	configure(google::protobuf::Message& message, const boost::property_tree::ptree& config);
};
//
//template<typename TYPE>
//inline bool
//PropertyMapperProto::add(const std::string& key,
//		typename std::enable_if<!std::is_base_of<google::protobuf::Message, TYPE>::value, TYPE>::type& t,
//		bool mandatory)
//{
//	return static_cast<PropertyMapper*>(this)->add(key, t, mandatory);
//}

#endif /* UAVAP_CORE_PROPERTYMAPPER_PROPERTYMAPPERPROTO_H_ */
