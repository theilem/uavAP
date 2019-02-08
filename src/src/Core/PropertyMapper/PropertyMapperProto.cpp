/*
 * PropertyMapperProto.cpp
 *
 *  Created on: Feb 4, 2019
 *      Author: mirco
 */
#include <uavAP/Core/PropertyMapper/PropertyMapperProto.h>
#include <google/protobuf/message.h>

PropertyMapperProto::PropertyMapperProto(const boost::property_tree::ptree& p) :
		PropertyMapper(p)
{
}

bool
PropertyMapperProto::addProto(const std::string& key, google::protobuf::Message& val, bool mandatory)
{
	if (key.empty())
		return configure(val, p_);

	auto value = p_.get_child_optional(key);
	if (value)
	{
		return configure(val, *value);
	}
	if (mandatory)
	{
		APLOG_ERROR << "PM: mandatory " << key << " missing";
		mandatoryCheck_ = false;
	}
	return false;
}

bool
PropertyMapperProto::configure(google::protobuf::Message& message, bool mandatory)
{
	if (configure(message, p_))
		return true;
	if (mandatory)
	{
		mandatoryCheck_ = false;
	}
	return false;
}

template<typename Type>
boost::optional<Type>
get(const google::protobuf::FieldDescriptor* fdescriptor, const boost::property_tree::ptree& config)
{
	auto value = config.get_optional<Type>(fdescriptor->name());
	if (value)
	{
		APLOG_TRACE << "Property " << fdescriptor->name() << " = " << *value;
		return value;
	}
	if (!fdescriptor->has_default_value())
	{
		APLOG_ERROR << "PM: mandatory " << fdescriptor->name() << " missing";
	}

	return boost::none;
}

bool
PropertyMapperProto::configure(google::protobuf::Message& message,
		const boost::property_tree::ptree& config)
{
	using namespace google::protobuf;
		const Descriptor* descriptor = message.GetDescriptor();
		const Reflection* reflection = message.GetReflection();
		PropertyMapperProto pm(config);

		for (int i = 1; i <= descriptor->field_count(); ++i)
		{
			const FieldDescriptor* fDescriptor = descriptor->FindFieldByNumber(i);

			switch (fDescriptor->cpp_type())
			{
			case FieldDescriptor::CPPTYPE_MESSAGE:
				pm.addProto(fDescriptor->name(), *reflection->MutableMessage(&message, fDescriptor),
						!fDescriptor->has_default_value());
				break;
			case FieldDescriptor::CPPTYPE_BOOL:
				if (auto val = get<bool>(fDescriptor, config))
					reflection->SetBool(&message, fDescriptor, *val);
				break;
			case FieldDescriptor::CPPTYPE_DOUBLE:
				if (auto val = get<double>(fDescriptor, config))
					reflection->SetDouble(&message, fDescriptor, *val);
				break;
			case FieldDescriptor::CPPTYPE_FLOAT:
				if (auto val = get<float>(fDescriptor, config))
					reflection->SetFloat(&message, fDescriptor, *val);
				break;
			case FieldDescriptor::CPPTYPE_INT32:
				if (auto val = get<int32_t>(fDescriptor, config))
					reflection->SetInt32(&message, fDescriptor, *val);
				break;
			case FieldDescriptor::CPPTYPE_INT64:
				if (auto val = get<int64_t>(fDescriptor, config))
					reflection->SetInt64(&message, fDescriptor, *val);
				break;
			case FieldDescriptor::CPPTYPE_UINT32:
				if (auto val = get<uint32_t>(fDescriptor, config))
					reflection->SetUInt32(&message, fDescriptor, *val);
				break;
			case FieldDescriptor::CPPTYPE_UINT64:
				if (auto val = get<uint64_t>(fDescriptor, config))
					reflection->SetUInt64(&message, fDescriptor, *val);
				break;
			default:
				break;
			}
		}

		APLOG_TRACE << "Configured message: " << std::endl << message.DebugString();

		return true; //TODO Fix return value
}
