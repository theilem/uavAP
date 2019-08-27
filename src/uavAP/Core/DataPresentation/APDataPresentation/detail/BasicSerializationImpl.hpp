////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
// 
// This file is part of uavAP.
// 
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
/**
 * @file BasicSerializationImpl.hpp
 * @brief Implementation of the generic serialization methods in BasicSerialization.h
 * @date Nov 14, 2017
 * @author Mirco Theile, mirco.theile@tum.de
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_DETAIL_BASICSERIALIZATIONIMPL_HPP_
#define UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_DETAIL_BASICSERIALIZATIONIMPL_HPP_

#include "uavAP/Core/DataPresentation/APDataPresentation/BasicSerialization.h"

template<class Type>
inline void
dp::split(BinaryFromArchive& ar, Type& val)
{
	load<BinaryFromArchive, Type>(ar, val);
}

template<class Type>
inline void
dp::split(BinaryToArchive& ar, Type& val)
{
	store<BinaryToArchive, Type>(ar, val);
}

template<class Type>
inline void
dp::split(FileFromArchive& ar, Type& val)
{
	load<FileFromArchive, Type>(ar, val);
}

template<class Type>
inline void
dp::split(FileToArchive& ar, Type& val)
{
	store<FileToArchive, Type>(ar, val);
}

template<class Archive>
inline void
dp::serialize(Archive& ar, char* val, std::size_t size)
{
	split(ar, val, size);
}

template<class Archive, typename EnumType>
void
dp::load(Archive& ar, typename std::enable_if<std::is_enum<EnumType>::value, EnumType>::type& val)
{
	uint8_t e;
	ar >> e;
	val = static_cast<EnumType>(e);
}

template<class Archive, typename EnumType>
void
dp::store(Archive& ar, typename std::enable_if<std::is_enum<EnumType>::value, EnumType>::type& val)
{
	ar << static_cast<uint8_t>(val);
}

template<class Archive, class EnumType>
void
dp::serialize(Archive& ar,
		typename std::enable_if<std::is_enum<EnumType>::value, EnumType>::type& val)
{
	split(ar, val);
}

template<class Archive, typename PODType>
inline void
dp::load(Archive& ar,
		typename std::enable_if<
				std::is_pod<PODType>::value && !std::is_enum<PODType>::value
						&& !std::is_base_of<SerializeCustom, PODType>::value, PODType>::type& val)
{
	load(ar, reinterpret_cast<char*>(&val), sizeof(PODType));
}

template<class Archive, typename PODType>
inline void
dp::store(Archive& ar,
		typename std::enable_if<
				std::is_pod<PODType>::value && !std::is_enum<PODType>::value
						&& !std::is_base_of<SerializeCustom, PODType>::value, PODType>::type& val)
{
	store(ar, reinterpret_cast<char*>(&val), sizeof(PODType));
}

template<class Archive, typename PODType>
inline void
dp::serialize(Archive& ar,
		typename std::enable_if<
				std::is_pod<PODType>::value && !std::is_enum<PODType>::value
						&& !std::is_base_of<SerializeCustom, PODType>::value, PODType>::type& val)
{
	split(ar, val);
}

template<class Archive, typename Type>
inline void
dp::load(Archive& ar, typename std::enable_if<is_vector<Type>::value, Type>::type& val)
{
	uint8_t size;
	ar >> size;
	for (uint8_t i = 0; i < size; ++i)
	{
		typename Type::value_type tmp;
		ar >> tmp;
		val.push_back(tmp);
	}
}

template<class Archive, typename Type>
inline void
dp::store(Archive& ar, typename std::enable_if<is_vector<Type>::value, Type>::type& val)
{
	ar << static_cast<uint8_t>(val.size());
	for (auto& it : val)
	{
		ar << it;
	}
}

template<class Archive, typename Type>
inline void
dp::serialize(Archive& ar, typename std::enable_if<is_vector<Type>::value, Type>::type& val)
{
	split(ar, val);
}

//template<class Archive, typename Type>
//inline void
//dp::load(Archive& ar, typename std::enable_if<is_optional<Type>::value, Type>::type& val)
//{
//	bool init;
//	ar >> init;
//	if (init)
//	{
//		Type temp;
//		ar >> temp;
//		val = temp;
//	}
//}
//
//template<class Archive, typename Type>
//inline void
//dp::store(Archive& ar, typename std::enable_if<is_optional<Type>::value, Type>::type& val)
//{
//	ar << val.is_initialized();
//	if (val)
//	{
//		ar << *val;
//	}
//}
//
//template<class Archive, typename Type>
//inline void
//dp::serialize(Archive& ar, typename std::enable_if<is_optional<Type>::value, Type>::type& val)
//{
//	split(ar, val);
//}

template<class Archive, typename Type>
inline void
dp::load(Archive& ar, typename std::enable_if<isOptional<Type>::value, Type>::type& val)
{
	bool init;
	ar >> init;
	if (init)
	{
		Type temp;
		ar >> temp;
		val = temp;
	}
}

template<class Archive, typename Type>
inline void
dp::store(Archive& ar, typename std::enable_if<isOptional<Type>::value, Type>::type& val)
{
	ar << val.is_initialized();
	if (val)
	{
		ar << *val;
	}
}

template<class Archive, typename Type>
inline void
dp::serialize(Archive& ar, typename std::enable_if<isOptional<Type>::value, Type>::type& val)
{
	split(ar, val);
}

template<class Archive, typename Type>
inline void
dp::load(Archive& ar, typename std::enable_if<is_map<Type>::value, Type>::type& val)
{
	uint8_t size;
	ar >> size;
	for (uint8_t i = 0; i < size; ++i)
	{
		typename Type::mapped_type valTmp;
		typename Type::key_type keyTmp;
		ar >> keyTmp >> valTmp;
		val.insert(std::make_pair(keyTmp, valTmp));
	}
}

template<class Archive, typename Type>
inline void
dp::store(Archive& ar, typename std::enable_if<is_map<Type>::value, Type>::type& val)
{
	ar << static_cast<uint8_t>(val.size());
	for (auto& it : val)
	{
		ar << it.first;
		ar << it.second;
	}
}

template<class Archive, typename Type>
inline void
dp::serialize(Archive& ar, typename std::enable_if<is_map<Type>::value, Type>::type& val)
{
	split(ar, val);
}

template<class Archive, typename Type>
inline void
dp::load(Archive& ar,
		typename std::enable_if<std::is_same<Type, std::string>::value, Type>::type& val)
{
	uint16_t size;
	ar >> size;
	val.resize(size);
	ar.read(const_cast<char*>(val.data()), size);
}

template<class Archive, typename Type>
inline void
dp::store(Archive& ar,
		typename std::enable_if<std::is_same<Type, std::string>::value, Type>::type& val)
{
	ar << static_cast<uint16_t>(val.size());
	ar.append(val.c_str(), val.size());
}

template<class Archive, typename Type>
inline void
dp::serialize(Archive& ar,
		typename std::enable_if<std::is_same<Type, std::string>::value, Type>::type& val)
{
	split(ar, val);
}

template<class Archive, typename Type>
inline void
dp::serialize(Archive& ar, typename std::enable_if<is_pair<Type>::value, Type>::type& val)
{
	ar & val.first;
	ar & val.second;
}

template<class Archive, typename Type>
inline void
dp::serialize(Archive& ar, typename std::enable_if<is_parameter<Type>::value, Type>::type& val)
{
	ar & val.value;
}

template<class Archive, typename Type>
inline void
dp::serialize(Archive& ar, typename std::enable_if<is_parameter_set<Type>::value, Type>::type& val)
{
	val.configure(ar);
}


#endif /* UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_DETAIL_BASICSERIALIZATIONIMPL_HPP_ */
