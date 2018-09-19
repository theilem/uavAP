/*
 * EnumMap.hpp
 *
 *  Created on: Jul 27, 2018
 *      Author: mircot
 */

#ifndef UAVAP_CORE_ENUMMAP_HPP_
#define UAVAP_CORE_ENUMMAP_HPP_
#include <uavAP/Core/Logging/APLogger.h>
#include <initializer_list>
#include <map>
#include <string>

template<typename ENUM>
class EnumMap
{
public:

	EnumMap(const EnumMap<ENUM>&) = delete;

	void
	operator=(const EnumMap<ENUM>&) = delete;

	static EnumMap& getInstance()
	{
		static EnumMap<ENUM> instance;
		return instance;
	}

	static void
	initialize(std::initializer_list<std::pair<ENUM, std::string>> l)
	{
		for (const auto& it : l)
		{
			getInstance().left_.insert(it);
			getInstance().right_.insert(std::make_pair(it.second, it.first));
		}
	}

	static ENUM
	convert(const std::string& str)
	{
		auto it = getInstance().right_.find(str);
		if (it == getInstance().right_.end())
		{
			APLOG_ERROR << "Unknown enum name for " << str;
			return static_cast<ENUM>(0);
		}
		return it->second;
	}

	static std::string
	convert(ENUM e)
	{
		auto it = getInstance().left_.find(e);
		if (it == getInstance().left_.end())
		{
			APLOG_ERROR << "Undefined enum value " << static_cast<int>(e);
			return std::string();
		}
		return it->second;
	}

protected:


	std::map<ENUM, std::string> left_;
	std::map<std::string, ENUM> right_;

private:

	EnumMap()
	{
	}


};

template <typename ENUM>
class EnumInitializer
{
public:

	EnumInitializer(std::initializer_list<std::pair<ENUM, std::string>> l)
	{
		EnumMap<ENUM>::initialize(l);
	}
};

template <typename First, typename Second>
std::pair<const First, Second>*
findInMap(std::map<First,Second>& map, const First& arg)
{
	typename std::map<First,Second>::iterator it = map.find(arg);
	if (it == map.end())
		return nullptr;
	return &(*it);
}

template <typename First, typename Second>
const std::pair<const First, Second>*
findInMap(const std::map<First,Second>& map, const First& arg)
{
	typename std::map<First,Second>::const_iterator it = map.find(arg);
	if (it == map.end())
		return nullptr;
	return &(*it);
}

#define ENUMMAP_INIT(e, ...)	const EnumInitializer<e> initializer_##e(__VA_ARGS__)

#endif /* UAVAP_CORE_ENUMMAP_HPP_ */
