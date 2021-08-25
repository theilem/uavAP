//
// Created by seedship on 8/18/21.
//

#ifndef UAVAP_GAINSCHEDULER_H
#define UAVAP_GAINSCHEDULER_H

#include <cpsCore/Configuration/Parameter.hpp>
#include <cpsCore/cps_object>

#include <cpsCore/Utilities/LinearAlgebra.h>
#include <cpsCore/Utilities/Angle.hpp>

static std::string
mapToStr(const std::unordered_map<std::string, FloatingType>& map)
{
	std::stringstream ss;
	ss << '{';
	bool first = true;
	for (const auto& regionit : map)
	{
		if (!first)
		{
			ss << ',';
		}
		first = false;
		ss << regionit.first << ':' << regionit.second;
	}
	ss << '}';
	return ss.str();
}

template<std::size_t rows, std::size_t cols>
struct GainScheduleRegion
{
	Parameter<std::unordered_map<std::string, FloatingType>> setpoint = {{}, "setpoint", true};
	Parameter<Eigen::Matrix<FloatingType, rows, cols, Eigen::DontAlign>> k = {{}, "k", true};

	template<typename Config>
	inline void
	configure(Config& c)
	{
		c & setpoint;
		c & k;
	}
};

template<std::size_t rows, std::size_t cols>
struct GainSchedulingParams
{
	Parameter<std::vector<GainScheduleRegion<rows, cols>>> regions = {{}, "regions", true};
	Parameter<std::unordered_map<std::string, FloatingType>> interpolation_stddev = {{}, "interpolation_stddev", true};

	template<typename Config>
	inline void
	configure(Config& c)
	{
		c & regions;
		c & interpolation_stddev;
	}

	inline Eigen::Matrix<FloatingType, rows, cols, Eigen::DontAlign>
	calculateGains(const std::unordered_map<std::string, FloatingType>& state)
	{
		using MatrixType = decltype(calculateGains(state));
		std::cout << "Current State:" << mapToStr(state) << "\n";
		std::vector<FloatingType> weights;
		weights.reserve(regions().size());
		MatrixType ans = MatrixType::Zero();

		FloatingType totalWeight = 0;
		std::cout << "Weights:\n";
		for (auto region : regions())
		{
			// Gaussian fuzzifier
			FloatingType weight = 1;
			for (const auto& it : interpolation_stddev())
			{
				auto key = it.first;
				auto setpointVal = region.setpoint().find(key);
				auto stateVal = state.find(key);
				if (setpointVal == region.setpoint().end())
				{
					CPSLOG_ERROR << "Key not found in GainScheduling Region Config: " << key;
					auto s = mapToStr(region.setpoint());
					CPSLOG_ERROR << "Region is: " << s;
				}
				else if (stateVal == state.end())
				{
					CPSLOG_ERROR << "Key not found in System State: " << key;
					auto s = mapToStr(state);
					CPSLOG_ERROR << "Region is: " << s;
				}
				else
				{
					weight *= exp(-0.5 * pow((setpointVal->second - stateVal->second) / it.second, 2));
				}
			}

			std::cout << "Region :" << mapToStr(region.setpoint()) << ":" << weight << "\n";
			weights.push_back(weight);
			totalWeight += weight;
		}

		std::cout << "Percentage:\n";
		for (unsigned idx = 0; idx < weights.size(); idx++)
		{
			auto percent = weights[idx] / totalWeight;
			std::cout << "Region :" << mapToStr(regions()[idx].setpoint()) << ":" << percent << "\n";
			ans += regions()[idx].k() * percent;
		}
		return ans;
	}
};

#endif //UAVAP_GAINSCHEDULER_H
