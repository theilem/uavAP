//
// Created by seedship on 8/18/21.
//

#ifndef UAVAP_GAINSCHEDULER_H
#define UAVAP_GAINSCHEDULER_H

#include <cpsCore/Configuration/Parameter.hpp>
#include <cpsCore/cps_object>

#include <cpsCore/Utilities/LinearAlgebra.h>
#include <cpsCore/Utilities/Angle.hpp>

template<std::size_t rows, std::size_t cols>
struct GainScheduleRegion
{
	Parameter<FloatingType> u = {0, "u", true};
	Parameter<FloatingType> phi = {decltype(phi.value)(), "phi_rad", true};
	Parameter<Eigen::Matrix<FloatingType, rows, cols, Eigen::DontAlign>> k = {{}, "k", true};

	template<typename Config>
	inline void
	configure(Config& c)
	{
		c & u;
		c & phi;
		c & k;
	}
};

template<std::size_t rows, std::size_t cols>
struct GainSchedulingParams
{
	Parameter<std::vector<GainScheduleRegion<rows, cols>>> regions = {{}, "regions", true};
	Parameter<FloatingType> u_stddev = {1, "u_stddev", true};
	Parameter<FloatingType> phi_stddev = {1, "phi_stddev", true};

	template<typename Config>
	inline void
	configure(Config& c)
	{
		c & regions;
		c & u_stddev;
		c & phi_stddev;
	}

	inline Eigen::Matrix<FloatingType, rows, cols, Eigen::DontAlign>
	calculateGains(FloatingType u, FloatingType phi)
	{
		using MatrixType = decltype(calculateGains(u, phi));
		std::cout << "Current State: u:" << u << ", phi:" << phi << "\n";
		std::vector<FloatingType> weights;
		weights.reserve(regions().size());
		MatrixType ans = MatrixType::Zero();

		FloatingType totalWeight = 0;
		for (auto region : regions())
		{
			// Gaussian fuzzifier
			auto weight = exp(-0.5 * pow((region.u() - u) / u_stddev(), 2)) *
						  exp(-0.5 * pow((region.phi() - phi) / phi_stddev(), 2));
			weights.push_back(weight);
			totalWeight += weight;
		}

		for (unsigned idx = 0; idx < weights.size(); idx++)
		{
			auto percent = weights[idx] / totalWeight;
			std::cout << "Controller u:" << regions()[idx].u() << ", phi:"
						 << regions()[idx].phi() << " " << percent << "\n";
			ans += regions()[idx].k() * percent;
		}
		return ans;
	}
};


#endif //UAVAP_GAINSCHEDULER_H
