#include <utility>
#include <nlohmann/json.hpp>

//
// Created by seedship on 9/6/21.
//

#ifndef UAVAP_REACHABILITYREGION_H
#define UAVAP_REACHABILITYREGION_H


template<std::size_t dim>
struct ReachabilityRegion
{
//	Parameter<Eigen::Matrix<FloatingType, Eigen::Dynamic, dim, Eigen::DontAlign>> A = {{}, "A", true};
//	Parameter<Eigen::Matrix<FloatingType, Eigen::Dynamic, 1, Eigen::DontAlign>> b = {{}, "b", true};
	ReachabilityRegion(Eigen::Matrix<FloatingType, Eigen::Dynamic, dim, Eigen::DontAlign> A,
					   Eigen::Matrix<FloatingType, Eigen::Dynamic, 1, Eigen::DontAlign> b) : A(std::move(A)),
																							 b(std::move(b))
	{}

	Eigen::Matrix<FloatingType, Eigen::Dynamic, dim, Eigen::DontAlign> A;
	Eigen::Matrix<FloatingType, Eigen::Dynamic, 1, Eigen::DontAlign> b;

//	template<typename Config>
//	inline void
//	configure(Config& c)
//	{
//		c & A;
//		c & b;
//	}

	bool
	reachable(const VectorN<dim>& vec)
	{
		Eigen::VectorXd rightSide = A * vec;

		return (rightSide.array() <= b.array()).all();
	}
};

template<std::size_t dim>
struct ReachabilitySet
{
//	Parameter<std::vector<ReachabilityRegion<dim>>> regions = {{}, "regions", true};

	std::vector<ReachabilityRegion<dim>> regions;

	void
	fromConfig(const std::string& config_path)
	{
		std::ifstream i(config_path);
		nlohmann::json j;
		i >> j;
		auto r = j["regions"];
		for (const auto &i : r)
		{
			std::vector<FloatingType> Avec = i["A"];
			std::vector<FloatingType> bvec = i["b"];


			std::cout << "Avec.size " << Avec.size() << "\n";
			std::cout << "bvec.size " << bvec.size() << "\n";

			auto numIneqs = bvec.size();

			std::cout << "numIneqs " << numIneqs << "\n";

			Eigen::MatrixXd Amat(numIneqs, dim);
			std::memcpy(Amat.data(), Avec.data(), Avec.size() * sizeof(FloatingType));
			Eigen::VectorXd bmat(numIneqs);
			std::memcpy(bmat.data(), bvec.data(), bvec.size() * sizeof(FloatingType));

			regions.emplace_back(Amat, bmat);
		}
	}

	bool
	reachable(const VectorN<dim>& vec)
	{
		for (auto r : regions)
		{
			if (r.reachable(vec))
			{
				return true;
			}
		}
		return false;
	}
};


#endif //UAVAP_REACHABILITYREGION_H
