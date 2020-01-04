/*
 * JsonPopulatorTest.cpp
 *
 *  Created on: Aug 27, 2019
 *      Author: mirco
 */

#include <boost/test/unit_test.hpp>
#include <uavAP/Core/PropertyMapper/ConfigurableObject.hpp>
#include <uavAP/Core/PropertyMapper/JsonPopulator.h>
#include <uavAP/FlightControl/LocalPlanner/ManeuverLocalPlanner/ManeuverLocalPlanner.h>
#include <fstream>

namespace
{
struct ParamsNested
{
	Parameter<float> p1 =
	{ 2.5, "p1", true };
	Parameter<std::string> p2 =
	{ "test", "p2", false };
	Parameter<uint64_t> p3 =
	{ 100, "p3", true };

	template<class Configurator>
	inline void
	configure(Configurator& c)
	{
		c & p1;
		c & p2;
		c & p3;
	}
};

struct Params
{
	Parameter<float> p1 =
	{ 1.0, "p1", true };
	Parameter<int> p2 =
	{ 7, "p2", true };
	Parameter<ParamsNested> p3 =
	{
	{ }, "p3", false };

	template<class Configurator>
	inline void
	configure(Configurator& c)
	{
		c & p1;
		c & p2;
		c & p3;
	}
};

class Test: public ConfigurableObject<Params>
{
public:

	static constexpr const char* const typeId = "test";

};

class Test2: public ConfigurableObject<Params>
{
public:
	static constexpr const char* const typeId = "test2";

	bool
	configure(const Configuration& config)
	{
		PropertyMapper<Configuration> pm(config);

		configureParams(pm);

		return pm.map();
	}

	template<typename Config>
	void
	configureParams(Config& c)
	{
		params.configure(c);

		ParameterRef<Test> ref(test,
		{ }, "sub_test", true);
		c & ref;

	}

private:

	Test test;

};

}

BOOST_AUTO_TEST_SUITE(JsonPopulatorTest)

BOOST_AUTO_TEST_CASE(populator_test_1)
{
	JsonPopulator pop;
	pop.populate<Test, Test>();

	std::string correct = "{\n"
			"\t\"test\":{\n"
			"\t\t\"p1\":1,\n"
			"\t\t\"p2\":7,\n"
			"\t\t\"p3\":{\n"
			"\t\t\t\"p1\":2.5,\n"
			"\t\t\t\"p2\":\"test\",\n"
			"\t\t\t\"p3\":100\n"
			"\t\t}\n"
			"\t},\n"
			"\t\"test\":{\n"
			"\t\t\"p1\":1,\n"
			"\t\t\"p2\":7,\n"
			"\t\t\"p3\":{\n"
			"\t\t\t\"p1\":2.5,\n"
			"\t\t\t\"p2\":\"test\",\n"
			"\t\t\t\"p3\":100\n"
			"\t\t}\n"
			"\t}\n"
			"}";

	BOOST_CHECK_EQUAL(correct.compare(pop.getString()), 0);
}

BOOST_AUTO_TEST_CASE(populator_test_2)
{
	JsonPopulator pop;
	pop.populate<Test2>();

	std::string correct = "{\n"
			"\t\"test2\":{\n"
			"\t\t\"p1\":1,\n"
			"\t\t\"p2\":7,\n"
			"\t\t\"p3\":{\n"
			"\t\t\t\"p1\":2.5,\n"
			"\t\t\t\"p2\":\"test\",\n"
			"\t\t\t\"p3\":100\n"
			"\t\t},\n"
			"\t\t\"sub_test\":{\n"
			"\t\t\t\"p1\":1,\n"
			"\t\t\t\"p2\":7,\n"
			"\t\t\t\"p3\":{\n"
			"\t\t\t\t\"p1\":2.5,\n"
			"\t\t\t\t\"p2\":\"test\",\n"
			"\t\t\t\t\"p3\":100\n"
			"\t\t\t}\n"
			"\t\t}\n"
			"\t}\n"
			"}";

	BOOST_CHECK_EQUAL(correct.compare(pop.getString()), 0);

}

BOOST_AUTO_TEST_SUITE_END()
