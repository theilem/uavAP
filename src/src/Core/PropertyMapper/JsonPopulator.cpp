/*
 * JsonPopulator.cpp
 *
 *  Created on: Jul 26, 2019
 *      Author: mirco
 */
#include <uavAP/Core/PropertyMapper/JsonPopulator.h>
#include <fstream>

JsonPopulator::JsonPopulator() :
		jsonString_(stringStream_.rdbuf())
{
	jsonString_ << "{" << std::endl;
	indent();
}

JsonPopulator::JsonPopulator(std::ofstream& file) :
		jsonString_(file.rdbuf())
{
	jsonString_ << "{" << std::endl;
	indent();
}

JsonPopulator::~JsonPopulator()
{
	jsonString_ << std::endl;
	jsonString_ << "}" << std::endl; // No need for outdent
}

std::string
JsonPopulator::getString() const
{
	return stringStream_.str() + "\n}";
}

void
JsonPopulator::addTabs()
{
	for (int k = 0; k < tabCounter_; k++)
		jsonString_ << "\t";
}

void
JsonPopulator::indent()
{
	tabCounter_++;
}

void
JsonPopulator::outdent()
{
	tabCounter_--;
}
