/*
 * erika_test.cpp
 *
 *  Created on: Jun 7, 2019
 *      Author: mirco
 */

#include <uavAP/Core/Object/StaticAggregation/StaticObjectContainer.h>
#include <iostream>

struct A
{
	void
	print()
	{
		std::cout << "A" << std::endl;
	}
};

struct B
{
	void
	print()
	{
		std::cout << "B" << std::endl;
	}

};

struct C : public A
{
	void
	print()
	{
		std::cout << "C" << std::endl;
	}
};

struct D
{
	void
	print()
	{
		std::cout << "D" << std::endl;
	}
};

int
main()
{
	StaticObjectContainer<C, B> container;
	container.get<A>()->print();
	container.get<B>()->print();
	if (auto c = container.get<C>())
		c->print();
	if (auto d = container.get<D>())
		d->print();
	return 0;
}
