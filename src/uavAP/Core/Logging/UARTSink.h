/*
 * UARTSink.h
 *
 *  Created on: Jun 7, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_LOGGING_UARTSINK_H_
#define UAVAP_CORE_LOGGING_UARTSINK_H_
#include <streambuf>
#include <sstream>

class UARTSink: public std::ostream
{
public:
	struct UARTRdbuf: public std::stringbuf
	{
		int
		sync() override
		{
#ifdef ERIKA
			int ret = 1;
			printk(str().c_str());
#else
			int ret = printf(str().c_str());
#endif
			this->str("");
			return ret;
		}
	};

	UARTSink()
	{
		rdbuf(&buf);
	}

private:

	UARTRdbuf buf;

};

#endif /* UAVAP_CORE_LOGGING_UARTSINK_H_ */
