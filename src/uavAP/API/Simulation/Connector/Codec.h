/*
 * Codec.h
 *
 *  Created on: Jul 28, 2017
 *      Author: mircot
 */

#ifndef SRC_SIMULATION_CODEC_H_
#define SRC_SIMULATION_CODEC_H_
#include <cstddef>
#include <string>

std::string
decode(std::string encoded);

std::string
encode(std::string source);

#endif /* SRC_SIMULATION_CODEC_H_ */
