#pragma once

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

// Convert a uint8_t vector to a printable string
std::string vector_to_string(std::vector<uint8_t> &vec);

// Convert a uint8_t vector to a hex string
std::string vector_to_hex_string(std::vector<uint8_t> &vec);

// Convert a hex string to bytes
std::vector<uint8_t> hex_string_to_vector(std::string &str);
