// Copyright (c) Sensrad 2023
#include "hugin_string_helpers.hpp"
// Boost
#include <boost/algorithm/hex.hpp>

// Convert a uint8_t vector to a printable string
std::string vector_to_string(std::vector<uint8_t> &vec) {
  // Filter out none-printable characters
  std::ostringstream oss;
  std::for_each(vec.cbegin(), vec.cend(), [&](char c) {
    if (isprint(c)) {
      oss << c;
    }
  });

  return oss.str();
}

// Convert a uint8_t vector to a hex string
std::string vector_to_hex_string(std::vector<uint8_t> &vec) {
  std::ostringstream ss;
  ss << std::hex << std::setfill('0');
  std::for_each(vec.cbegin(), vec.cend(),
                [&](int c) { ss << std::setw(2) << c; });
  return ss.str();
}

// Convert a hex string to bytes
std::vector<uint8_t> hex_string_to_vector(std::string &hex_str) {
  std::vector<uint8_t> bytes;
  boost::algorithm::unhex(hex_str, std::back_inserter(bytes));
  return bytes;
}
