#include <gtest/gtest.h>

#include "hugin_string_helpers.hpp"

#include <string>
#include <vector>

TEST(string_helpers, vector_to_string) {
  std::vector<uint8_t> input{65, 66, 67, 68};
  auto output = vector_to_string(input);
  std::string expected = "ABCD";

  EXPECT_EQ(output, expected);
}

TEST(string_helpers, vector_to_hex_string) {
  std::vector<uint8_t> input{0, 65, 66, 67, 68};
  auto output = vector_to_hex_string(input);
  std::string expected = "0041424344";

  EXPECT_EQ(output, expected);
}
