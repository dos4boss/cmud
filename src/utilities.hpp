#pragma once

#include <type_traits>
#include <vector>
#include <inttypes.h>
#include <stdexcept>

template <typename T,
          bool big_endian = true,
          typename std::enable_if<std::is_arithmetic<T>::value>::type * = nullptr>
std::vector<uint8_t> to_byte_vector(T const &value) {
  std::vector<uint8_t> bytes;

  for (size_t i = 0; i < sizeof(value); i++) {
    uint8_t byte = value >> (i * 8);
    if constexpr (big_endian) {
      bytes.insert(bytes.begin(), byte);
    }
    else {
      bytes.insert(bytes.end(), byte);
    }
  }

  return bytes;
}

template <typename T,
          bool big_endian = true,
          typename std::enable_if<std::is_arithmetic<T>::value>::type * = nullptr>
T from_byte_vector(const std::vector<uint8_t> &bytes) {
  if(sizeof(T) != bytes.size())
    throw std::runtime_error("Byte length is not matching size requested of type");
  T result{0};
  for (size_t i = 0; i < sizeof(T); i++) {
    if constexpr (big_endian) {
        result |= bytes[i];
    } else {
      result |= bytes[sizeof(T) - i - 1];
    }
    if (i != sizeof(T) - 1)
      result <<= 8;
  }
  return result;
}

template <typename T,
          typename iterator,
          bool big_endian = true,
          typename std::enable_if<std::is_arithmetic<T>::value>::type * = nullptr>
T from_byte_iterator(iterator &byte_iterator) {
  T result{0};
  for (size_t i = 0; i < sizeof(T); i++) {
    if (byte_iterator == iterator())
      throw std::runtime_error("Iterator at end..");
    if constexpr (big_endian) {
      result <<= 8;
      result |= uint8_t(*byte_iterator++);
    } else {
      result |= T(*byte_iterator++) << (i * 8);
    }
  }
  return result;
}
