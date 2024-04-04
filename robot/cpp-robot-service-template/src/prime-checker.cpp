#include <cstdint>
#include <iostream>

#include "prime-checker.hpp"

// slighly rewritten to avoid nesting and unnessary declarations
bool PrimeChecker::isPrime(uint16_t n) {
  if (n < 2 || (n % 2 == 0 && n != 2)) {
    return false;
  }
  for (uint16_t i = 3; (i * i) <= n; i += 2) {
    if (n % i == 0) {
      return false;
    }
  }
  return true;
}
