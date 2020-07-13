#include "RandomGenerator.h"

namespace copter_utils {

std::random_device RandomGenerator::rd;
std::mt19937 RandomGenerator::e2(RandomGenerator::rd());

}
