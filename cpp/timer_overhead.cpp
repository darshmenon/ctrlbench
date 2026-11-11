// Median cost of one back-to-back pair of clock reads, to compare with the Python timer.
#include <algorithm>
#include "common.hpp"
int main() {
  std::vector<double> t(200000);
  for (auto & x : t) { double a = cb::now_ns(); x = cb::now_ns() - a; }
  std::nth_element(t.begin(), t.begin() + t.size() / 2, t.end());
  std::printf("%.1f\n", t[t.size() / 2]);
}
