#include "OnlineStats.h"
#include <cmath>

void OnlineStats::addData(double x) {
  if (n_ == 0) K_ = x;
  auto d = x - K_;
  sum_ += d;
  sum2_ += d*d;
  ++n_;
}

void OnlineStats::removeData(double x) {
  auto d = x - K_;
  sum_ -= d;
  sum2_ -= d*d;
  --n_;
}

double OnlineStats::stdevp() const {
  if (n_ < 1) return 0.0;
  return sqrt((sum2_ - (sum_*sum_)/n_)/n_);
}

double OnlineStats::stdevs() const {
  if (n_ < 1) return 0.0;
  return sqrt((sum2_ - (sum_*sum_) / n_) / (n_-1));
}
