#ifndef ONLINE_STATS_H
#define ONLINE_STATS_H

class OnlineStats
{
  // online standard deviation
  // see: https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
  // section on incremental computation
public:
  OnlineStats() : n_(0), K_(0.0), sum_(0.0), sum2_(0.0) {};
  virtual ~OnlineStats() {};

  void addData(double x);     // add data itme; only mathematical effect, the data is not stored
  void removeData(double x);  // remove data item; only mathematical effect, the data is not stored

  double n()      const { return n_; }
  double mean()   const { return K_ + sum_ / n_; }
  double stdevp() const;      // standard deviation of population
  double stdevs() const;      // standard deviation of sample
private:
  int n_;         // number of data items seen so far
  double K_;      // assumed mean - for numerical stability
  double sum_;    // sum of data items seen so far
  double sum2_;   // squared sum of data items seen so far
};

#endif /* ONLINE_STATS_H */
