
/* metric.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_METRIC_METRIC_HPP
#define MY_LIDAR_GRAPH_SLAM_METRIC_METRIC_HPP

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>

namespace MyLidarGraphSlam {
namespace Metric {

enum class MetricType
{
    None,
    Counter,
    Gauge,
    Distribution,
    Histogram,
    MetricFamily,
};

class MetricBase
{
public:
    /* Constructor */
    MetricBase(MetricType type,
               const std::string& metricId) :
        mType(type),
        mId(metricId) { }
    /* Destructor */
    virtual ~MetricBase() = default;

    /* Retrieve the metric type */
    virtual MetricType Type() const final { return this->mType; }
    /* Retrieve the metric Id string */
    virtual const std::string& Id() const final { return this->mId; }

protected:
    /* Metric type */
    MetricType  mType;
    /* Metric Id string */
    std::string mId;
};

class CounterBase : public MetricBase
{
public:
    /* Constructor */
    CounterBase(const std::string& metricId) :
        MetricBase(MetricType::Counter, metricId) { }
    /* Destructor */
    ~CounterBase() = default;

    /* Reset the counter value */
    virtual void Reset() = 0;

    /* Retrieve the counter value */
    virtual double Value() const = 0;

    /* Increment the counter by specified value */
    virtual void Increment(double val = 1.0) = 0;

    /* Dump the counter object */
    virtual void Dump(std::ostream& outStream) const = 0;
};

class NullCounter final : public CounterBase
{
public:
    /* Constructor */
    NullCounter() : CounterBase("") { }
    /* Destructor */
    ~NullCounter() = default;

    /* Reset the counter value */
    void Reset() override { }

    /* Retrieve the counter value */
    double Value() const override { return 0.0; }

    /* Increment the counter by specified value */
    void Increment(double) override { }

    /* Dump the counter object */
    void Dump(std::ostream&) const override { }
};

class Counter final : public CounterBase
{
public:
    /* Constructor */
    Counter(const std::string& metricId) :
        CounterBase(metricId),
        mValue(0.0) { }
    /* Constructor with default counter */
    Counter(const std::string& metricId,
            double initVal) :
        CounterBase(metricId),
        mValue(initVal) { }
    /* Destructor */
    ~Counter() = default;

    /* Reset the counter value */
    void Reset() override { this->mValue = 0.0; }

    /* Retrieve the counter value */
    double Value() const override { return this->mValue; }

    /* Increment the counter by specified value */
    void Increment(double val = 1.0) override
    { this->mValue += std::max(0.0, val); }

    /* Dump the counter object */
    void Dump(std::ostream& outStream) const override;

private:
    /* Counter value */
    double mValue;
};

class GaugeBase : public MetricBase
{
public:
    /* Constructor */
    GaugeBase(const std::string& metricId) :
        MetricBase(MetricType::Gauge, metricId) { }
    /* Destructor */
    ~GaugeBase() = default;

    /* Reset the gauge value */
    virtual void Reset() = 0;

    /* Retrieve the gauge value */
    virtual double Value() const = 0;
    /* Set the gauge value */
    virtual void SetValue(double val) = 0;

    /* Increment the gauge by specified value */
    virtual void Increment(double val = 1.0) = 0;
    /* Decrement the gauge by specified value */
    virtual void Decrement(double val = 1.0) = 0;

    /* Dump the gauge object */
    virtual void Dump(std::ostream& outStream) const = 0;
};

class NullGauge final : public GaugeBase
{
public:
    /* Constructor */
    NullGauge() : GaugeBase("") { }
    /* Destructor */
    ~NullGauge() = default;

    /* Reset the gauge value */
    void Reset() override { }

    /* Retrieve the gauge value */
    double Value() const override { return 0.0; }
    /* Set the gauge value */
    void SetValue(double) override { }

    /* Increment the gauge by specified value */
    void Increment(double) override { }
    /* Decrement the gauge by specified value */
    void Decrement(double) override { }

    /* Dump the gauge object */
    void Dump(std::ostream&) const override { }
};

class Gauge final : public GaugeBase
{
public:
    /* Constructor */
    Gauge(const std::string& metricId) :
        GaugeBase(metricId),
        mValue(0.0) { }
    /* Constructor with default value */
    Gauge(const std::string& metricId,
          double initVal) :
        GaugeBase(metricId),
        mValue(initVal) { }
    /* Destructor */
    ~Gauge() = default;

    /* Reset the gauge value */
    void Reset() override { this->mValue = 0.0; }

    /* Retrieve the gauge value */
    double Value() const override { return this->mValue; }
    /* Set the gauge value */
    void SetValue(double val) override { this->mValue = val; }

    /* Increment the gauge by specified value */
    void Increment(double val = 1.0) override { this->mValue += val; }
    /* Decrement the gauge by specified value */
    void Decrement(double val = 1.0) override { this->mValue -= val; }

    /* Dump the gauge object */
    void Dump(std::ostream& outStream) const override;

private:
    /* Gauge value */
    double mValue;
};

class DistributionBase : public MetricBase
{
public:
    /* Constructor */
    DistributionBase(const std::string& metricId) :
        MetricBase(MetricType::Distribution, metricId) { }
    /* Destructor */
    ~DistributionBase() = default;

    /* Reset the distribution */
    virtual void Reset() = 0;

    /* Observe the value and update mean and variance */
    virtual void Observe(double val) = 0;

    /* Retrieve the number of the observed values */
    virtual int NumOfSamples() const = 0;
    /* Retrieve the sum of the observed values */
    virtual double Sum() const = 0;
    /* Retrieve the mean of the observed values */
    virtual double Mean() const = 0;
    /* Retrieve the unbiased variance of the observed values */
    virtual double Variance() const = 0;
    /* Retrieve the standard deviation of the observed values */
    virtual double StandardDeviation() const = 0;
    /* Retrieve the maximum of the observed values */
    virtual double Maximum() const = 0;
    /* Retrieve the minimum of the observed values */
    virtual double Minimum() const = 0;

    /* Dump the distribution object */
    virtual void Dump(std::ostream& outStream) const = 0;
};

class NullDistribution final : public DistributionBase
{
public:
    /* Constructor */
    NullDistribution() : DistributionBase("") { }
    /* Destructor */
    ~NullDistribution() = default;

    /* Reset the distribution */
    void Reset() override { }

    /* Observe the value and update mean and variance */
    void Observe(double) override { }

    /* Retrieve the number of the observed values */
    int NumOfSamples() const override { return 0; }
    /* Retrieve the sum of the observed values */
    double Sum() const override { return 0.0; }
    /* Retrieve the mean of the observed values */
    double Mean() const override { return 0.0; }
    /* Retrieve the unbiased variance of the observed values */
    double Variance() const override { return 0.0; }
    /* Retrieve the standard deviation of the observed values */
    double StandardDeviation() const override { return 0.0; }
    /* Retrieve the maximum of the observed values */
    double Maximum() const override { return 0.0; }
    /* Retrieve the minimum of the observed values */
    double Minimum() const override { return 0.0; }

    /* Dump the distribution object */
    void Dump(std::ostream&) const override { }
};

class Distribution final : public DistributionBase
{
public:
    /* Constructor */
    Distribution(const std::string& metricId) :
        DistributionBase(metricId),
        mNumOfSamples(0),
        mSum(0.0),
        mMean(0.0),
        mScaledVariance(0.0),
        mMaximum(0.0),
        mMinimum(0.0) { }
    /* Destructor */
    ~Distribution() = default;

    /* Reset the distribution */
    void Reset() override;

    /* Observe the value and update mean and variance */
    void Observe(double val) override;

    /* Retrieve the number of the observed values */
    int NumOfSamples() const override { return this->mNumOfSamples; }
    /* Retrieve the sum of the observed values */
    double Sum() const override { return this->mSum; }
    /* Retrieve the mean of the observed values */
    double Mean() const override { return this->mMean; }
    /* Retrieve the unbiased variance of the observed values */
    double Variance() const override;
    /* Retrieve the standard deviation of the observed values */
    double StandardDeviation() const override;
    /* Retrieve the maximum of the observed values */
    double Maximum() const override { return this->mMaximum; }
    /* Retrieve the minimum of the observed values */
    double Minimum() const override { return this->mMinimum; }

    /* Dump the distribution object */
    void Dump(std::ostream& outStream) const override;

private:
    /* Number of the observed values */
    int    mNumOfSamples;
    /* Sum of the observed values */
    double mSum;
    /* Mean of the observed values */
    double mMean;
    /* Scaled variance of the observed values */
    double mScaledVariance;
    /* Maximum of the observed values */
    double mMaximum;
    /* Minimum of the observed values */
    double mMinimum;
};

class HistogramBase : public MetricBase
{
public:
    /* Type declarations */
    using BucketBoundaries = std::vector<double>;

    /* Constructor */
    HistogramBase(const std::string& metricId) :
        MetricBase(MetricType::Histogram, metricId) { }
    /* Destructor */
    ~HistogramBase() = default;

    /* Reset the histogram */
    virtual void Reset() = 0;

    /* Observe the value */
    virtual void Observe(double val) = 0;

    /* Retrieve the boundary values */
    virtual const BucketBoundaries& Boundaries() const = 0;
    /* Retrieve the bucket counts */
    virtual const std::vector<double>& Counts() const = 0;
    /* Retrieve the summation counter */
    virtual double SumValues() const = 0;

    /* Retrieve the number of observed values */
    virtual double NumOfSamples() const = 0;
    /* Retrieve the mean of the observed values */
    virtual double Mean() const = 0;

    /* Retrieve the value range of the specified bucket */
    virtual void ValueRange(std::size_t bucketIdx,
                            double& rangeMin,
                            double& rangeMax) const = 0;

    /* Dump the histogram object */
    virtual void Dump(std::ostream& outStream, bool isVerbose) const = 0;
};

class NullHistogram final : public HistogramBase
{
public:
    /* Constructor */
    NullHistogram() : HistogramBase("") { }
    /* Destructor */
    ~NullHistogram() = default;

    /* Reset the histogram */
    void Reset() override { }

    /* Observe the value */
    void Observe(double) override { }

    /* Retrieve the boundary values */
    const BucketBoundaries& Boundaries() const override
    { return this->mEmptyBucketBoundaries; }
    /* Retrieve the bucket counts */
    const std::vector<double>& Counts() const override
    { return this->mEmptyBucketCounts; }
    /* Retrieve the summation counter */
    double SumValues() const override { return 0.0; }

    /* Retrieve the number of observed values */
    double NumOfSamples() const override { return 0.0; }
    /* Retrieve the mean of the observed values */
    double Mean() const override { return 0.0; }

    /* Retrieve the value range of the specified bucket */
    void ValueRange(std::size_t bucketIdx,
                    double& rangeMin,
                    double& rangeMax) const override;

    /* Dump the histogram object */
    void Dump(std::ostream&, bool) const override { }

private:
    /* Empty bucket boundaries */
    BucketBoundaries    mEmptyBucketBoundaries;
    /* Empty bucket counts */
    std::vector<double> mEmptyBucketCounts;
};

class Histogram final : public HistogramBase
{
public:
    /* Create the bucket boundaries with fixed width */
    static BucketBoundaries CreateFixedWidthBoundaries(
        double startVal, double bucketWidth, int numOfFiniteBuckets);

    /* Create the bucket boundaries with fixed width */
    static BucketBoundaries CreateFixedWidthBoundaries(
        double startVal, double endVal, double bucketWidth);

    /* Create the bucket boundaries with exponential width */
    static BucketBoundaries CreateExponentialWidthBoundaries(
        double startVal, double endVal, double baseVal);

    /* Constructor */
    Histogram(const std::string& metricId,
              const BucketBoundaries& bucketBoundaries);
    /* Destructor */
    ~Histogram() = default;

    /* Reset the histogram */
    void Reset() override;

    /* Observe the value */
    void Observe(double val) override;

    /* Retrieve the boundary values */
    const BucketBoundaries& Boundaries() const override
    { return this->mBucketBoundaries; }
    /* Retrieve the bucket counts */
    const std::vector<double>& Counts() const override
    { return this->mBucketCounts; }
    /* Retrieve the summation counter */
    double SumValues() const override
    { return this->mSumValues; }

    /* Retrieve the number of observed values */
    double NumOfSamples() const override;
    /* Retrieve the mean of the observed values */
    double Mean() const override;

    /* Retrieve the value range of the specified bucket */
    void ValueRange(std::size_t bucketIdx,
                    double& rangeMin,
                    double& rangeMax) const override;

    /* Dump the histogram object */
    void Dump(std::ostream& outStream, bool isVerbose) const override;

private:
    /* Bucket boundaries */
    BucketBoundaries    mBucketBoundaries;
    /* Bucket counters */
    std::vector<double> mBucketCounts;
    /* Sum of the observed values */
    double              mSumValues;
};

template <typename T>
class MetricFamily final : public MetricBase
{
public:
    /* Constructor */
    MetricFamily(const std::string& metricId,
                 T* pNullMetric) :
        MetricBase(MetricType::MetricFamily, metricId),
        mNullMetric(pNullMetric) { }
    /* Destructor */
    ~MetricFamily() = default;

    /* Append the new metric */
    void Append(T* pMetric);
    /* Remove the metric */
    void Remove(const std::string& metricId);

    /* Retrieve the number of the metrics */
    inline std::size_t NumOfMetrics() const
    { return this->mMetrics.size(); }
    /* Retrieve the metric at a specified index */
    inline const T* MetricAt(std::size_t metricIdx) const
    { return this->mMetrics.at(metricIdx).get(); }
    /* Retrieve the metric at a specified index */
    T* MetricAt(std::size_t metricIdx)
    { return this->mMetrics.at(metricIdx).get(); }

    /* Retrieve the specified metric object */
    const T* Metric(const std::string& metricId) const;
    /* Retrieve the specified metric object */
    T* Metric(const std::string& metricId);

    /* Retrieve the specified metric object */
    const T* operator()(const std::string& metricId) const
    { return this->Metric(metricId); }
    /* Retrieve the specified metric object */
    T* operator()(const std::string& metricId)
    { return this->Metric(metricId); }

private:
    /* List of the metric Ids and metric objects */
    std::vector<std::unique_ptr<T>> mMetrics;
    /* Null metric */
    std::unique_ptr<T>              mNullMetric;
};

class MetricManager final
{
private:
    /* Constructor */
    MetricManager() :
        mCounterMetrics("Counters", new NullCounter()),
        mGaugeMetrics("Gauges", new NullGauge()),
        mDistributionMetrics("Distributions", new NullDistribution()),
        mHistogramMetrics("Histograms", new NullHistogram()) { }
    /* Destructor */
    ~MetricManager() = default;

public:
    /* Type definitions */
    using ptree = boost::property_tree::ptree;

    /* Copy constructor (disabled) */
    MetricManager(const MetricManager&) = delete;
    /* Copy assignment operator (disabled) */
    MetricManager& operator=(const MetricManager&) = delete;
    /* Move constructor (disabled) */
    MetricManager(MetricManager&&) = delete;
    /* Move assignment operator (disabled) */
    MetricManager& operator=(MetricManager&&) = delete;

    /* Get the MetricManager singleton instance */
    static MetricManager* Instance();

    /* Convert all metrics to the Boost property tree */
    ptree ToPropertyTree() const;

    /* Retrieve the counter metrics */
    const MetricFamily<CounterBase>& CounterMetrics() const
    { return this->mCounterMetrics; }
    /* Retrieve the counter metrics */
    MetricFamily<CounterBase>& CounterMetrics()
    { return this->mCounterMetrics; }

    /* Retrieve the gauge metrics */
    const MetricFamily<GaugeBase>& GaugeMetrics() const
    { return this->mGaugeMetrics; }
    /* Retrieve the gauge metrics */
    MetricFamily<GaugeBase>& GaugeMetrics()
    { return this->mGaugeMetrics; }

    /* Retrieve the distribution metrics */
    const MetricFamily<DistributionBase>& DistributionMetrics() const
    { return this->mDistributionMetrics; }
    /* Retrieve the distribution metrics */
    MetricFamily<DistributionBase>& DistributionMetrics()
    { return this->mDistributionMetrics; }

    /* Retrieve the histogram metrics */
    const MetricFamily<HistogramBase>& HistogramMetrics() const
    { return this->mHistogramMetrics; }
    /* Retrieve the histogram metrics */
    MetricFamily<HistogramBase>& HistogramMetrics()
    { return this->mHistogramMetrics; }

private:
    /* List of the counter metrics */
    MetricFamily<CounterBase>      mCounterMetrics;
    /* List of the gauge metrics */
    MetricFamily<GaugeBase>        mGaugeMetrics;
    /* List of the distribution metrics */
    MetricFamily<DistributionBase> mDistributionMetrics;
    /* List of the histogram metrics */
    MetricFamily<HistogramBase>    mHistogramMetrics;
};

} /* namespace Metric */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_METRIC_METRIC_HPP */
