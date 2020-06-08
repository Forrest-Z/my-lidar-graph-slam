
/* metric.cpp */

#include "my_lidar_graph_slam/metric/metric.hpp"

namespace MyLidarGraphSlam {
namespace Metric {

/*
 * Counter class implementations
 */

/* Dump the counter object */
void Counter::Dump(std::ostream& outStream) const
{
    outStream << "Counter Id: " << this->mId << ", "
              << "Value: " << this->mValue << '\n';
}

/*
 * Gauge class implementations
 */

/* Dump the gauge object */
void Gauge::Dump(std::ostream& outStream) const
{
    outStream << "Gauge Id: " << this->mId << ", "
              << "Value: " << this->mValue << '\n';
}

/*
 * Distribution class implementations
 */

/* Observe the value and update mean and variance */
void Distribution::Observe(double val)
{
    ++this->mNumOfSamples;
    this->mSum += val;

    if (this->mNumOfSamples == 1) {
        this->mMean = val;
        this->mScaledVariance = 0.0;
        this->mMaximum = val;
        this->mMinimum = val;
    } else {
        const double prevMean = this->mMean;
        this->mMean += (val - prevMean) / this->mNumOfSamples;
        this->mScaledVariance += (val - prevMean) * (val - this->mMean);
        this->mMaximum = std::max(this->mMaximum, val);
        this->mMinimum = std::min(this->mMinimum, val);
    }
}

/* Retrieve the unbiased variance of the observed values */
double Distribution::Variance() const
{
    return this->mNumOfSamples > 1 ?
        this->mScaledVariance / (this->mNumOfSamples - 1) : 0.0;
}

/* Retrieve the standard deviation of the observed values */
double Distribution::StandardDeviation() const
{
    return std::sqrt(this->Variance());
}

/* Dump the distribution object */
void Distribution::Dump(std::ostream& outStream) const
{
    outStream << "Distribution Id: " << this->mId << ", "
              << "Number of samples: " << this->mNumOfSamples << ", "
              << "Sum: " << this->mSum << ", "
              << "Mean: " << this->mMean << ", "
              << "Variance: " << this->Variance() << ", "
              << "Standard deviation: " << this->StandardDeviation() << ", "
              << "Max: " << this->mMaximum << ", "
              << "Min: " << this->mMinimum << '\n';
}

/*
 * NullHistogram class implementations
 */

/* Retrieve the value range of the specified bucket */
void NullHistogram::ValueRange(std::size_t,
                               double& rangeMin,
                               double& rangeMax) const
{
    rangeMin = 0.0;
    rangeMax = 0.0;
}

/*
 * Histogram class implementations
 */

/* Create the bucket boundaries with fixed width */
Histogram::BucketBoundaries Histogram::CreateFixedWidthBoundaries(
    double startVal, double bucketWidth, int numOfFiniteBuckets)
{
    /* Input checks */
    assert(bucketWidth > 0.0);
    assert(numOfFiniteBuckets >= 0);

    /* Compute bucket boundaries */
    BucketBoundaries bucketBoundaries;
    bucketBoundaries.reserve(numOfFiniteBuckets + 1);

    double boundary = startVal;
    bucketBoundaries.emplace_back(boundary);

    for (int i = 0; i < numOfFiniteBuckets; ++i) {
        boundary += bucketWidth;
        bucketBoundaries.emplace_back(boundary);
    }

    return bucketBoundaries;
}

/* Create the bucket boundaries with fixed width */
Histogram::BucketBoundaries Histogram::CreateFixedWidthBoundaries(
    double startVal, double endVal, double bucketWidth)
{
    /* Input checks */
    assert(endVal >= startVal);
    assert(bucketWidth > 0.0);

    /* Compute bucket boundaries */
    const int numOfFiniteBuckets = static_cast<int>(
        std::ceil((endVal - startVal) / bucketWidth));
    return Histogram::CreateFixedWidthBoundaries(
        startVal, bucketWidth, numOfFiniteBuckets);
}

/* Create the bucket boundaries with exponential width */
Histogram::BucketBoundaries Histogram::CreateExponentialWidthBoundaries(
    double startVal, double endVal, double baseVal)
{
    /* Input checks */
    assert(startVal > 0.0);
    assert(endVal >= startVal);
    assert(baseVal > 1.0);

    /* Compute bucket boundaries */
    BucketBoundaries bucketBoundaries;
    double boundary = startVal;

    while (boundary < endVal) {
        bucketBoundaries.emplace_back(boundary);
        boundary *= baseVal;
    }

    return bucketBoundaries;
}

/* Constructor */
Histogram::Histogram(const std::string& metricId,
                     const BucketBoundaries& bucketBoundaries) :
    HistogramBase(metricId),
    mBucketBoundaries(bucketBoundaries),
    mBucketCounts(bucketBoundaries.size() + 1),
    mSumValues(0.0)
{
    assert(!this->mBucketBoundaries.empty());
    assert(std::is_sorted(this->mBucketBoundaries.cbegin(),
                          this->mBucketBoundaries.cend()));
}

/* Observe the value */
void Histogram::Observe(double val)
{
    /* Determine the bucket index */
    const auto bucketIt = std::find_if(
        this->mBucketBoundaries.cbegin(),
        this->mBucketBoundaries.cend(),
        [val](const double boundary) { return val < boundary; });
    const auto bucketIdx = static_cast<std::size_t>(
        std::distance(this->mBucketBoundaries.cbegin(), bucketIt));

    /* Update the bucket counters */
    this->mBucketCounts[bucketIdx] += 1.0;

    /* Update the value sum */
    this->mSumValues += val;
}

/* Retrieve the number of observed values */
double Histogram::NumOfSamples() const
{
    /* Compute the number of the observed values */
    const double numOfSamples = std::accumulate(
        this->mBucketCounts.cbegin(),
        this->mBucketCounts.cend(),
        0.0);

    return numOfSamples;
}

/* Retrieve the mean of the observed values */
double Histogram::Mean() const
{
    /* Return the mean of the observed values */
    return this->mSumValues / this->NumOfSamples();
}

/* Retrieve the value range of the specified bucket */
void Histogram::ValueRange(std::size_t bucketIdx,
                           double& rangeMin,
                           double& rangeMax) const
{
    /* Input checks */
    assert(bucketIdx < this->mBucketCounts.size());

    /* Set the value range of the specified bucket */
    rangeMin = (bucketIdx == 0) ?
        std::numeric_limits<double>::min() :
        this->mBucketBoundaries[bucketIdx - 1];
    rangeMax = (bucketIdx == this->mBucketCounts.size() - 1) ?
        std::numeric_limits<double>::max() :
        this->mBucketBoundaries[bucketIdx];

    return;
}

/* Dump the histogram object */
void Histogram::Dump(std::ostream& outStream) const
{
    outStream << "Histogram Id: " << this->mId << ", "
              << "Number of samples: " << this->NumOfSamples() << ", "
              << "Sum: " << this->mSumValues << ", "
              << "Mean: " << this->Mean() << '\n';

    /* Dump the histogram (number of the values in each bucket) */
    outStream << " - " << this->mBucketBoundaries.front() << ": "
              << this->mBucketCounts.front() << '\n';

    for (std::size_t i = 0; i < this->mBucketBoundaries.size() - 1; ++i)
        outStream << this->mBucketBoundaries[i] << " - "
                  << this->mBucketBoundaries[i + 1] << ": "
                  << this->mBucketCounts[i + 1] << '\n';

    outStream << this->mBucketBoundaries.back() << " - : "
              << this->mBucketCounts.back() << '\n';
}

/*
 * MetricFamily class implementations
 */

/* Append the new metric */
template <typename T>
void MetricFamily<T>::Append(T* pMetric)
{
    const auto metricIt = std::find_if(
        this->mMetrics.cbegin(), this->mMetrics.cend(),
        [pMetric](const std::unique_ptr<T>& m) {
            return m->Id() == pMetric->Id(); });
    assert(metricIt == this->mMetrics.end());
    static_cast<void>(metricIt);
    this->mMetrics.push_back(std::unique_ptr<T>(pMetric));
}

/* Remove the metric */
template <typename T>
void MetricFamily<T>::Remove(const std::string& metricId)
{
    const auto metricIt = std::find_if(
        this->mMetrics.cbegin(), this->mMetrics.cend(),
        [metricId](const std::unique_ptr<T>& m) {
            return m->Id() == metricId; });
    assert(metricIt != this->mMetrics.end());
    this->mMetrics.erase(metricIt);
}

/* Retrieve the specified metric object */
template <typename T>
const T* MetricFamily<T>::Metric(const std::string& metricId) const
{
    const auto metricIt = std::find_if(
        this->mMetrics.cbegin(), this->mMetrics.cend(),
        [metricId](const std::unique_ptr<T>& m) {
            return m->Id() == metricId; });

    if (metricIt == this->mMetrics.end())
        return this->mNullMetric.get();

    const auto metricIdx = static_cast<std::size_t>(
        std::distance(this->mMetrics.cbegin(), metricIt));
    return this->mMetrics.at(metricIdx).get();
}

/* Retrieve the specified metric object */
template <typename T>
T* MetricFamily<T>::Metric(const std::string& metricId)
{
    return const_cast<T*>(
        static_cast<const MetricFamily<T>&>(*this).Metric(metricId));
}

/* Template class declarations */
template class MetricFamily<CounterBase>;
template class MetricFamily<GaugeBase>;
template class MetricFamily<DistributionBase>;
template class MetricFamily<HistogramBase>;

/*
 * MetricManager class implementations
 */

/* Get the MetricManager singleton instance */
MetricManager* MetricManager::Instance()
{
    static MetricManager theInstance;
    return &theInstance;
}

/* Convert all metrics to the Boost property tree */
MetricManager::ptree MetricManager::ToPropertyTree() const
{
    ptree rootTree;

    /* Process counter metrics */
    const std::size_t numOfCounters = this->mCounterMetrics.NumOfMetrics();
    ptree counterTree;

    for (std::size_t i = 0; i < numOfCounters; ++i) {
        const CounterBase* pCounter = this->mCounterMetrics.MetricAt(i);
        ptree metricTree;
        metricTree.put("Id", pCounter->Id());
        metricTree.put("Value", pCounter->Value());
        counterTree.push_back(std::make_pair("", metricTree));
    }

    /* Process gauge metrics */
    const std::size_t numOfGauges = this->mGaugeMetrics.NumOfMetrics();
    ptree gaugeTree;

    for (std::size_t i = 0; i < numOfGauges; ++i) {
        const GaugeBase* pGauge = this->mGaugeMetrics.MetricAt(i);
        ptree metricTree;
        metricTree.put("Id", pGauge->Id());
        metricTree.put("Value", pGauge->Value());
        gaugeTree.push_back(std::make_pair("", metricTree));
    }

    /* Process distribution metrics */
    const std::size_t numOfDistributions =
        this->mDistributionMetrics.NumOfMetrics();
    ptree distributionTree;

    for (std::size_t i = 0; i < numOfDistributions; ++i) {
        const DistributionBase* pDistribution =
            this->mDistributionMetrics.MetricAt(i);
        ptree distTree;
        distTree.put("Id", pDistribution->Id());
        distTree.put("NumOfSamples", pDistribution->NumOfSamples());
        distTree.put("Sum", pDistribution->Sum());
        distTree.put("Mean", pDistribution->Mean());
        distTree.put("StandardDeviation", pDistribution->StandardDeviation());
        distTree.put("Maximum", pDistribution->Maximum());
        distTree.put("Minimum", pDistribution->Minimum());
        distributionTree.push_back(std::make_pair("", distTree));
    }

    /* Process histogram metrics */
    const std::size_t numOfHistograms =
        this->mHistogramMetrics.NumOfMetrics();
    ptree histogramTree;

    for (std::size_t i = 0; i < numOfHistograms; ++i) {
        const HistogramBase* pHistogram =
            this->mHistogramMetrics.MetricAt(i);
        ptree metricTree;
        metricTree.put("Id", pHistogram->Id());
        metricTree.put("NumOfSamples", pHistogram->NumOfSamples());
        metricTree.put("SumValues", pHistogram->SumValues());

        const double safeMean = pHistogram->NumOfSamples() > 0.0 ?
                                pHistogram->Mean() : 0.0;
        metricTree.put("Mean", safeMean);

        /* Process bucket boundaries */
        ptree boundariesTree;
        for (const double boundary : pHistogram->Boundaries()) {
            ptree elementTree;
            elementTree.put_value(boundary);
            boundariesTree.push_back(std::make_pair("", elementTree));
        }

        /* Process bucket counts */
        ptree countsTree;
        for (const double count : pHistogram->Counts()) {
            ptree elementTree;
            elementTree.put_value(count);
            countsTree.push_back(std::make_pair("", elementTree));
        }

        metricTree.add_child("BucketBoundaries", boundariesTree);
        metricTree.add_child("BucketCounts", countsTree);
        histogramTree.push_back(std::make_pair("", metricTree));
    }

    rootTree.add_child("Counters", counterTree);
    rootTree.add_child("Gauges", gaugeTree);
    rootTree.add_child("Distributions", distributionTree);
    rootTree.add_child("Histograms", histogramTree);

    return rootTree;
}

} /* namespace Metric */
} /* namespace MyLidarGraphSlam */
