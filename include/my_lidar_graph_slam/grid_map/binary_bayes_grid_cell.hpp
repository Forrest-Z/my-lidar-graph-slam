
/* binary_bayes_grid_cell.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_GRID_MAP_BINARY_BAYES_GRID_CELL_HPP
#define MY_LIDAR_GRAPH_SLAM_GRID_MAP_BINARY_BAYES_GRID_CELL_HPP

#include "my_lidar_graph_slam/grid_map/grid_cell.hpp"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <utility>

namespace MyLidarGraphSlam {

template <typename T>
class BinaryBayesGridCell final : public GridCell<T, double>
{
public:
    /* Default constructor */
    BinaryBayesGridCell() : GridCell<T, double>(),
                            mValue(Unknown) { }
    /* Destructor */
    ~BinaryBayesGridCell() = default;

    /* Copy constructor */
    BinaryBayesGridCell(const BinaryBayesGridCell&) = default;
    /* Copy assignment operator */
    BinaryBayesGridCell& operator=(const BinaryBayesGridCell&) = default;
    /* Move constructor */
    BinaryBayesGridCell(BinaryBayesGridCell&&) noexcept = default;
    /* Move assignment operator */
    BinaryBayesGridCell& operator=(BinaryBayesGridCell&&) noexcept = default;

    /* Cast operator */
    explicit operator T() const override { return this->mValue; }
    /* Get the occupancy probability value of the grid cell */
    T Value() const override { return this->mValue; }

    /* Reset the grid cell state */
    void Reset() override;

    /* Update the value of the grid cell */
    void Update(const double& probValue) override;

    /* Unknown occupancy probability value */
    using GridCell<T, double>::Unknown;

    /* Smallest possible occupancy probability value */
    static constexpr T ProbabilityMin = static_cast<T>(1e-3);
    /* Largest possible occupancy probability value */
    static constexpr T ProbabilityMax = static_cast<T>(1.0 - ProbabilityMin);

    /* Clamp a occupancy probability value */
    static T ClampValue(T probValue);
    /* Compute a odds from the occupancy probability value */
    static T ValueToOdds(T probValue);
    /* Compute an occupancy probability value from the odds */
    static T OddsToValue(T oddsValue);

private:
    /* Occupancy probability value */
    T mValue;
};

/* Reset the grid cell state */
template <typename T>
void BinaryBayesGridCell<T>::Reset()
{
    this->mValue = Unknown;
}

/* Update the value of the grid cell */
template <typename T>
void BinaryBayesGridCell<T>::Update(const double& probValue)
{
    /* Assign a predefined probability value
     * if this grid cell is formerly not observed */
    if (this->mValue == Unknown) {
        /* Clamp the occupancy probability value for safety */
        this->mValue = ClampValue(probValue);
        return;
    }
    
    /* Otherwise, update the probability value using Binary Bayes Filter */
    const T oldOdds = ValueToOdds(this->mValue);
    const T valueOdds = ValueToOdds(probValue);
    const T newValue = OddsToValue(oldOdds * valueOdds);

    /* Clamp the occupancy probability value for safety */
    this->mValue = ClampValue(newValue);
}

/* Clamp a occupancy probability value */
template <typename T>
T BinaryBayesGridCell<T>::ClampValue(T probValue)
{
    /* Clamp a occupancy probability value */
    return std::clamp(probValue, ProbabilityMin, ProbabilityMax);
}

/* Compute a odds from the occupancy probability value */
template <typename T>
T BinaryBayesGridCell<T>::ValueToOdds(T probValue)
{
    /* Ensure that the probability value is not unknown */
    assert(probValue != Unknown);
    /* Clamp the occupancy probability value to avoid zero division */
    const T clampedValue = ClampValue(probValue);
    /* Compute the odds from the occupancy probability value */
    return clampedValue / (1.0 - clampedValue);
}

/* Compute an occupancy probability value from the odds */
template <typename T>
T BinaryBayesGridCell<T>::OddsToValue(T oddsValue)
{
    return ClampValue(oddsValue / (1.0 + oddsValue));
}

} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_GRID_MAP_BINARY_BAYES_GRID_CELL_HPP */
