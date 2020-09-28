
/* robust_loss_function.hpp */

#ifndef MY_LIDAR_GRAPH_SLAM_MAPPING_ROBUST_LOSS_FUNCTION_HPP
#define MY_LIDAR_GRAPH_SLAM_MAPPING_ROBUST_LOSS_FUNCTION_HPP

#include <memory>

namespace MyLidarGraphSlam {
namespace Mapping {

/* Type definitions for convenience */
class LossFunction;
using LossFunctionPtr = std::shared_ptr<LossFunction>;

/*
 * Base class for robust loss functions
 */
class LossFunction
{
public:
    /* Constructor */
    LossFunction() = default;
    /* Destructor */
    virtual ~LossFunction() = default;

    /* Compute a loss */
    virtual double Loss(double squaredError) const = 0;
    /* Compute a weight */
    virtual double Weight(double squaredError) const = 0;
};

/*
 * Squared loss function (Gaussian and not robust)
 */
class LossSquared final : public LossFunction
{
public:
    /* Constructor */
    LossSquared() = default;
    /* Destructor */
    ~LossSquared() = default;

    /* Compute a loss defined as follows
     * \rho(t) = t */
    double Loss(double squaredError) const override { return squaredError; }
    /* Compute a weight defined as follows
     * \rho'(t) = 1 */
    double Weight(double) const override { return 1.0; }
};

/*
 * Huber loss function
 */
class LossHuber final : public LossFunction
{
public:
    /* Constructor */
    LossHuber(double scale);
    /* Destructor */
    ~LossHuber() = default;

    /* Compute a loss defined as follows
     * \rho(t) = t (t \le s), \rho(t) = 2 \sqrt{st} - s (t > s) */
    double Loss(double squaredError) const override;
    /* Compute a weight defined as follows
     * \rho'(t) = 1 (t \le s), \rho'(t) = \sqrt{s / t} (t > s) */
    double Weight(double squaredError) const override;

private:
    /* Scale parameter */
    double mScale;
};

/*
 * Cauchy loss function
 */
class LossCauchy final : public LossFunction
{
public:
    /* Constructor */
    LossCauchy(double scale);
    /* Destructor */
    ~LossCauchy() = default;

    /* Compute a loss defined as follows
     * \rho(t) = s \ln (1 + t / s) */
    double Loss(double squaredError) const override;
    /* Compute a weight defined as follows
     * \rho'(t) = 1 / (1 + t / s) = s / (s + t) */
    double Weight(double squaredError) const override;

private:
    /* Scale parameter */
    double mScale;
};

/*
 * Fair loss function
 */
class LossFair final : public LossFunction
{
public:
    /* Constructor */
    LossFair(double scale);
    /* Destructor */
    ~LossFair() = default;

    /* Compute a loss function defined as follows
     * \rho(t) = 2s (\sqrt{t / s} - \ln(1 + \sqrt{t / s})) */
    double Loss(double squaredError) const override;
    /* Compute a weight defined as follows
     * \rho'(t) = 1 / (1 + \sqrt{t / s}) */
    double Weight(double squaredError) const override;

private:
    /* Scale parameter */
    double mScale;
};

/*
 * Geman-McClure loss function
 */
class LossGemanMcClure final : public LossFunction
{
public:
    /* Constructor */
    LossGemanMcClure(double scale);
    /* Destructor */
    ~LossGemanMcClure() = default;

    /* Compute a loss function defined as follows
     * \rho(t) = s(1 - 1 / (1 + t / s)) = st / (s + t) */
    double Loss(double squaredError) const override;
    /* Compute a weight defined as follows
     * \rho'(t) = 1 / (1 + t / s)^2 = s^2 / (s + t)^2 */
    double Weight(double squaredError) const override;

private:
    /* Scale parameter */
    double mScale;
};

/*
 * Welsch loss function
 */
class LossWelsch final : public LossFunction
{
public:
    /* Constructor */
    LossWelsch(double scale);
    /* Destructor */
    ~LossWelsch() = default;

    /* Compute a loss function defined as follows
     * \rho(t) = s(1 - \exp(-t / s)) */
    double Loss(double squaredError) const override;
    /* Compute a weight defined as follows
     * \rho'(t) = \exp(-t / s) */
    double Weight(double squaredError) const override;

private:
    /* Scale parameter */
    double mScale;
};

/*
 * Dynamic Covariance Scaling (DCS) loss function
 */
class LossDCS final : public LossFunction
{
public:
    /* Constructor */
    LossDCS(double scale);
    /* Destructor */
    ~LossDCS() = default;

    /* Compute a loss function defined as follows
     * \rho(t) = (s / (s + t))^2 t + (1 - s / (s + t))^2 s =
     * s^2 t / (s + t)^2 + (t / (s + t))^2 s =
     * s^2 t / (s + t)^2 + t^2 s / (s + t)^2 =
     * st (s + t) / (s + t)^2 = st / (s + t) */
    double Loss(double squaredError) const override;
    /* Compute a weight defined as follows
     * \rho'(t) = 1 (t <= s), \rho'(t) = (2s / (s + t))^2 (t > s) */
    double Weight(double squaredError) const override;

private:
    /* Scale parameter */
    double mScale;
};

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */

#endif /* MY_LIDAR_GRAPH_SLAM_MAPPING_ROBUST_LOSS_FUNCTION_HPP */
