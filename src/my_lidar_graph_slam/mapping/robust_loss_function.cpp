
/* robust_loss_function.cpp */

#include "my_lidar_graph_slam/mapping/robust_loss_function.hpp"

#include <cassert>
#include <cmath>

namespace MyLidarGraphSlam {
namespace Mapping {

/*
 * LossHuber class implementations
 */

/* Constructor */
LossHuber::LossHuber(double scale) :
    LossFunction(), mScale(scale)
{
    /* Scale parameter must be positive or zero */
    assert(this->mScale >= 0.0);
}

/* Compute a loss defined as follows
 * \rho(t) = t (t \le s), \rho(t) = 2 \sqrt{st} - s (t > s) */
double LossHuber::Loss(double squaredError) const
{
    assert(squaredError >= 0.0);
    return (squaredError <= this->mScale) ? squaredError :
        (2.0 * std::sqrt(this->mScale * squaredError) - this->mScale);
}

/* Compute a weight defined as follows
 * \rho'(t) = 1 (t \le s), \rho'(t) = \sqrt{s / t} (t > s) */
double LossHuber::Weight(double squaredError) const
{
    assert(squaredError >= 0.0);
    return (squaredError <= this->mScale) ? 1.0 :
        std::sqrt(this->mScale / squaredError);
}

/*
 * LossCauchy class implementations
 */

/* Constructor */
LossCauchy::LossCauchy(double scale) :
    LossFunction(), mScale(scale)
{
    /* Scale parameter must be positive or zero */
    assert(this->mScale >= 0.0);
}

/* Compute a loss defined as follows
 * \rho(t) = s \ln (1 + t / s) */
double LossCauchy::Loss(double squaredError) const
{
    assert(squaredError >= 0.0);
    return this->mScale * std::log1p(squaredError / this->mScale);
}

/* Compute a weight defined as follows
 * \rho'(t) = 1 / (1 + t / s) = s / (s + t) */
double LossCauchy::Weight(double squaredError) const
{
    assert(squaredError >= 0.0);
    return this->mScale / (this->mScale + squaredError);
}

/*
 * LossFair class implementations
 */

/* Constructor */
LossFair::LossFair(double scale) :
    LossFunction(), mScale(scale)
{
    /* Scale parameter must be positive or zero */
    assert(this->mScale >= 0.0);
}

/* Compute a loss function defined as follows
 * \rho(t) = 2s (\sqrt{t / s} - \ln(1 + \sqrt{t / s})) */
double LossFair::Loss(double squaredError) const
{
    assert(squaredError >= 0.0);
    const double sqrtError = std::sqrt(squaredError / this->mScale);
    return 2.0 * this->mScale * (sqrtError - std::log1p(sqrtError));
}

/* Compute a weight defined as follows
 * \rho'(t) = 1 / (1 + \sqrt{t / s}) */
double LossFair::Weight(double squaredError) const
{
    assert(squaredError >= 0.0);
    const double sqrtError = std::sqrt(squaredError / this->mScale);
    return 1.0 / (1.0 + sqrtError);
}

/*
 * LossGemanMcClure class implementations
 */

/* Constructor */
LossGemanMcClure::LossGemanMcClure(double scale) :
    LossFunction(), mScale(scale)
{
    /* Scale parameter must be positive or zero */
    assert(this->mScale >= 0.0);
}

/* Compute a loss function defined as follows
 * \rho(t) = s(1 - 1 / (1 + t / s)) = st / (s + t) */
double LossGemanMcClure::Loss(double squaredError) const
{
    assert(squaredError >= 0.0);
    return this->mScale * squaredError / (this->mScale + squaredError);
}

/* Compute a weight defined as follows
 * \rho'(t) = 1 / (1 + t / s)^2 = s^2 / (s + t)^2 */
double LossGemanMcClure::Weight(double squaredError) const
{
    assert(squaredError >= 0.0);
    const double squaredScale = this->mScale * this->mScale;
    const double errorPlusScale = this->mScale + squaredError;
    return squaredScale / (errorPlusScale * errorPlusScale);
}

/*
 * LossWelsch class implementations
 */

/* Constructor */
LossWelsch::LossWelsch(double scale) :
    LossFunction(), mScale(scale)
{
    /* Scale parameter must be positive or zero */
    assert(this->mScale >= 0.0);
}

/* Compute a loss function defined as follows
 * \rho(t) = s(1 - \exp(-t / s)) */
double LossWelsch::Loss(double squaredError) const
{
    assert(squaredError >= 0.0);
    return this->mScale * (-std::expm1(-squaredError / this->mScale));
}

/* Compute a weight defined as follows
 * \rho'(t) = \exp(-t / s) */
double LossWelsch::Weight(double squaredError) const
{
    assert(squaredError >= 0.0);
    return std::exp(-squaredError / this->mScale);
}

/*
 * LossDCS class implementations
 */

/* Constructor */
LossDCS::LossDCS(double scale) :
    LossFunction(), mScale(scale)
{
    /* Scale parameter must be positive or zero */
    assert(this->mScale >= 0.0);
}

/* Compute a loss function defined as follows
 * \rho(t) = (s / (s + t))^2 t + (1 - s / (s + t))^2 s =
 * s^2 t / (s + t)^2 + (t / (s + t))^2 s =
 * s^2 t / (s + t)^2 + t^2 s / (s + t)^2 =
 * st (s + t) / (s + t)^2 = st / (s + t) */
double LossDCS::Loss(double squaredError) const
{
    assert(squaredError >= 0.0);
    return this->mScale * squaredError / (this->mScale + squaredError);
}

/* Compute a weight defined as follows
 * \rho'(t) = 1 (t <= s), \rho'(t) = (2s / (s + t))^2 (t > s) */
double LossDCS::Weight(double squaredError) const
{
    assert(squaredError >= 0.0);
    return (squaredError <= this->mScale) ? 1.0 :
        std::pow(2.0 * this->mScale / (squaredError + this->mScale), 2.0);
}

} /* namespace Mapping */
} /* namespace MyLidarGraphSlam */
