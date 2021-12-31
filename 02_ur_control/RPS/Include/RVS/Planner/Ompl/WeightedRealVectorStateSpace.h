// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <RVS/Common/Macros.h>
RVS_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/spaces/RealVectorStateSpace.h>
RVS_COMMON_IGNORE_WARNINGS_POP
#include <RVS/Planner/Ompl/StateSamplerWrapper.h>

namespace RVS
{
/// @addtogroup Planner
/// @{

namespace OB = ompl::base;

/**
 * @brief OMPL Weighted State Sampler.
 *
 * This allows you to provided a weight factor different
 * joint when sampling. This is useful when you have a
 * gantry with two linear axis with a robot attached. When
 * sampling near you may want to scale down the rail sampling.
 */
class WeightedRealVectorStateSampler : public OB::RealVectorStateSampler
{
public:
    WeightedRealVectorStateSampler(const OB::StateSpace *space)
        : OB::RealVectorStateSampler(space)
    {
    }

    /** \brief Sample a state such that each component state[i] is
        uniformly sampled from [near[i]-distance, near[i]+distance].
        If this interval exceeds the state space bounds, the
        interval is truncated. */
    virtual void sampleUniformNear(OB::State *state, const OB::State *near,
                                   double distance) override;

    /** \brief Sample a state such that each component state[i] has
        a Gaussian distribution with mean mean[i] and standard
        deviation std_dev. If the sampled value exceeds the state
        space boundary, it is thresholded to the nearest boundary. */
    virtual void sampleGaussian(OB::State *state, const OB::State *mean,
                                double std_dev) override;
};

/** \brief A state space representing R<sup>n</sup>. The distance function is
 * the weighted L2 norm. */
class WeightedRealVectorStateSpace : public OB::RealVectorStateSpace
{
public:
    /** \brief The definition of a state in R<sup>n</sup> */
    class StateType : public OB::State
    {
    public:
        StateType() = default;

        /** \brief Access element i of values.  This does not
            check whether the index is within bounds */
        double operator[](unsigned int i) const { return values[i]; }

        /** \brief Access element i of values.  This does not
            check whether the index is within bounds */
        double &operator[](unsigned int i) { return values[i]; }

        /** \brief The value of the actual vector in R<sup>n</sup> */
        double *values;
    };

    /** \brief Constructor. The dimension of of the space needs to be specified.
       A space representing R<sup>dim</sup> will be instantiated */
    WeightedRealVectorStateSpace(unsigned int dim = 0);

    virtual ~WeightedRealVectorStateSpace() override = default;

    virtual double distance(const OB::State *state1,
                            const OB::State *state2) const override;

    // virtual double getMaximumExtent() const override;

    // virtual double getMeasure() const override;

    virtual OB::StateSamplerPtr allocDefaultStateSampler() const override;

    void setStateSampler(const StateSamplerPtr &sampler);

    virtual OB::StateSamplerPtr allocStateSampler() const override;

    bool setWeights(const std::vector<double> &weights);

    const std::vector<double> &getWeights() const { return m_weights; }

    virtual void registerProjections() override {}

private:
    std::vector<double> m_weights;
    StateSamplerPtr m_sampler;
};

/// @}
} // namespace RVS