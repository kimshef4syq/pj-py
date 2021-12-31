// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <RVS/Common/Macros.h>
RVS_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateSampler.h>
RVS_COMMON_IGNORE_WARNINGS_POP
#include <RVS/Planner/Core/StateSampler.h>

namespace RVS
{
/// @addtogroup Planner
/// @{
namespace OB = ompl::base;

RVS_CLASS_FORWARD(StateSamplerWrapper);

/** @brief RVS StateSampler to ompl StateSampler*/
class StateSamplerWrapper : public OB::StateSampler
{
public:
    StateSamplerWrapper(const OB::RealVectorStateSpace *space,
                        const StateSamplerPtr &sampler);

    virtual ~StateSamplerWrapper() = default;

    virtual void sampleUniform(OB::State *state) override;

    virtual void sampleUniformNear(OB::State *state, const OB::State *near,
                                   double distance) override;

    virtual void sampleGaussian(OB::State *state, const OB::State *mean,
                                double stdDev) override;

    unsigned int GetDoF() const { return space_->getDimension(); }

    unsigned int GetMaxTries() const { return m_max_tries; }

    void SetMaxTries(const unsigned int max_tries) { m_max_tries = max_tries; }

private:
    void _JointVectorToOmplState(const JointVector &vector,
                                 OB::State *state) const;

private:
    StateSamplerPtr m_sampler;
    unsigned int m_max_tries;
};

/// @}
} // namespace RVS
