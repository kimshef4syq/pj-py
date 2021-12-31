// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include "StateConstraint.h"

namespace RVS
{
/// @addtogroup Planner
/// @{

RVS_CLASS_FORWARD(StateSampler);

/**
 * @brief StateSampler
 *
 */
class StateSampler : public std::enable_shared_from_this<StateSampler>
{
public:
    // non-copyable
    StateSampler(const StateSampler &) = delete;
    StateSampler &operator=(const StateSampler &) = delete;

    explicit StateSampler(const std::string &name) : m_name(std::move(name))
    {
        RVS_TRACE("Constructing StateSampler");
    }

    virtual ~StateSampler() { RVS_TRACE("Destroying StateSampler"); }

    const std::string &GetName() const { return m_name; }

    /**
     * @brief Sample a state
     *
     * @param states
     * @return bool, return false in case of failure.
     */
    virtual bool SampleUniform(JointVector &state) = 0;

    /**
     * @brief Sample a state near another, within specified distance.
     *
     * @param near
     * @param distance
     * @param state
     * @return bool Return false, in case of failure.
     */
    virtual bool SampleUniformNear(const JointVector &near,
                                   const double distance,
                                   JointVector &state) = 0;

    /**
     * @brief Sample a state using a Gaussian distribution with given \e mean
     * and standard deviation (\e stdDev).
     *
     *    As with SampleUniform, the implementation of SampleGaussian is
     *    specific to the derived class and few assumptions can be made
     *    about the distance between `state` and `mean`.
     *
     * @param mean mean state
     * @param std_dev standard deviation
     * @param state output state
     * @return bool Return false, in case of failure.
     */
    virtual bool SampleGaussian(const JointVector &mean, const double std_dev,
                                JointVector &state) = 0;

protected:
    std::string m_name;
};


RVS_CLASS_FORWARD(FixOrientationSampler);

/**
 * @brief This class is specifically used to randomly sample pose with fixed
 * orientation
 *
 */
class FixOrientationSampler : public StateSampler
{
public:
    explicit FixOrientationSampler(const ManipulatorPtr &manipulator);

    FixOrientationSampler(const ManipulatorPtr &manipulator,
                          const SO3d &ref_orientation);

    virtual ~FixOrientationSampler()
    {
        RVS_TRACE("Destroying FixOrientationSampler");
    }

    virtual bool SampleUniform(JointVector &state) override;

    virtual bool SampleUniformNear(const JointVector &near,
                                   const double distance,
                                   JointVector &state) override;

    virtual bool SampleGaussian(const JointVector &mean, const double std_dev,
                                JointVector &state) override;

    bool SetRefOrientation(const SO3d &ref_orientation);

    const SO3d &GetRefOrientation() const { return m_ref_orientation; }

private:
    bool _PoseToState(const SE3d &pose, const JointVector &joint_seed,
                      JointVector &state) const;

private:
    KinematicsBasePtr m_kin_solver;
    SE3d m_kin_base_pose;
    SO3d m_ref_orientation; ///< reference orientation in world frame
};


RVS_CLASS_FORWARD(BoxRegionSampler)

/**
 * @brief This class is specifically used to randomly sample joint states in a
 * box region
 *
 * @todo This is a dirty implementation, we use a joint_seed to find nearest IK,
 * but maybe we can GetAllIK() here, or use different joint_seeds
 *
 */
class BoxRegionSampler : public StateSampler
{
public:
    BoxRegionSampler(const ManipulatorPtr &manipulator, const Box &box_region);

    virtual ~BoxRegionSampler() { RVS_TRACE("Destroying BoxRegionSampler"); }

    virtual bool SampleUniform(JointVector &state) override;

    virtual bool SampleUniformNear(const JointVector &near,
                                   const double distance,
                                   JointVector &state) override;

    virtual bool SampleGaussian(const JointVector &mean, const double std_dev,
                                JointVector &state) override;

    bool SetJointSeed(const JointVector &seed);

    bool SetRefOrientation(const SO3d &ref_orientation);

    bool SetOrientationRange(const SO3Tangentd &orientation_range);

    const JointVector &GetJointSeed() const { return m_joint_seed; }

    const SO3d &GetRefOrientation() const { return m_ref_orientation; }

    const SO3Tangentd &GetOrientationRange() const
    {
        return m_orientation_range;
    };

    const Box &GetBoxRegion() const { return m_box_region; }

private:
    bool _PoseToState(const SE3d &pose, JointVector &state) const;

    bool _PoseToState(const SE3d &pose, const JointVector &joint_seed,
                      JointVector &state) const;

private:
    KinematicsBasePtr m_kin_solver;
    SE3d m_kin_base_pose;
    JointVector m_joint_seed; ///< joint seed to find the nearest ik
    SO3d m_ref_orientation; ///< reference orientation in world frame
    SO3Tangentd m_orientation_range;
    Box m_box_region; ///< box region
};

/**
 * @brief This class samples from the results of the graph generator,different
 * from the two samplers implemented above, this class can guarantee the 100%
 * success rate of sampling
 *
 */
RVS_CLASS_FORWARD(GraphSampler)
class GraphSampler : public StateSampler
{
public:
    GraphSampler(const std::vector<JointVector> &graph)
        : StateSampler("GraphSampler")
    {
        RVS_TRACE("Constructing GraphSampler");
        m_graph = graph;
    }

    virtual ~GraphSampler() { RVS_TRACE("Destroying GraphSampler"); }

    virtual bool SampleUniform(JointVector &state) override;

    virtual bool SampleUniformNear(const JointVector &near,
                                   const double distance, JointVector &state);

    virtual bool SampleGaussian(const JointVector &mean, const double std_dev,
                                JointVector &state);

private:
    std::vector<JointVector> m_graph;
};


/// @}
} // namespace RVS