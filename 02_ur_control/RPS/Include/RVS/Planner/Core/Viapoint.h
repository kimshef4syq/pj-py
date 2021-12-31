// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Planner/Core/Region.h>
#include <RVS/KinematicsHeader.h>
#include <RVS/Kinematics/Types.h>


namespace RVS
{
/// @addtogroup Planner
/// @{

/**
 * @brief TimeConstraint for an Viapoint
 * It gives a time specification with a lower and a upper bound to the viapoint
 */
struct TimeConstraint
{
    /** @brief Construct a new TimeConstraint object,
     *  default constructor is a non-specified time constraint */
    TimeConstraint() : lower(-1.0), upper(-1.0) {}

    /** @brief Construct a new TimeConstraint object,
     *  @param nominal: construct with a time stamp*/
    TimeConstraint(double nominal)
    {
        if (nominal < 0.0) {
            RVS_ERROR("nominal time should not be < 0");
            lower = -1.0;
            upper = -1.0;
        }

        lower = nominal;
        upper = nominal;
    }

    /** @brief Construct a new TimeConstraint object,
     *  @param lower: the lower bound of TimeConstraint
     *  @param upper: the upper bound of TimeConstraint
     * */
    TimeConstraint(double lower_in, double upper_in)
        : lower(lower_in), upper(upper_in)
    {
        if(upper_in < lower_in){
            RVS_ERROR("upper time should not be < lower time");
            lower = -1.0;
            upper = -1.0;
        }
    }

    /** @brief Is the time constraint specified */
    bool IsSpecified() const { return upper > -Constants<double>::Epsilon() && lower > -Constants<double>::Epsilon(); }

    double lower; //< default value is -1 
    double upper; //< default values is -1
};

enum ViapointType
{
    ViapointType_Undef = 0, ///< Unknown viapoint type
    ViapointType_Joint = 1, ///< Joint space viapoint
    ViapointType_JointRegionDiscrete = 2, ///< Discrete Joint space region
    ViapointType_JointRegion = 3, ///< Joint space region continous
    ViapointType_Cart = 4, ///< Cartesian space viapoint
    ViapointType_CartRegionDiscrete = 5, ///< Discrete Cartesian region
    ViapointType_CartRegion = 6, ///<  Cartesian region continuous
};


RVS_CLASS_FORWARD(Viapoint);

RVS_CLASS_FORWARD(JointViapoint);

RVS_CLASS_FORWARD(JointRegionDiscrete);

RVS_CLASS_FORWARD(CartesianViapoint);

RVS_CLASS_FORWARD(CartesianRegionDiscrete)

RVS_CLASS_FORWARD(AxialSymmetric);

template <typename LieGroup>
class RegionViapoint;

template <typename T>
struct region_trait;

template <>
struct region_trait<SE3d>
{
    using Tangent = typename SE3d::Tangent;
    using Discrete = CartesianRegionDiscrete;
    static const ViapointType viapoint_type = ViapointType_CartRegion;
    static const int DoF = 6;
};

template <>
struct region_trait<Rxd>
{
    using Tangent = typename Rxd::Tangent;
    using Discrete = JointRegionDiscrete;
    static const ViapointType viapoint_type = ViapointType_JointRegion;
    static const int DoF = -1;
};


/**
 * @brief Defines a generic way of sending viapoints to planner
 *
 */
class Viapoint : public std::enable_shared_from_this<Viapoint>
{
public:
    Viapoint(const ViapointType type = ViapointType_Undef,
             const TimeConstraint &time_constraint = TimeConstraint(),
             const bool is_critical = false)
        : m_type(type), m_time_constraint(time_constraint),
          m_is_critical(is_critical)
    {
    }

    virtual ~Viapoint() = default;

    ViapointType GetType() const { return m_type; }

    TimeConstraint GetTimeConstraint() const { return m_time_constraint; }

    void SetTimeConstraint(const TimeConstraint &time_constraint)
    {
        m_time_constraint = time_constraint;
    }

    bool IsTimeConstrainted() const { return m_time_constraint.IsSpecified(); }

    bool IsCritical() const { return m_is_critical; }

    void SetIsCritical(const bool is_critical) { m_is_critical = is_critical; }

    bool IsCartesianViapointType() const { return m_type >= ViapointType_Cart; }

    bool IsJointViapointType() const
    {
        return (m_type <= ViapointType_JointRegion
                && m_type >= ViapointType_Joint);
    }

    /**
     * @brief Get the Joint Positions
     *
     * @param kin_solver
     * @param joint_seed
     * @return std::vector<JointVector>
     */
    virtual std::vector<JointVector> GetJointPositions(
        const std::shared_ptr<KinematicsBase> &kin_solver = nullptr,
        const JointVector &joint_seed = JointVector(0),
        bool use_all_ik = true) const = 0;

    /**
     * @brief Get Cartesian Pose
     *
     * @param kin_solver
     * @return std::vector<SE3d>
     */
    virtual std::vector<SE3d> GetCartesianPoses(
        const std::shared_ptr<KinematicsBase> &kin_solver = nullptr) const = 0;

protected:
    ViapointType m_type; ///< Should be set by the derived class for casting
                         ///< Viapoint back to appropriate derived class type

    TimeConstraint
        m_time_constraint; ///< time constraint is used for specifying time
                           ///< stamp of this viapoint

    bool m_is_critical; ///< If false, this point is used as a guide/heuristic
                        ///< rather than a rigid viapoint (Default=true)
};

RVS_CLASS_FORWARD(JointViapoint);
/**
 * @brief Defines a joint space position viapoint for planner
 *
 */
class JointViapoint : public Viapoint
{
public:
    /**
     * @brief Construct a new JointViapoint from JointVector
     *
     * @param joint_positions JointVector space element
     * @param is_critical Is this viapoint is critical, default is true
     */
    JointViapoint(const JointVector &joint_positions,
                  const TimeConstraint &time_constraint = TimeConstraint(),
                  const bool is_critical = true)
        : Viapoint(ViapointType_Joint, time_constraint, is_critical),
          m_joint_positions(joint_positions)
    {
    }

    /**
     * @brief Construct a new JointViapoint from std::vector
     *
     * @param joint_positions std::vector of joint position
     * @param is_critical Is this viapoint is critical, default is true
     */
    JointViapoint(const std::vector<double> &joint_positions,
                  const TimeConstraint &time_constraint = TimeConstraint(),
                  const bool is_critical = true)
        : Viapoint(ViapointType_Joint, time_constraint, is_critical)
    {
        const size_t num_dof = joint_positions.size();
        m_joint_positions.Resize(num_dof);
        for (size_t i = 0; i < num_dof; ++i) {
            m_joint_positions[i] = joint_positions[i];
        }
    }

    virtual std::vector<JointVector> GetJointPositions(
        const std::shared_ptr<KinematicsBase> &kin_solver = nullptr,
        const JointVector &joint_seed = JointVector(0),
        bool use_all_ik = true) const override;

    virtual std::vector<SE3d>
    GetCartesianPoses(const std::shared_ptr<KinematicsBase> &kin_solver =
                          nullptr) const override;

private:
    JointVector m_joint_positions; ///< Configuration space state
};

RVS_CLASS_FORWARD(JointRegionDiscrete);
/**
 * @brief Defines a joint space position viapoint for planner
 *
 */
class JointRegionDiscrete : public Viapoint
{
public:
    /**
     * @brief Construct a new JointRegionDiscrete from JointVector
     *
     * @param joint_positions_vec JointVector space elements
     * @param is_critical Is this viapoint is critical, default is true
     */
    JointRegionDiscrete(
        const std::vector<JointVector> &joint_positions_vec,
        const TimeConstraint &time_constraint = TimeConstraint(),
        const bool is_critical = true)
        : Viapoint(ViapointType_JointRegionDiscrete, time_constraint,
                   is_critical),
          m_joint_positions_vec(joint_positions_vec)
    {
    }

    virtual std::vector<JointVector> GetJointPositions(
        const std::shared_ptr<KinematicsBase> &kin_solver = nullptr,
        const JointVector &joint_seed = JointVector(0),
        bool use_all_ik = true) const override;

    virtual std::vector<SE3d>
    GetCartesianPoses(const std::shared_ptr<KinematicsBase> &kin_solver =
                          nullptr) const override;

private:
    std::vector<JointVector>
        m_joint_positions_vec; ///< Configuration space states
};

RVS_CLASS_FORWARD(CartesianViapoint);
/**
 * @brief Defines a Cartesian space viapoint for planner, can be convert to
 * joint space with a kinsolver
 *
 */
class CartesianViapoint : public Viapoint
{
public:
    /**
     * @brief Construct a new CartesianViapoint
     *
     * @param cartesian_pose SE3d pose
     * @param ref_link reference link name, default is GetWorldName()
     * @param is_critical
     */
    CartesianViapoint(const SE3d &cartesian_pose,
                      const TimeConstraint &time_constraint = TimeConstraint(),
                      const bool is_critical = true)
        : Viapoint(ViapointType_Cart, time_constraint, is_critical),
          m_pose(cartesian_pose)
    {
    }

    const SE3d &GetPose() const { return m_pose; }

    virtual std::vector<JointVector> GetJointPositions(
        const std::shared_ptr<KinematicsBase> &kin_solver = nullptr,
        const JointVector &joint_seed = JointVector(0),
        bool use_all_ik = true) const override;

    virtual std::vector<SE3d>
    GetCartesianPoses(const std::shared_ptr<KinematicsBase> &kin_solver =
                          nullptr) const override;

private:
    SE3d m_pose;
};

RVS_CLASS_FORWARD(CartesianRegionDiscrete);
/**
 * @brief Defines a discrete Catesian region viapoint, contains a list of
 * CatersianPoses
 *
 */
class CartesianRegionDiscrete : public Viapoint
{
public:
    /**
     * @brief Construct a new CartesianRegionDiscrete object
     *
     * @param poses a list of SE3 pose
     * @param ref_link reference link
     * @param is_critical
     */
    CartesianRegionDiscrete(
        const std::vector<SE3d> &poses,
        const TimeConstraint &time_constraint = TimeConstraint(),
        const bool is_critical = true)
        : Viapoint(ViapointType_CartRegionDiscrete, time_constraint,
                   is_critical),
          m_poses(poses)
    {
    }

    virtual std::vector<JointVector> GetJointPositions(
        const std::shared_ptr<KinematicsBase> &kin_solver = nullptr,
        const JointVector &joint_seed = JointVector(0),
        bool use_all_ik = true) const override;

    virtual std::vector<SE3d>
    GetCartesianPoses(const std::shared_ptr<KinematicsBase> &kin_solver =
                          nullptr) const override;

private:
    std::vector<SE3d> m_poses;
};

template <typename LieGroup>
class RegionViapoint : public Region<LieGroup>, public Viapoint
{
    using Tangent = typename LieGroup::Tangent;
    using Discrete = typename region_trait<LieGroup>::Discrete;

    static const ViapointType viapoint_type =
        region_trait<LieGroup>::viapoint_type;

    static const int DoF = region_trait<LieGroup>::DoF;


public:
    /**
     * @brief Construct a new RegionViapoint object
     *
     * @param origin: origin pose of the region
     * @param lower_bound: lower bound of the region defined in the tangent
     * space at origin
     * @param upper_bound: upper bound of the region defined in the tangent
     * space at origin
     * @param is_critical should be treated as a cost or a constraint.
     */
    RegionViapoint(const LieGroup &origin, const Tangent &lower_bound,
                   const Tangent &upper_bound,
                   const std::vector<size_t> &num_samples,
                   const TimeConstraint &time_constraint = TimeConstraint(),
                   const bool is_critical = true)
        : Region<LieGroup>(origin, lower_bound, upper_bound),
          Viapoint(viapoint_type, time_constraint, is_critical),
          m_num_samples(std::move(num_samples))
    {
    }

    /**
     * @brief Construct a new RegionViapoint object
     *
     * @param origin: origin pose of the region
     * @param lower_bound: lower bound of the region defined in the tangent
     * space at origin
     * @param upper_bound: upper bound of the region defined in the tangent
     * space at origin
     * @param is_critical should be treated as a cost or a constraint.
     */
    RegionViapoint(const LieGroup &origin, const Tangent &lower_bound,
                   const Tangent &upper_bound, size_t num_samples = 10,
                   const TimeConstraint &time_constraint = TimeConstraint(),
                   const bool is_critical = true)
        : RegionViapoint(origin, lower_bound, upper_bound,
                         std::vector<size_t>(origin.DoF(), num_samples),
                         time_constraint, is_critical)
    {
    }

    /**
     * @brief Construct a new Cartesian Region object
     *
     * @param origin: origin pose of the region
     * @param is_critical should be treated as a cost or a constraint.
     */
    RegionViapoint(const LieGroup &origin,
                   const TimeConstraint &time_constraint = TimeConstraint(),
                   const bool is_critical = true)
        : RegionViapoint(origin, Tangent::ZeroStatic(), Tangent::ZeroStatic(),
                         std::vector<size_t>(origin.DoF(), 0), time_constraint,
                         is_critical)
    {
    }

    /**
     * @brief Set number of samples on each dof of the tangent space
     * @param num_samples: a std::vector which has size equals to the dof of the
     * tangent space
     */
    RVSReturn SetNumSamples(const std::vector<size_t> &num_samples);

    /**
     * @brief Get the corresponding discretized region
     */
    std::shared_ptr<Discrete> Discretize() const;


    virtual std::vector<SE3d>
    GetCartesianPoses(const std::shared_ptr<KinematicsBase> &kin_solver =
                          nullptr) const override;

    virtual std::vector<JointVector> GetJointPositions(
        const std::shared_ptr<KinematicsBase> &kin_solver = nullptr,
        const JointVector &joint_seed = JointVector(0),
        bool use_all_ik = true) const override;

protected:
    std::vector<size_t> m_num_samples;
};


using CartesianRegion = RegionViapoint<SE3d>;
using JointRegion = RegionViapoint<Rxd>;

RVS_DECLARE_PTR(CartesianRegion, CartesianRegion);
RVS_DECLARE_PTR(JointRegion, JointRegion);

enum AxialSymmetricType
{
    AxialSymmetricType_X = 0, ///< redundancy allowing rotate about the X-axis
    AxialSymmetricType_Y = 1, ///< redundancy allowing rotate about the Y-axis
    AxialSymmetricType_Z = 2, ///< redundancy allowing rotate about the Z-axis
};

RVS_CLASS_FORWARD(AxialSymmetric);
class AxialSymmetric : public CartesianRegion
{
public:
    AxialSymmetric(const SE3d &origin_pose, const size_t num_samples = 10,
                   const AxialSymmetricType type = AxialSymmetricType_Z,
                   const TimeConstraint &time_constraint = TimeConstraint(),
                   const bool is_critical = true);


    AxialSymmetric(const CVec3d &position, const CVec3d &normal,
                   const CVec3d &x_reference = CVec3d(1, 0, 0),
                   const size_t num_samples = 10,
                   const TimeConstraint &time_constraint = TimeConstraint(),
                   const bool is_critical = true);

    /**
     * @brief Get pose by a given theta rotate about the axis
     * @param theta: theta
     * @return generated pose
     */
    SE3d GetPoseOnAxis(const double theta) const
    {
        return GetOrigin() + (theta * m_axis);
    }

    /**
     * @brief Reset the origin pose by a given x_reference direction
     * @param x_direction
     * @return RVSReturn_Success or RVSReturn_Failed
     */
    RVSReturn ResetXDirection(const CVec3d &x_direction);

    /**
     * @brief Get the Position
     */
    CVec3d GetPosition() const { return GetOrigin().Translation(); }


    /**
     * @brief Get the Axis
     */
    CVec3d GetAxis() const;

private:
    bool _Normal2SO3(const CVec3d &normal, const CVec3d &x_direction,
                     SO3d &orientation);
    AxialSymmetricType m_type;
    SE3Tangentd m_axis;
};


/// @}
} // namespace RVS