// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <RVS/Environment/Multibody.h>
#include <RVS/Kinematics/KinematicsBase.h>
#include <RVS/Kinematics/Utils.h>
#include <RVS/Trajectory/TrajectoryUtilsEx.h>

#include <RVS/Planner/Core/Types.h>
#include <RVS/Planner/Core/StateConstraint.h>
#include <RVS/Planner/Core/StateCost.h>

#include <iostream>
#include <string>

#define GraspStatusEnum(PREFIX)                                                \
    PREFIX##_NO_IK, PREFIX##_BAD_MANIPULABILITY, PREFIX##_BAD_JOINT_LIMIT,     \
        PREFIX##_COLLISION_EEF, PREFIX##_COLLISION_ROBOT,


namespace RVS
{
/// @addtogroup Planner
/// @{

enum GraspStatus
{
    GRASP_NOT_INIT = 0,
    GRASP_VALID,
    TCP_NORMAL_TILT_ANGLE_INVALID,
    // clang-format off
    GraspStatusEnum(PICK_POSE) 
    GraspStatusEnum(PICK_APPROACH_POSE)
    GraspStatusEnum(PICK_APPROACH_MOTION) 
    GraspStatusEnum(PICK_RETREAT_POSE)
    GraspStatusEnum(PICK_RETREAT_MOTION) 
    GraspStatusEnum(PLACE_POSE)
    GraspStatusEnum(PLACE_APPROACH_POSES)
    GraspStatusEnum(PLACE_APPROACH_MOTION)
    GraspStatusEnum(PLACE_RETREAT_POSES)
    GraspStatusEnum(PLACE_RETREAT_MOTION)
    // clang-format on
};

/**
 * @brief This struct is used for keeping info of each pick pose
 *
 */
struct PickInfo : public std::enable_shared_from_this<PickInfo>
{
    // Used for automatically generating grasp pose from Mesh file of workpiece
    // and eef
    std::shared_ptr<Multibody> object;
    std::shared_ptr<EndEffector> eef;

    SE3d pose_in_obj; // grasp pose defined in workpiece frame
    SE3Tangentd approach_tangent; // (defined in tcp
                                  // frame)approach_tangent.normalized() is
                                  // approach direction, approach_tangent.norm()
                                  // is tcp feed distance
    double linear_approach_step; // resolution in approach straight line
    SE3Tangentd retreat_tangent; // (defined in world
                                 // frame)retreat_tangent.normalized() is
                                 // retreat direction, retreat_tangent.norm() is
                                 // tcp retreat distance
    double center_retreat_percentage; // retreat a certain percentage
                                      // towards the center of pick region
    double linear_retreat_step; // resolution in retreat straight line
};

/**
 * @brief This struct is used for keeping info of each place pose
 *
 */
struct PlaceInfo : public std::enable_shared_from_this<PlaceInfo>
{
    R3d place_position; // place position(x,y,z)
    std::vector<SO3d> place_orientations; // support different place
                                          // orientations in one same position

    std::vector<SE3Tangentd>
        approach_tangents; // (defiened in world frame) multiple approach
                           // lines before final placement
    double linear_approach_step; // resolution in approach straight line
    std::vector<SE3Tangentd> retreat_tangents; // (defiened in world frame)
                                               // multiple retreat lines
    double linear_retreat_step; // resolution in retreat straight line
};

/**
 * @brief This struct is used for keeping grasp plan result of each
 * workpiece_pose, PickInfo, and PlaceInfo(if have)
 *
 * IF one workpiece has 4 PickInfo(grasp pose), and detection proc gets 5
 * workpieces, then grasp plan returns 4 * 5 = 20 GraspMsg, including 1
 * success GraspMsg and 19 useless GraspMsg(failed and other unused)
 *
 */
struct GraspMsg : public std::enable_shared_from_this<GraspMsg>
{
    GraspStatus status{GRASP_NOT_INIT}; // grasp status
    SE3d workpiece_pose; // target workpiece pose

    // ----Pick----
    PickInfo pick_info;
    double pick_score;
    double tcp_normal_tilt_angle; // Maximum tilt angle of tcp feed

    SE3d pick_pose; // workpiece_pose * pick_info.pose_in_obj
    JointVector pick_joint; // ik of pick_pose

    SE3d pick_approach_pose; // pick_pose * pick_info.approach_tangent * -1.0
    JointVector pick_approach_joint; // ik of pick_approach_pose
    std::vector<JointVector>
        pick_approach_waypoints; // straight waypoints from
                                 // pick_approach_joint to pick_joint

    SE3d pick_retreat_pose; // center_retreat_tangent +
                            // pick_info.retreat_tangent + pick_pose
    JointVector pick_retreat_joint; // ik of pick_retreat_pose
    std::vector<JointVector>
        pick_retreat_waypoints; // straight waypoints from pick_joint to
                                // pick_retreat_joint

    // ----Place----
    PlaceInfo place_info;
    double place_score;

    SE3d place_pose; // PlaceInfo.place_pose * pick_info.pose_in_obj
    JointVector place_joint; // ik of place_pose

    std::vector<SE3d>
        place_approach_poses; // place_info.approach_tangents * place_pose
    std::vector<JointVector>
        place_approach_joints; // ik of place_approach_poses
    std::vector<std::vector<JointVector>>
        place_approach_waypoints; // waypoints from place_approach_joints to
                                  // place_joint

    std::vector<SE3d>
        place_retreat_poses; // place_info.retreat_tangents * place_pose
    std::vector<JointVector> place_retreat_joints; // ik of place_retreat_poses
    std::vector<std::vector<JointVector>>
        place_retreat_waypoints; // waypoints from place_joint to
                                 // place_retreat_joints
};

RVS_CLASS_FORWARD(GraspPlanner);
class GraspPlanner : public std::enable_shared_from_this<GraspPlanner>
{
public:
    explicit GraspPlanner(const std::string &name = "GraspPlanner")
        : m_name(std::move(name))
    {
        RVS_TRACE("Consructing GraspPlanner");
        m_configuration = nullptr;
        m_world_to_robot = SE3d(0, 0, 0, 0, 0, 0, 1);
        m_kin_solver = nullptr;
        m_grasp_constraint = nullptr;

        m_workpiece_poses.clear();
        m_workpiece_unique_names.clear();
    }

    virtual ~GraspPlanner() { RVS_TRACE("Destroying GraspPlanner"); }

    struct Configuration : public ConfigurationBase
    {
        RVS_DECLARE_PTR_MEMBER(Configuration);

    public:
        Configuration(
            std::shared_ptr<Environment> env_in,
            const std::vector<std::shared_ptr<Manipulator>> &manipulators_in);

        virtual ~Configuration() override = default;

        /** @brief Check the necessary parameters and set ManipulabilityCost ,
         * JointLimitCost and Collision Constraint */
        virtual bool Setup() override;

        /** @brief Check whether the collision constraint is valid */
        bool IsConstraintValid(const JointVector &state);

        /** @brief Get Manipulability Cost */
        double GetManipulabilityCost(const JointVector &state);

        /** @brief Get JointLimit Cost */
        double GetJointLimitCost(const JointVector &state);

        /** @brief Check if the place plan is activated */
        bool IsPlaceActivated() { return place_activated; }

    public:
        bool safe_mode; // enbale to consider point cloud collision, disable
                        // to make plan faster

        std::string workpiece_name; // The name prefix of the workpiece

        std::vector<PickInfo>
            pick_infos; // each grasp pose has a corresponding PickInfo
        Box pick_region; // pick region
        JointVector pick_region_seed; // pick region seed
        JointVector place_region_seed; // place region seed

        // cost weight used to sort workpiece
        double workpiece_height_weight;
        double workpiece_dist_to_center_weight;
        // constraint threshold used to select workpiece
        double tcp_normal_max_tilt_angle;
        double max_joint_limit_value;
        double max_manipulability_value;

    protected:
        bool place_activated;
        std::shared_ptr<ManipulabilityCost> manipulability_cost;
        std::shared_ptr<JointLimitsCost> joint_limit_cost;
    };

    struct GraspPlannerRequest
    {
        std::vector<SE3d> obj_poses; // candidate poses
        std::shared_ptr<Multibody> point_clouds; // optional

        bool enable_place; // optional
        PlaceInfo place_info; // optional
    };

    struct GraspPlannerResponse
    {
        GraspMsg success_res;
        std::vector<GraspMsg> failed_res;
        std::vector<GraspMsg>
            other_res; // In order to speed up, we find the successful
                       // result and return it, and the remaining unchecked
                       // are classified as other
    };

    bool SetConfiguration(const Configuration::Ptr &configuration);

    RVSReturn IsConfigured() const;

    const Configuration::Ptr &GetConfiguration() const
    {
        return m_configuration;
    }

    static GraspPlanner::GraspPlannerRequest
    CreateRequest(const std::vector<SE3d> &obj_poses,
                  const std::shared_ptr<Multibody> &point_clouds = nullptr,
                  bool enable_place = false,
                  const PlaceInfo &place_info = PlaceInfo());

    bool Solve(const GraspPlannerRequest &request,
               GraspPlannerResponse &response);

private:
    // Module1(Todo): Grasp pose generator
    // Generate grasp pose from std::shared_ptr<Multibody> object and
    // std::shared_ptr<EndEffector> eef;
    // bool _GenerateGraspPose();

    // Module2: Filter
    // Input one workpiece pose and one pick info, give detail info of
    // GraspMsg, just the reachability (IK) and legality(manipulability,
    // joint limit, collision)
    bool _PickFilter(const SE3d &obj_pose, const PickInfo &pick_info,
                     GraspMsg &pick_msg);
    // Input one GraspMsg(pick), give detail info of
    // GraspMsgs(place), just the reachability (IK) and
    // legality(manipulability, joint limit, collision)
    bool _PlaceFilter(const GraspMsg &pick_msg, const PlaceInfo &place_info,
                      std::vector<GraspMsg> &pick_and_place_msgs);

    // Module3: Plan
    // Calculate approach waypoints and retreat waypoints
    bool _PickPlan(GraspMsg &pick_msg);
    bool _PlacePlan(GraspMsg &place_msg);

    /**
     * @brief Sort Workpiece(According to the height of the pose and the
     * distance from the center of the area)
     *
     * @param poses  origin poses
     */
    std::vector<SE3d> _SortWorkpiece(const std::vector<SE3d> &poses);

    /**
     * @brief Add workpieces and pointclouds to Collision Checker
     *
     * @param state_constraint  collision checker constraint
     * @param obj_pose workpieces pose
     * @param point_clouds point clouds
     */
    std::pair<bool, std::vector<std::string>> _AddObsToConstraint(
        const std::shared_ptr<StateConstraint> &state_constraint,
        const std::vector<SE3d> &obj_pose,
        const std::shared_ptr<Multibody> &point_clouds);

    /**
     * @brief Remove workpieces and pointclouds in Collision Checker
     */
    bool _RemoveObsInConstraint(
        const std::shared_ptr<StateConstraint> &state_constraint);

    /**
     * @brief Disable one workpiece's Collision
     *
     * @param state_constraint  collision checker constraint
     * @param obj_pose target workpiece pose
     */
    bool _DisableTargetWorkpieceInConstraint(
        const std::shared_ptr<StateConstraint> &state_constraint,
        const SE3d &obj_pose);

    /**
     * @brief Enable one workpiece's Collision
     *
     * @param state_constraint  collision checker constraint
     * @param obj_pose target workpiece pose
     */
    bool _EnableTargetWorkpieceInConstraint(
        const std::shared_ptr<StateConstraint> &state_constraint,
        const SE3d &obj_pose);

    /**
     * @brief Calculate the tilt angle of the tcp normal(That is, the angle
     * between the line from approach pose to pick pose and the vertical
     * direction)
     *
     * @param tcp_pose  pick pose
     * @param tcp_approach_pose pick approach pose
     */
    double _ComputeTCPNormalTiltAngle(const SE3d &tcp_pose,
                                      const SE3d &tcp_approach_pose);

    /**
     * @brief Generate Pick Retreat Pose(Custom retreat direction + retreat
     * towards the center of the area)
     *
     * @param tcp_pose  pick pose
     * @param retreat_tangent retreat direction and retreat distance
     * @param region pick region
     * @param center_retreat_percentage Percentage of retreat to the
     * regional center
     * @param tcp_retreat_pose Calculated retreat pose
     */
    void _GeneratePickRetreatPose(const SE3d &tcp_pose,
                                  const SE3Tangentd &retreat_tangent,
                                  const Box &region,
                                  const double center_retreat_percentage,
                                  SE3d &tcp_retreat_pose);

    /**
     * @brief Generate Place Retreat Pose
     *
     * @param tcp_pose  place pose
     * @param retreat_tangents retreat directions and retreat distances
     * @param tcp_retreat_poses Calculated retreat poses
     */
    void
    _GeneratePlaceRetreatPoses(const SE3d &tcp_pose,
                               const std::vector<SE3Tangentd> &retreat_tangents,
                               std::vector<SE3d> &tcp_retreat_poses);

private:
    std::string m_name;
    Configuration::Ptr m_configuration;

    SE3d m_world_to_robot;
    std::shared_ptr<KinematicsBase> m_kin_solver;

    std::shared_ptr<CompoundStateConstraint> m_grasp_constraint;

    std::vector<SE3d> m_workpiece_poses;
    std::vector<std::string> m_workpiece_unique_names;
};

} // namespace RVS