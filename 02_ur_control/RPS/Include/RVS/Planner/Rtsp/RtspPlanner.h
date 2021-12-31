// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Common/LoggerUtils.h>
#include <RVS/Planner/Core/MotionPlannerBase.h>
#include <RVS/Planner/Core/Types.h>
#include <RVS/Planner/Rtsp/RtspGTravelingSalesmenProblem.h>
#include <RVS/Planner/Rtsp/RtspGraphSearch.h>
#include <RVS/Planner/Rtsp/RtspPathPlanner.h>

namespace RVS
{
/// @addgroup Planner
/// @{

class RTSPPlanner : public MotionPlannerBase
{
public:
    /// @todo: struct Configuration : public ConfigurationBase

    struct Configuration : public std::enable_shared_from_this<Configuration>
    {
    public:
        RVS_DECLARE_PTR_MEMBER(Configuration);

        Configuration()
            : m_gtsp_config(nullptr), m_gs_config(nullptr), m_pp_config(nullptr)
        {
            RVS_INFO("Initialize RTSP configuration parameters...");
        }

        Configuration(std::shared_ptr<GTSPConfig> gtsp_config,
                      std::shared_ptr<GraphSearchConfig> gs_config,
                      std::shared_ptr<PathPlannerBase> pp_config)
            : m_gtsp_config(gtsp_config), m_gs_config(gs_config),
              m_pp_config(pp_config)
        {
            RVS_INFO("Initialize RTSP configuration parameters...");
        }

        // destruction function
        virtual ~Configuration() = default;

        std::shared_ptr<GTSPConfig> m_gtsp_config; // dependent on GTSP solvers

        std::shared_ptr<GraphSearchConfig>
            m_gs_config; // dependent on graph search algorithms

        std::shared_ptr<PathPlannerBase>
            m_pp_config; // dependent on motion planning solvers
    };

    explicit RTSPPlanner(const std::string &name = "RTSP_planner")
        : MotionPlannerBase(name), m_rtsp_config(nullptr)
    {
    }

    /**
     * @brief Initialize the related parameters with RTSP planner
     *
     * @param config_params The configuration parameters which are intimately
     * linked with planning in task and config space of robot arm
     *
     */
    bool SetConfiguration(const Configuration::Ptr &configuration)
    {
        m_rtsp_config = configuration;
        return true;
    }

    /**
     * @brief Get the initialized configuration parameters
     *
     * @return const Config::Ptr&
     */
    const Configuration::Ptr &GetConfiguration() const { return m_rtsp_config; }


    /**
     * @brief To set weights which is required by calculating the "distance" of
     * two poses
     *
     * @param weights The weight value of each component of target pose
     *
     *
     */
    void SetPoseWeights(const CVecXd &weights);

    /**
     * @brief Get the target pose weights
     *
     * @return CVecXd The weight value of each component of target pose
     *
     */
    CVecXd GetPoseWeights() const;

    /**
     * @brief Set the sample parameters which is aimed at generating required
     * nodes when input graph nodes are targets pose instead of joint
     * configurations
     *
     * @param manip The manipulator that will move following given target poses
     * @param rot_axis The reference rotation axis in order to generate new
     * target poses, i.e., it can take X,Y or Z axis
     * @param sample_int The sampling interval in order to generate new target
     * pose around given reference rotation axis, the range of value is (0,2*pi]
     *
     */
    void SetSampleParams(std::shared_ptr<Manipulator> manip,
                         RotationAxis rot_axis, double sample_int);

    /**
     * @brief Set the reference start target which is the joint configuration
     * that manipulator arrives at it first.
     *
     * @param init_joint_config The input initial joint configuration
     *
     */
    void SetReferStartTargetAtCS(const JointVector &init_joint_config);

    /**
     * @brief Get the reference start target
     *
     * @return const JointVector&  The current reference start target
     *
     */
    JointVector GetReferStartTargetAtCS() const;

    /**
     * @brief To keep the idential joint configuraiton at the start and end of
     * motion of manipulator, which exclusively adjusts to the result of
     * resampling the nodes which represent target poses
     *
     * @param is_same If it is true, the joint configuration will be equal.
     * Otherwise, it may be different from each other
     *
     */
    void EnableConsistency(bool is_same);

    /**
     * @brief check whether the rtsp planner has been initialized properly or
     * not
     *
     * @return RVSReturn  It will return RVSReturn_Success if configured,
     * otherwise, RVSReturn_NotInitialized will be returned
     */
    virtual RVSReturn IsConfigured() const override;
    /**
     * @brief Solve the Robotics Task Sequencing Problem with three separated
     * steps
     * @note: For more details, please refer to https://arxiv.org/abs/1709.09343
     * (RoboTSP - A Fast Solutioon to the Robotic Task Sequencing Problem)
     *
     * @param request The multiple goal poses at task space (it may be TSP
     * (Travelling Salesman Problem) or GTSP)
     * @param response The optimized joint configuration sequences (i.e, joint
     * motion trajectories) accorrding to given optimized criteria
     * @param verbose The related information when solving RTSP
     * @return true Succeed to solve the RTSP
     * @return false Fail to do it
     */
    virtual bool Solve(const MotionPlannerRequest &request,
                       MotionPlannerResponse &response,
                       const bool verbose = false) override;

    /**
     * @brief Get the minimal cost path at task space
     *
     * @return const std::vector<SE3d>& The minimal cost path generated by
     * solving GTSP
     */
    const std::vector<SE3d> &GetPathAtTaskSpace() const;

    /**
     * @brief Get the shorest path at configuration space
     *
     * @return std::vector<JointVector> The shortest path yielded by searching
     * graph (i.e. A* and Dijkstra)
     */
    const std::vector<JointVector> &GetPathAtConfigSpace() const;

    /**
     * @brief To store the minimal cost path into the data file after solving
     * RTSP problem
     *
     * @param file_path  The path name of data file that one would like to save
     * @return bool It will return true if succeeding in saving trajectory into
     * file. The false will be returned, othterwise.
     */
    bool SaveTrajToFile(const std::string &file_path);

    // At present, the func Terminate() and Clear() don't perform any terminate
    // and clear task
    virtual bool Terminate() override { return true; }
    virtual void Clear() override {}

private:
    Configuration::Ptr m_rtsp_config;
    std::vector<SE3d>
        m_rtsp_path_from_gtsp; ///< the path generated by solving GTSP
    std::vector<JointVector>
        m_rtsp_path_from_gs; ///< the path yielded by searching graph
    std::vector<JointVector>
        m_rtsp_path_from_pp; ///< the trajectory obtained by RTSP planner
};

/// @}
} // namespace RVS