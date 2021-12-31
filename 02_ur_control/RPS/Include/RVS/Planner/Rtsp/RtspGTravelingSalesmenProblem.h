// Copyright (c) RVBUST, Inc - All rights reserved
#pragma once

#include <filesystem>
#include <boost/algorithm/string/predicate.hpp>
#include <RVS/Common/Logger.h>
#include <RVS/Planner/Core/Types.h>

namespace RVS
{
/// @addgroup Planner
/// @{


enum GTSPType
{
    GTSPType_Undef = 0,
    GTSPType_Gtsp = 1, ///< symmetric instances of GTSP
    GTSPType_AGtsp = 2 ///< asymmetric instances of GTSP
};

enum GtspSolverType
{
    GtspSolverType_Undef = 0, ///< unknown Gtsp solver type
    GtspSolverType_Glkh = 1, ///< solver GLKH-1.0
};

struct GTSPConfig
{
    /**
     * @brief Construct a new GTSPConfig object
     *
     * @param solver_name The name of GTSP solver
     * @param solver_type The type of solver to solve GTSP
     *
     */
    GTSPConfig(const std::string &solver_name, GtspSolverType solver_type);

    // destruction function
    virtual ~GTSPConfig(){};

    /**
     * @brief To synchronize latest parameters into solver's configuration
     *
     */
    void UpdateGTSPConfig();

    /**
     * @brief To get the solver name of solving GTSP
     *
     * @return std::string  The name of current GTSP solver
     *
     */
    const std::string &GetGTSPSolverName() const { return m_gtsp_solver_name; }

    /**
     * @brief To get the type of GTSP solver
     *
     * @return GtspSolverType  The enumeration type of GTSP solver. The GLKH
     * solver is adopted at present
     *
     */
    GtspSolverType GetGTSPSolverType() const { return m_solver_type; }

    /**
     * @brief To determine whether the GTSP solver has been well configured
     *
     * @return RVSReturn The result (Succeeded or Uninitialized)
     *
     */
    RVSReturn IsConfigured() { return m_is_configured; }

    /**
     * @brief To solve the GTSP problem by specifying the solver
     *
     * @return true  The result that given solver succeeds in solving the
     * problem
     * @return false The result that it fails to do it
     *
     */
    virtual bool SolveGTSP() = 0;

    /**
     * @brief Get the minimal cost path (ordered target poses)
     *
     * @return std::vector<SE3d>  The new ordered target poses that minimizes
     * the cost of path (trajectory)
     *
     */
    virtual const std::vector<SE3d> &GetMinimalCostPath() const = 0;

    /**
     * @brief Get the tour index with respect to original input targets
     *
     * @return std::vector<size_t> The tour index yielded by GTSP solver
     *
     */
    virtual const std::vector<size_t> &GetTourIndex() const = 0;

    std::string m_gtsp_solver_name; ///< the name of solver to solve GTSP
    GtspSolverType m_solver_type; ///< gtsp solver type
    RVSReturn m_is_configured; ///< determine whether all parameters has been
                               ///< initialized

protected:
    // To initialize the related parameters with some GTSP solver
    virtual bool _InitializeParameters() = 0;
};

struct GLKHProblemDescription;

struct GLKHConfig : public GTSPConfig
{
    /**
     * @brief Construct a new GLKHConfig object
     *
     * @param problem_des  The glkh problem description (please refer to TSPLIB
     * for its format). It is a standard parameter file that holds the details
     * of GTSP problem
     *
     */
    GLKHConfig(GLKHProblemDescription &problem_des);

    // destruction function
    virtual ~GLKHConfig() {}

    /**
     * @brief To update (set) raw targets
     *
     * @param new_targets The goal targets that one wants to find a minimized
     * cost path from them
     *
     */
    void UpdateRawTargets(const std::vector<ViapointPtr> &new_targets);

    /**
     * @brief Get the raw targets
     *
     * @return std::vector<ViapointPtr> The targets that the user inputs
     *
     */
    const std::vector<ViapointPtr> &GetRawTargets() const;

    /**
     * @brief To get the arguments related to GLKH solver
     *
     * @return const std::string The arguments of GLKH solver whose format is
     * similar to TSPLIB
     *
     */
    const std::string &GetGLKHParams();

    /**
     * @brief To get the GTSP problem description formatted by GTSPLIB
     *
     * @return GLKHProblemDescription&  The reference of GTSP problem
     * description
     *
     */
    GLKHProblemDescription &GetGLKHProblemDes() { return m_glkh_problem_des; }

    /**
     * @brief To solve the GTSP problem by GLKH solver
     * @note  See struct GTSPConfig for the meaning of parameters
     *
     */
    virtual bool SolveGTSP() override;

    /**
     * @brief Get the minimal cost path (ordered target poses)
     * @note  See struct GTSPConfig for the meaning of parameters
     *
     */
    virtual const std::vector<SE3d> &GetMinimalCostPath() const override;

    /**
     * @brief Get the tour index with respect to original input targets
     * @note  See struct GTSPConfig for the meaning of parameters
     *
     */
    virtual const std::vector<size_t> &GetTourIndex() const override;

    size_t m_ascent_candidates; ///< The number of candidates to be associated
                                ///< with each node during the ascent
    size_t m_initial_period; ///< The length of the first period in the ascent
    size_t m_max_candidates; ///< The maximum number of candidate edges to be
                             ///< associated with each node
    size_t m_max_trials; ///< The maximum number of trials in each run
    std::string m_output_tour_file; ///< The name of a file where the best tour
                                    ///< is to be written
    std::string m_pi_file; ///< The name of a file to which penalties (Pi-values
                           ///< determined by the ascent) are to be written
    size_t m_population_size; ///< The maximum size of the population in LKH's
                              ///< genetic algorithm
    size_t m_precision; ///< The internal precision in the representation of
                        ///< transformed distances (dij = precision * cij + pii
                        ///< + pij)
    size_t m_runs; ///< The total number of runs
    size_t m_seed; ///< The initial seed for random number generation
    int m_trace_level; ///< The level of detail of the output given during
                       ///< the solution process
private:
    // Get file stream from glkh parameter
    FILE *_GetGLKHParameters();
    GLKHProblemDescription
        &m_glkh_problem_des; ///< the description of GTSP problem formatted by
                             ///< GTSPLIB
    std::string m_glkh_params; ///< The related parameters of GLKH solver

protected:
    // To initialize the related parameters with GLKH solver
    virtual bool _InitializeParameters() override;
};


struct GLKHProblemDescription
{
    friend struct GLKHConfig;
    using Ptr = std::shared_ptr<GLKHProblemDescription>;
    using TernaryOperation =
        std::function<double(const SE3d &, const SE3d &, const CVecXd &)>;
    /**
     * @brief Construct a new GLKHProblemDescription object
     *
     * @param dof The degree of freedom of given manipulator
     * @param prob_type The type of GTSP problem (GTSP or AGTSP)
     *
     */
    GLKHProblemDescription(size_t dof = 6, GTSPType prob_type = GTSPType_Gtsp);

    /**
     * @brief Construct a new GLKHProblemDescription object
     *
     * @param goals The targets that will be sorted by GTSP solver
     * @param dof The degree of freedom of given manipulator
     * @param prob_type The type of GTSP problem (GTSP or AGTSP)
     *
     */
    GLKHProblemDescription(const std::vector<ViapointPtr> &goals,
                           size_t dof = 6, GTSPType prob_type = GTSPType_Gtsp);
    // destruction function
    ~GLKHProblemDescription(){};

    /**
     * @brief To initialize the GTSP problem described by GTSPLIB format
     *
     * @return true The result if it succeeds in initializing it
     * @return false The result if it fails to do it
     *
     */
    bool InitializeGLKHGoals();

    /**
     * @brief To Get the initialized GTSP problem formatted by GTSPLIB
     *
     * @return const std::string& The description details of GLKH problem
     *
     */
    const std::string &GetGTSPProblem();

    /**
     * @brief Set the raw targets that will be sorted by GLKH solver
     *
     * @param new_targets  The targets (poses at the task space of manipulator)
     * that the user inputs
     *
     */
    void SetRawTargets(const std::vector<ViapointPtr> &new_targets);

    /**
     * @brief Get the raw targets that the user inputs
     *
     * @return std::vector<ViapointPtr> The initial targets (poses at the
     * task space of manipulator)
     *
     */
    const std::vector<ViapointPtr> &GetRawTargets() const { return m_goals; }

    /**
     * @brief To set weights which is required by calculating the "distance" of
     * two poses
     *
     * @param weights The weight value of each component of target pose
     *
     */
    void SetPoseWeights(const CVecXd &weights);

    /**
     * @brief Get the target pose weights
     *
     * @return const CVecXd& The weight value of each component of target pose
     *
     */
    const CVecXd &GetPoseWeights() const { return m_pose_weights; }

    /**
     * @brief To update (set) metric function that computes the distance between
     * two poses
     *
     * @param to  The callable expression whose signature is required to include
     * three parameters
     *
     */
    void UpdateMetricFunc(const TernaryOperation &to) { m_pose_dist_func = to; }

    std::vector<ViapointPtr>
        m_goals; ///< the raw targets (poses at the task space of manipulator)
    std::vector<SE3d> m_targets_at_ts; ///< all targets in task space
    std::vector<size_t> m_tour_index; ///< The target indices of a tour
                                      ///< corresponding to task space
    std::vector<SE3d> m_minimal_cost_path; ///< poses at the minimal cost path
    size_t m_dof; ///< the active freedom of degree of robot arm'
    CVecXd m_pose_weights; ///< the weights with the each component of pose
                           ///< (such as position(x,y,z), orientation angle
                           ///< (alpha, beta, gamma))
    TernaryOperation
        m_pose_dist_func; ///< the callable expression that is used to quantify
                          ///< the distance between two poses

    GTSPType m_prob_type; ///< the type of gtsp (GTSP or AGTSP)
    std::string m_comments; ///< a concise descritption about the problem
    size_t m_dim; ///< the total number of targets
    size_t m_gtsp_sets; ///< the number of regions in GTSP
    std::string m_edge_weight_type; ///< the encoded type of cost or weight of
                                    ///< any two adjacent edges
    std::string m_edge_weight_format; ///< the edge weights format corresponding
                                      ///< to the aforemetioned encoded type
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>
        m_edge_weight_section; ///< the edge weights data according to the above
                               ///< format
    std::vector<std::vector<size_t>>
        m_gtsp_set_section; ///< the specific index info at each region for
                            ///< all targets
private:
    // Generate the Description of GTSP problems
    bool _GenerateGTSPProblem();
    // To yield file stream with GTSP problem formatted by GTSPLIB
    FILE *_GetGTSPProblem();
    std::string m_glkh_problem; ///< The description of GTSP problem
};

/// @}
} // namespace RVS