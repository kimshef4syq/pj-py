// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <math.h>
#include <RVS/Common/Utils.h>
#include <RVS/Common/Types.h>
#include <RVS/LieGroup/SE3.h>
#include <RVS/LieGroup/SO3.h>

namespace RVS
{
/// @addgroup Kinematics
/// @{

enum TCPCalibrationType
{
    TCPCalibrationType_Position = 0, ///< Calibration tool position
    TCPCalibrationType_Position_Z =
        1, ///< Calibration tool position and z-axis direction
    TCPCalibrationType_Position_ZX =
        2 ///< Calibration tool position and orientation
};

/**
 * @brief Use for manipulation TCP calibration
 *
 */
class TCPCalibration
{
public:
    /**
     * @brief Construct a TCP calibration slover
     */
    TCPCalibration();

    ///@brief clean up the TCPCalibration object
    ~TCPCalibration() { RVS_TRACE("Destructing TCPCalibration!"); }


    /**
     * @brief  Set the method of use for TCP calibration
     * @param type[in] the TCP calibration type
     */
    void SetMethod(const TCPCalibrationType type);

    /**
     * @brief Load the meausure result form the vector
     *
     * @param poses[in] the vector that save the meausure result
     * @return RVSReturn read file result
     */
    RVSReturn LoadData(const std::vector<SE3d> &poses);

    /**
     * @brief Get the Calibration result
     *
     * @param tcp_pose[out] The position and orientation of the end-effector
     * @return RVSReturn
     */
    RVSReturn Solve(SE3d &tcp_pose);

private:
    /**
     * @brief Divide the result to m_rotation_list and m_translation_list
     * according the method
     */
    RVSReturn _DivideDatas();

    /**
     * @brief Get the rotation about the end effector
     * @param position[in] the TCP position
     * @param rotation[out] the TCP rotation
     */
    RVSReturn _GetRotation(const std::vector<double> &position, SO3d &rotation);

private:
    TCPCalibrationType m_type;
    std::vector<SE3d> m_poses; // Meausure result
    std::vector<SE3d>
        m_rotation_list; // A part of meausure result use to calculate rotation
    std::vector<SE3d> m_translation_list; // A part of meausure result use to
                                          // calculate position
};

/// @}
} // namespace RVS