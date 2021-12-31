// Copyright (c) RVBUST, Inc - All rights reserved.
#include <RVS/Controller/GenericRobotController.h>
#include <RVS/RobotMessage/Socket.h>

namespace RVS
{

/**
 * @brief ControllerServer is used to establish a TCP server, the server could
 * accept request from client about controlling robot, getting robot states and
 * so on. This could be an example server (running in lower robot controller)
 * to be used with SimpleController.
 *
 */
class ControllerServer
{
public:
    ControllerServer(ControllerBasePtr controller);

    ~ControllerServer();

    /**
     * @brief Start TCP server. At most one client could be conneted at the same
     * time. And it is running as a single thread.
     *
     * @param ip IP address
     * @param port listening port
     * @return true
     * @return false
     */
    bool StartServer(const std::string &ip = "0.0.0.0", unsigned port = 9999);

    /**
     * @brief Stop TCP server
     *
     * @return true
     * @return false
     */
    bool StopServer();

    ControllerBasePtr GetController() { return m_controller.lock(); }
    KinematicsBasePtr GetKinSolver() { return m_kin_solver.lock(); }

private:
    void _RunServerThread(const std::string &ip, unsigned port);
    void _HandleMotionCtrlMsg(const MessageMotionControl &msg);
    void _HandleJointFullPtMsg(const MessageJointFullPt &msg);
    void _HandleCartesianFullPtMsg(const MessageCartesianFullPt &msg);
    void _HandleSetIOMsg(const MessageSetIO &msg);
    void _HandleReadIOMsg(const MessageReadIO &msg);

    ControllerBaseWeakPtr m_controller;
    KinematicsBaseWeakPtr m_kin_solver;
    bool m_keep_server_alive{false};
    bool m_is_server_thread_running{false};
    TCPClientPtr m_client;
    TCPServerPtr m_server;

    std::list<Pose> m_cart_waypoints_buffer;
    std::list<JointVector> m_joint_waypoints_buffer;
};

RVS_DECLARE_PTR(ControllerServer, ControllerServer)
} // namespace RVS