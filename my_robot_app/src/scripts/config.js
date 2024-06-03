const Config = {
    PROTOCOL: 'wss://',
    ROSBRIDGE_SERVER_IP: "156.214.222.92",
    ROSBRIDGE_SERVER_PORT: "9090",
    RECONNECTION_TIMER: 3000,
    CMD_VEL_TOPIC: "/cmd_vel",
    ODOM_TOPIC: "/odom",
    AMCL_TOPIC: "amcl_pose",
    POSE_TOPIC: "/robot_pose",
};

export default Config;