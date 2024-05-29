const Config = {
    PROTOCOL: 'ws://',
    ROSBRIDGE_SERVER_IP: "192.168.100.216",
    ROSBRIDGE_SERVER_PORT: "9090",
    RECONNECTION_TIMER: 3000,
    CMD_VEL_TOPIC: "/cmd_vel",
    ODOM_TOPIC: "/odom",
    AMCL_TOPIC: "amcl_pose",
    POSE_TOPIC: "/robot_pose",
};

export default Config;