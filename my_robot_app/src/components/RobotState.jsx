import React, { Component } from "react";
import { Row, Col } from "react-bootstrap";
import Config from "../scripts/config";
import * as Three from "three";

class RobotState extends Component {
  state = {
    connected: false,
    x: 0,
    y: 0,
    orientation: 0,
    linear_velocity: 0,
    angular_velocity: 0,
  };

  ros = new window.ROSLIB.Ros();

  componentDidMount() {
    this.initConnection();
  }

  componentWillUnmount() {
    if (this.ros) {
      this.ros.close();
    }
  }

  initConnection = () => {
    this.ros.on("connection", () => {
      console.log("Connection established in RobotState Component!");
      this.setState({ connected: true }, this.getRobotState);
    });
  
    this.ros.on("close", () => {
      console.log("Connection is closed!");
      this.resetState();
      this.scheduleReconnect();
    });
  
    this.ros.on("error", (error) => {
      console.error("Connection problem:", error);
      this.resetState();
    });
  
    this.connect();
  };
  
  resetState = () => {
    this.setState({
      connected: false,
      x: 0,
      y: 0,
      orientation: 0,
      linear_velocity: 0,
      angular_velocity: 0,
    });
  };

  connect = () => {
    try {
      this.ros.connect(`ws://${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`);
    } catch (error) {
      console.error("Connection problem:", error);
    }
  };

  scheduleReconnect = () => {
    setTimeout(() => {
      this.connect();
    }, Config.RECONNECTION_TIMER);
  };

  getRobotState = () => {
    const poseSubscriber = new window.ROSLIB.Topic({
      ros: this.ros,
      name: Config.AMCL_TOPIC,
      messageType: "geometry_msgs/PoseWithCovarianceStamped",
    });

    poseSubscriber.subscribe((message) => {
      this.setState({
        x: message.pose.pose.position.x.toFixed(2),
        y: message.pose.pose.position.y.toFixed(2),
        orientation: this.getOrientationFromQuaternion(message.pose.pose.orientation).toFixed(2),
      });
    });

    const velocitySubscriber = new window.ROSLIB.Topic({
      ros: this.ros,
      name: Config.ODOM_TOPIC,
      messageType: "nav_msgs/Odometry",
    });

    velocitySubscriber.subscribe((message) => {
      this.setState({
        linear_velocity: message.twist.twist.linear.x.toFixed(2),
        angular_velocity: message.twist.twist.angular.z.toFixed(2),
      });
    });
  };

  getOrientationFromQuaternion = (ros_orientation_quaternion) => {
    const q = new Three.Quaternion(
      ros_orientation_quaternion.x,
      ros_orientation_quaternion.y,
      ros_orientation_quaternion.z,
      ros_orientation_quaternion.w
    );
    const RPY = new Three.Euler().setFromQuaternion(q);
    return RPY.z * (180 / Math.PI);
  };

render() {
  return (
    <div>
      <Row>
        <Col>
          <h4 className="mt-4">Position</h4>
          <p className="mt-0">x: {this.state.x}</p>
          <p className="mt-0">y: {this.state.y}</p>
          <p className="mt-0">Orientation: {this.state.orientation}</p>
        </Col>
      </Row>
      <Row>
        <Col>
          <h4 className="mt-4">Velocities</h4>
          <p className="mt-0">
            Linear Velocity: {this.state.linear_velocity}
          </p>
          <p className="mt-0">
            Angular Velocity: {this.state.angular_velocity}
          </p>
        </Col>
      </Row>
    </div>
  );
}
}

export default RobotState;