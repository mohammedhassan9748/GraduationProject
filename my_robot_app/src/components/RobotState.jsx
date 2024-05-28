import React, { Component } from "react";
import { Row, Col } from "react-bootstrap";
import Config from "../scripts/config";
import * as Three from "three";
import "./../styles/RobotState.css"; // Import the CSS file

class RobotState extends Component {
  state = {
    connected: false,
    x: 0,
    y: 0,
    orientation: 0,
    linear_velocity: 0,
    angular_velocity: 0,
    reconnecting: false, // Added state for indicating reconnection attempt
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
      this.setState({ connected: true, reconnecting: false }, this.getRobotState);
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
      reconnecting: true, // Set reconnecting to true during reconnection attempt
    });
  };

  connect = () => {
    const protocol = window.location.protocol === 'https:' ? 'wss://' : 'ws://';
    try {
        this.ros.connect(`${protocol}${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`);
    } catch (error) {
        console.error("Connection problem: ", error);
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
      <div className="robot-state-container">
        {this.state.reconnecting && ( // Display reconnection indicator
          <p className="reconnecting-indicator">Reconnecting...</p>
        )}
        <Row>
          <Col>
            <h4 className="robot-state-header">Position</h4>
            <p className="robot-state-label">x: <span className="robot-state-value">{this.state.x}</span></p>
            <p className="robot-state-label">y: <span className="robot-state-value">{this.state.y}</span></p>
            <p className="robot-state-label">Orientation: <span className="robot-state-value">{this.state.orientation}</span></p>
          </Col>
        </Row>
        <Row>
          <Col>
            <h4 className="robot-state-header">Velocities</h4>
            <p className="robot-state-label">Linear Velocity: <span className="robot-state-value">{this.state.linear_velocity}</span></p>
            <p className="robot-state-label">Angular Velocity: <span className="robot-state-value">{this.state.angular_velocity}</span></p>
          </Col>
        </Row>
      </div>
    );
  }
}

export default RobotState;
