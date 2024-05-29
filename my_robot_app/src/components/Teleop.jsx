import React, { Component } from "react";
import { Joystick } from "react-joystick-component";
import Config from "../scripts/config";
import "./../styles/Teleop.css"; // Import the CSS file

class Teleop extends Component {
  state = {
    ros: null,
    connected: false,
  };

  componentDidMount() {
    this.initConnection();
  }

  componentWillUnmount() {
    if (this.state.ros) {
      this.state.ros.close();
    }
  }

  initConnection() {
    const ros = new window.ROSLIB.Ros();

    ros.on("connection", () => {
      console.log("Connection established in Teleoperation Component!");
      this.setState({ ros, connected: true });
    });

    ros.on("close", () => {
      console.log("Connection is closed!");
      this.setState({ connected: false }, this.scheduleReconnect);
    });

    ros.on("error", (error) => {
      console.error("Connection problem: ", error);
      this.setState({ connected: false });
    });

    this.connect(ros);
  }

  connect(ros) {
    try {
      ros.connect(`${Config.PROTOCOL}${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`);
    } catch (error) {
      console.error("Connection problem: ", error);
    }
  }

  scheduleReconnect = () => {
    setTimeout(() => {
      if (this.state.ros) {
        this.connect(this.state.ros);
      }
    }, Config.RECONNECTION_TIMER);
  };

  handleMove = (event) => {
    if (this.state.ros && this.state.connected) {
      const cmdVel = new window.ROSLIB.Topic({
        ros: this.state.ros,
        name: Config.CMD_VEL_TOPIC,
        messageType: "geometry_msgs/Twist",
      });

      const twist = new window.ROSLIB.Message({
        linear: {
          x: event.y,
          y: 0,
          z: 0,
        },
        angular: {
          x: 0,
          y: 0,
          z: -event.x,
        },
      });

      cmdVel.publish(twist);
    } else {
      console.log("ROS connection is not established.");
    }
  };

  handleStop = () => {
    if (this.state.ros && this.state.connected) {
      const cmdVel = new window.ROSLIB.Topic({
        ros: this.state.ros,
        name: Config.CMD_VEL_TOPIC,
        messageType: "geometry_msgs/Twist",
      });

      const twist = new window.ROSLIB.Message({
        linear: {
          x: 0,
          y: 0,
          z: 0,
        },
        angular: {
          x: 0,
          y: 0,
          z: 0,
        },
      });

      cmdVel.publish(twist);
    } else {
      console.log("ROS connection is not established.");
    }
  };

  render() {
    const { connected } = this.state;

    return (
      <div className="teleop-container">
        <div className={`status ${connected ? "connected" : "disconnected"}`}>
          {connected ? "Connected" : "Disconnected"}
        </div>
        <div className="joystick-container">
          <Joystick
            size={100}
            baseColor="#EEEEEE"
            stickColor="#BBBBBB"
            move={this.handleMove}
            stop={this.handleStop}
          />
        </div>
      </div>
    );
  }
}

export default Teleop;
