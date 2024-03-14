import React, { Component } from "react";
import { Joystick } from "react-joystick-component";
import Config from "../scripts/config";

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
      console.log("connection established in Teleoperation Component!");
      this.setState({ ros, connected: true });
    });

    ros.on("close", () => {
      console.log("connection is closed!");
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
      ros.connect(`ws://${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`);
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
    console.log("handle move");
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
  };

  handleStop = (event) => {
    console.log("handle stop");
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
  };

  render() {
    return (
      <div>
        <Joystick
          size={100}
          baseColor="#EEEEEE"
          stickColor="#BBBBBB"
          move={this.handleMove}
          stop={this.handleStop}
        />
      </div>
    );
  }
}

export default Teleop;
