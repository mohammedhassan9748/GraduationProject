import React, { Component } from "react";
import Config from "../scripts/config";

class Map extends Component {
  constructor(props) {
    super(props);
    this.state = {
      ros: null,
      connected: false,
      viewerInitialized: false,
    };
    this.viewer = null; // Hold the viewer in the component instance
  }

  componentDidMount() {
    this.initConnection();
  }

  componentWillUnmount() {
    // Clean up ROS connection
    if (this.state.ros) {
      this.state.ros.close();
    }
    // Clean up viewer
    if (this.viewer) {
      this.viewer.removeAllChildren();
      this.viewer = null;
    }
  }

  initConnection = () => {
    // Initialize ROS connection
    const ros = new window.ROSLIB.Ros({
      url: `ws://${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`
    });

    // Define ROS event listeners
    ros.on('connection', () => {
      console.log('Connected to websocket server.');
      this.setState({ ros, connected: true }, this.createViewer);
    });

    ros.on('close', () => {
      console.log('Connection to websocket server closed.');
      this.setState({ connected: false, viewerInitialized: false });
      // Attempt to reconnect
      setTimeout(this.initConnection, Config.RECONNECTION_TIMER);
    });

    ros.on('error', (error) => {
      console.error('Connection problem:', error);
      this.setState({ connected: false });
    });

    // Attempt to connect to ROS
    try {
      ros.connect(`ws://${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`);
    } catch (error) {
      console.log('Connection problem:', error);
    }
  };

  createViewer = () => {
    if (this.state.connected && !this.viewer) {
      this.viewer = new window.ROS2D.Viewer({
        divID: 'nav_div',
        width: 640,
        height: 480,
      });

      this.initializeMap();
    }
  };

  initializeMap = () => {
    if (!this.state.viewerInitialized) {
      new window.NAV2D.OccupancyGridClientNav({
        ros: this.state.ros,
        rootObject: this.viewer.scene,
        viewer: this.viewer,
        serverName: '/move_base',
        withOrientation: true,
      });

      this.setState({ viewerInitialized: true });
    }
  };

  render() {
    return (
      <div>
        <div id="nav_div" style={{ width: '640px', height: '480px' }} />
      </div>
    );
  }
}

export default Map;
