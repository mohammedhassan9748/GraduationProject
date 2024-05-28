import React, { Component } from "react";
import Config from "../scripts/config";

class Map extends Component {
  constructor(props) {
    super(props);
    this.state = {
      ros: null,
      connected: false,
      viewerInitialized: false,
      navMessage: 'Waiting for navigation status...',
    };
    this.viewer = null; // Hold the viewer in the component instance
    this.globalPathLayer = null;
    this.localPathLayer = null; // To hold the local path layer
    this.robotMarker = null; // To hold the robot marker
  }

  componentDidMount() {
    this.initConnection();
  }

  componentWillUnmount() {
    if (this.state.ros) {
      this.state.ros.close();
    }
    this.viewer = null;
  }

  initConnection = () => {
    const ros = new window.ROSLIB.Ros({
      url: `ws://${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`
    });

    ros.on('connection', () => {
      console.log('Connected to websocket server.');
      this.setState({ ros, connected: true }, () => {
        this.createViewer();
        this.subscribeToNavStatus();
        this.subscribeToPaths();
        this.subscribeToRobotPose();
      });
    });

    ros.on('close', () => {
      console.log('Connection to websocket server closed.');
      this.setState({ connected: false, viewerInitialized: false });
      setTimeout(this.initConnection, Config.RECONNECTION_TIMER);
    });

    ros.on('error', (error) => {
      console.error('Connection problem:', error);
      this.setState({ connected: false });
    });

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
        width: 1080,
        height: 720,
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

      this.globalPathLayer = new window.ROS2D.PathShape({
        ros: this.state.ros,
        strokeSize: 0.05,
        strokeColor: 'rgba(255, 0, 0, 0.66)',
      });

      this.localPathLayer = new window.ROS2D.PathShape({
        ros: this.state.ros,
        strokeSize: 0.05,
        strokeColor: 'rgba(0, 0, 255, 0.66)', // Blue color for local path
      });

      this.robotMarker = new window.ROS2D.ArrowShape({
        size: 0.3,
        strokeSize: 0.08,
        fillColor: 'rgba(0, 255, 0, 0.9)',
      });

      this.viewer.scene.addChild(this.globalPathLayer);
      this.viewer.scene.addChild(this.localPathLayer);
      this.viewer.scene.addChild(this.robotMarker);

      this.setState({ viewerInitialized: true });
    }
  };

  subscribeToNavStatus = () => {
    const { ros } = this.state;
    if (!ros) {
      console.error('ROS connection not established');
      return;
    }

    const navStatusListener = new window.ROSLIB.Topic({
      ros,
      name: '/move_base/status',
      messageType: 'actionlib_msgs/GoalStatusArray'
    });

    navStatusListener.subscribe((message) => {
      const status = this.processNavigationMessage(message);
      this.setState({ navMessage: status });
    });
  };

  processNavigationMessage = (message) => {
    if (message.status_list && message.status_list.length > 0) {
      const { status, text } = message.status_list[0];
      return `Status: ${status}, Info: ${text}`;
    }
    return 'No active navigation status.';
  };

  subscribeToPaths = () => {
    const { ros } = this.state;

    if (!ros) {
      console.error('ROS connection not established for path subscription');
      return;
    }

    const globalPathListener = new window.ROSLIB.Topic({
      ros,
      name: '/move_base/NavfnROS/plan',
      messageType: 'nav_msgs/Path',
    });

    globalPathListener.subscribe((message) => {
      if (this.globalPathLayer) {
        if (message.poses && Array.isArray(message.poses) && message.poses.length > 0) {
          this.globalPathLayer.setPath(message);
        } else {
          console.warn('Received empty or invalid path message, clearing path');
          //this.globalPathLayer.setPath({ poses: [] }); // Clear the path
        }
      } else {
        console.error('globalPathLayer is not initialized');
      }
    });

    const localPathListener = new window.ROSLIB.Topic({
      ros,
      name: '/move_base/DWAPlannerROS/local_plan', // Adjust the topic name as needed
      messageType: 'nav_msgs/Path',
    });

    localPathListener.subscribe((message) => {
      if (this.localPathLayer) {
        if (message.poses && Array.isArray(message.poses) && message.poses.length > 0) {
          this.localPathLayer.setPath(message);
        } else {
          console.warn('Received empty or invalid local path message, clearing path');
          //this.localPathLayer.setPath({ poses: [] }); // Clear the path
        }
      } else {
        console.error('localPathLayer is not initialized');
      }
    });
  };

  subscribeToRobotPose = () => {
    const { ros } = this.state;
    if (!ros) {
      console.error('ROS connection not established');
      return;
    }

    const poseListener = new window.ROSLIB.Topic({
      ros,
      name: '/amcl_pose', // Typically, AMCL publishes the robot's pose on the /amcl_pose topic
      messageType: 'geometry_msgs/PoseWithCovarianceStamped',
    });

    poseListener.subscribe((message) => {
      if (this.robotMarker) {
        const pose = message.pose.pose;
        this.robotMarker.x = pose.position.x;
        this.robotMarker.y = -pose.position.y; // Note: ROS2D typically has y-axis inverted
        this.robotMarker.rotation = this.quaternionToDegrees(pose.orientation);
      } else {
        console.error('robotMarker is not initialized');
      }
    });
  };

  quaternionToDegrees = (orientation) => {
    const q0 = orientation.w;
    const q1 = orientation.x;
    const q2 = orientation.y;
    const q3 = orientation.z;

    const siny_cosp = 2 * (q0 * q3 + q1 * q2);
    const cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);

    return Math.atan2(siny_cosp, cosy_cosp) * (-180 / Math.PI);
  };

  render() {
    return (
      <div>
        <div id="nav_div" style={{ width: '1080px', height: '720px' }} />
        <div id="nav_status">
          <h3 className="mt-3">Navigation Status</h3>
          <p>{this.state.navMessage}</p>
        </div>
      </div>
    );
  }
}

export default Map;
