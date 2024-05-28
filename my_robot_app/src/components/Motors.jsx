import React, { Component } from 'react';
import { Button } from 'react-bootstrap';
import Config from '../scripts/config';
import './../styles/Buttons.css'; // Import the consolidated CSS file

class Motors extends Component {
    constructor(props) {
        super(props);
        const savedStatus = localStorage.getItem('motorStatus') || 'drop';
        this.state = {
            ros: null,
            motorStatus: savedStatus,
        };
    }

    componentDidMount() {
        this.initConnection();
    }

    componentWillUnmount() {
        if (this.state.ros) {
            this.state.ros.close();
        }
    }

    initConnection = () => {
        const ros = new window.ROSLIB.Ros({
            url: `ws://${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`
        });

        ros.on('connection', () => {
            console.log('Connected to websocket server for motor control.');
            this.setState({ ros });
        });

        ros.on('close', () => {
            console.log('Connection to websocket server closed.');
            this.setState({ ros: null });
        });

        ros.on('error', (error) => {
            console.error('Connection problem:', error);
        });

        try {
            ros.connect(`ws://${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`);
        } catch (error) {
            console.log('Connection problem:', error);
        }
    };

    toggleMotor = () => {
        const newStatus = this.state.motorStatus === 'lift' ? 'drop' : 'lift';
        this.setState({ motorStatus: newStatus }, () => {
            this.publishMotorStatus();
            localStorage.setItem('motorStatus', newStatus);
        });
    };

    publishMotorStatus = () => {
        if (!this.state.ros) {
            console.error('ROS connection not established');
            return;
        }

        const motorCommand = new window.ROSLIB.Topic({
            ros: this.state.ros,
            name: '/chatter', // The topic name should match the one in the Arduino code
            messageType: 'std_msgs/String'
        });

        const message = new window.ROSLIB.Message({
            data: this.state.motorStatus
        });

        motorCommand.publish(message);
    };

    render() {
        return (
            <div>
                <Button
                    onClick={this.toggleMotor}
                    className="control-button control-button-info"
                >
                    {this.state.motorStatus === 'lift' ? "Drop" : "Lift"}
                </Button>
                <p className="current-state">Current command: <span>{this.state.motorStatus}</span></p>
            </div>
        );
    }
}

export default Motors;
