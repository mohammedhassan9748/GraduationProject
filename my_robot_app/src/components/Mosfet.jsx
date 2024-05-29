import React, { Component } from 'react';
import { Button } from 'react-bootstrap';
import Config from '../scripts/config';
import './../styles/Buttons.css'; // Import the consolidated CSS file

class Mosfet extends Component {
    constructor(props) {
        super(props);
        const savedMosfetState = JSON.parse(localStorage.getItem('mosfetState')) || false;
        this.state = {
            ros: null,
            mosfetState: savedMosfetState,
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
            url: `${Config.PROTOCOL}${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`
        });

        ros.on('connection', () => {
            console.log('Connected to websocket server for MOSFET control.');
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
            ros.connect(`${Config.PROTOCOL}${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`);
        } catch (error) {
            console.log('Connection problem:', error);
        }
    };

    toggleMosfet = () => {
        const newState = !this.state.mosfetState;
        this.setState({ mosfetState: newState }, () => {
            this.publishMosfetState();
            // Save the new state to localStorage
            localStorage.setItem('mosfetState', JSON.stringify(newState));
        });
    };

    publishMosfetState = () => {
        if (!this.state.ros) {
            console.error('ROS connection not established');
            return;
        }

        const mosfetCommand = new window.ROSLIB.Topic({
            ros: this.state.ros,
            name: '/mosfet_state',
            messageType: 'std_msgs/Bool'
        });

        const message = new window.ROSLIB.Message({
            data: this.state.mosfetState
        });

        mosfetCommand.publish(message);
    };

    render() {
        return (
            <div>
                <Button
                    onClick={this.toggleMosfet}
                    className={`control-button ${this.state.mosfetState ? "control-button-danger" : "control-button-success"}`}
                >
                    {this.state.mosfetState ? "Turn Off" : "Turn On"}
                </Button>
                <p className="current-state">Current State: <span>{this.state.mosfetState ? "ON" : "OFF"}</span></p>
            </div>
        );
    }
}

export default Mosfet;
