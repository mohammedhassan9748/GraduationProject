import React, { Component } from 'react';
import Alert from 'react-bootstrap/Alert';
import Config from '../scripts/config';

class Connection extends Component {
    state = {
        connected: false,
    };

    componentDidMount() {
        this.connectToRosBridge();
    }

    componentWillUnmount() {
        // Clear the reconnection timeout to prevent it from firing after the component has unmounted
        if (this.reconnectTimeout) {
            clearTimeout(this.reconnectTimeout);
        }
        // Properly close the connection when the component unmounts
        if (this.ros) {
            this.ros.close();
        }
    }

    connectToRosBridge = () => {
        const protocol = window.location.protocol === 'https:' ? 'wss://' : 'ws://';
        this.ros = new window.ROSLIB.Ros({
            url: `${protocol}${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`
        });

        this.ros.on('connection', () => {
            console.log('Connected to websocket server.');
            this.setState({ connected: true });
        });

        this.ros.on('error', (error) => {
            console.error('Error connecting to websocket server: ', error);
            this.setState({ connected: false });
        });

        this.ros.on('close', () => {
            console.warn('Connection to websocket server closed.');
            this.setState({ connected: false });
            // Attempt to reconnect after a specified delay
            this.scheduleReconnect();
        });
    };

    scheduleReconnect = () => {
        // Attempt to reconnect after a delay (e.g., 5 seconds)
        this.reconnectTimeout = setTimeout(() => {
            console.log('Attempting to reconnect to the ROS bridge...');
            this.connectToRosBridge();
        }, Config.RECONNECTION_TIMER); // Use the delay from the config
    };

    render() {
        return (
            <div>
                <Alert
                    className="text-center m-3"
                    variant={this.state.connected ? 'success' : 'danger'}
                >
                    {this.state.connected ? 'Robot Connected' : 'Robot Disconnected'}
                </Alert>
                <div className="main-content">
                    {/* Main content of your application */}
                </div>
            </div>
        );
    }
}

export default Connection;
