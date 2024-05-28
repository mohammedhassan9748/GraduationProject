import batteryImage from '../imgs/Battery.png';
import React, { Component } from 'react';
import Config from '../scripts/config';
import './../styles/Battery.css';  // Import the CSS file

class Battery extends Component {
    constructor(props) {
        super(props);
        this.state = {
            ros: null,
            voltages: Array(3).fill('Waiting for data...'),  // Placeholder for 3 cell voltages
            totalVoltage: 'Waiting for total voltage...'
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
        const protocol = window.location.protocol === 'https:' ? 'wss://' : 'ws://';
        const ros = new window.ROSLIB.Ros({
            url: `${protocol}${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`
        });

        ros.on('connection', () => {
            console.log('Connected to websocket server.');
            this.setState({ ros }, this.subscribeToBatteryVoltage);
        });

        ros.on('close', () => {
            console.log('Disconnected from websocket server.');
            this.setState({ ros: null });
        });

        ros.on('error', (error) => {
            console.error('Error connecting to websocket server:', error);
        });

        try {
            ros.connect(`${protocol}${Config.ROSBRIDGE_SERVER_IP}:${Config.ROSBRIDGE_SERVER_PORT}`);
        } catch (error) {
            console.log('Connection problem:', error);
        }
    };

    subscribeToBatteryVoltage = () => {
        const { ros } = this.state;
        if (!ros) {
            console.error('ROS connection not established');
            return;
        }

        const batteryVoltageListener = new window.ROSLIB.Topic({
            ros: ros,
            name: '/battery_voltage',
            messageType: 'std_msgs/String'
        });

        batteryVoltageListener.subscribe((message) => {
            const data = message.data.split(', ');
            const voltages = data.slice(0, 3).map(v => v.trim());  // Extract individual cell voltages
            const totalVoltage = data[3].trim();  // Total voltage is the last element
            this.setState({ voltages, totalVoltage });
        });
    };

    render() {
        const imageUrl = batteryImage; // Path to the image used for all cells
        return (
            <div className="battery-container">
                <h3>Battery Voltage</h3>
                <div className="battery-grid">
                    {this.state.voltages.map((voltage, index) => (
                        <div key={index} className="battery-cell">
                            <img src={imageUrl} alt={`Cell ${index + 1}`} />
                            <p>Cell {index + 1}</p>
                            <span>{voltage}</span>
                        </div>
                    ))}
                </div>
                <div className="total-voltage">
                    <h4>Total Voltage:</h4>
                    <span>{this.state.totalVoltage}</span>
                </div>
            </div>
        );
    }
}

export default Battery;
