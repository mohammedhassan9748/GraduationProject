import React, { Component } from "react";
import { Row, Col } from "react-bootstrap";
import Connection from "./Connection";
import Teleop from "./Teleop";
import Map from "./Map";
import Motor from "./Motors";
import RobotState from "./RobotState";
import Mosfet from "./Mosfet";
import "./../styles/Control.css"; // Import the CSS file

class Control extends Component {
  render() {
    return (
      <div>
        <h1 className="text-center mt-3 mb-4">Robot Control Page</h1>
        <Row className="mb-3">
          <Col md={12}>
            <Connection />
          </Col>
        </Row>
        <Row className="mb-3">
          <Col md={3}>
            <h2 className="text-center">Teleop</h2>
            <div className="control-container">
              <Teleop />
            </div>
          </Col>
          <Col md={3}>
            <h2 className="text-center">Robot State</h2>
            <div className="control-container">
              <RobotState />
            </div>
          </Col>
          <Col md={3}>
            <h2 className="text-center">Motors</h2>
            <div className="control-container">
              <Mosfet />
            </div>
          </Col>
          <Col md={3}>
            <h2 className="text-center">Pick</h2>
            <div className="control-container">
              <Motor />
            </div>
          </Col>
        </Row>
        <Row className="mb-3">
          <Col md={12}>
            <h2 className="text-center">Map</h2>
            <Map />
          </Col>
        </Row>
      </div>
    );
  }
}

export default Control;
