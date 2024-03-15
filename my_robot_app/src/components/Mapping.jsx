import React, { Component } from "react";
import Connection from "./Connection";
import Teleop from "./Teleop";
import Map from "./Map";
import { Row, Col, Container, Button } from "react-bootstrap";
import RobotState from "./RobotState";

class Mapping extends Component {
  state = {};

  render() {
    return (
      <div>
        <Container>
          <h1 className="text-center mt-3">Robot Control Page</h1>
          <Row>
            <Col>
              <Connection />
            </Col>
          </Row>
          <Row>
            <Col>
              <Teleop />
            </Col>
          </Row>
          <Row>
            {" "}
            <Col>
              <RobotState />
            </Col>
            <Col>
              <h1>MAP</h1>
              <Map />
            </Col>
          </Row>
        </Container>
      </div>
    );
  }
}

export default Mapping;