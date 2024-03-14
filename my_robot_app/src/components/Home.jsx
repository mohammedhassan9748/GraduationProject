import React, { Component } from "react";
import Connection from "./Connection";
import Teleop from "./Teleop";
import { Row, Col, Container, Button } from "react-bootstrap";
class Home extends Component {
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
        </Container>
      </div>
    );
  }
}

export default Home;