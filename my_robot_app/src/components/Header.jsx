import React, { Component } from "react";
import { Navbar,Nav,NavDropdown,Container } from "react-bootstrap";

class Header extends Component{
    render(){
        return (
            <Navbar bg="dark" variant="dark" expand="lg" collapseOnSelect>
            <Container >
              <Navbar.Brand href="/">Robot</Navbar.Brand>
              <Navbar.Toggle aria-controls="basic-navbar-nav" />
              <Navbar.Collapse id="basic-navbar-nav">
                <Nav className="me-auto">
                  <Nav.Link href="/control">Control</Nav.Link>
                  <Nav.Link href="/battery">battery</Nav.Link>
                  <Nav.Link href="/about">About</Nav.Link>
                </Nav>
              </Navbar.Collapse>
            </Container>
          </Navbar>
        );
    }
}

export default Header;