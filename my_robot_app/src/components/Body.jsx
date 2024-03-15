import React, { Component } from "react";
import { Container } from "react-bootstrap";
import { Route, BrowserRouter as Router, Routes } from "react-router-dom";
import Mapping from "./Mapping"
import About from "./About"

class Body extends Component{

    render(){
        return (
            <main>
            <Container>
                <Router>
                    <Routes>
                        <Route path="/mapping" element={<Mapping/>}></Route>
                        <Route path="/about" element={<About/>}></Route>
                    </Routes>
                </Router>
            </Container>
            </main>
        )
    }
}

export default Body;