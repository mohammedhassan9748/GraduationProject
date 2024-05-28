import React, { Component } from "react";
import { Container } from "react-bootstrap";
import { Route, BrowserRouter as Router, Routes } from "react-router-dom";
import Control from "./Control"
import Battery from "./Battery"
import About from "./About"

class Body extends Component{

    render(){
        return (
            <main>
            <Container>
                <Router>
                    <Routes>
                        <Route path="/control" element={<Control/>}></Route>
                        <Route path="/battery" element={<Battery/>}></Route>
                        <Route path="/about" element={<About/>}></Route>   
                    </Routes>
                </Router>
            </Container>
            </main>
        )
    }
}

export default Body;