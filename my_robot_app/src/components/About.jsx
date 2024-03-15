import React, { Component } from "react";
import teamMembers from '../scripts/teamMembers';

class About extends Component{
    render(){
        return (
            <div className="about-container mt-3">
              <h1>About Us</h1>
              <div className="team-members">
                {teamMembers.map((member, index) => (
                  <div key={index} className="team-member">
                    <img src={member.imageUrl} alt={member.name} />
                    <h2>{member.name}</h2>
                    <p>{member.role}</p>
                  </div>
                ))}
              </div>
            </div>
        );
    }
}

export default About;