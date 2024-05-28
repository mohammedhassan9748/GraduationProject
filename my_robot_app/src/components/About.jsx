import React, { Component } from "react";
import teamMembers from '../scripts/teamMembers';
import './../styles/About.css'; 

class About extends Component {
    render() {
        // Remove the last team member from the array
        const filteredTeamMembers = teamMembers.slice(0, -1);

        return (
            <div className="about-container mt-3">
                <h1>About Us</h1>
                <div className="team-members">
                    {filteredTeamMembers.map((member, index) => (
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
