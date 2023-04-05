
#pragma once

// glm
#include <glm/glm.hpp>

// project
#include "scene.hpp"


class Boid {
private:

	glm::vec3 m_position;
	glm::vec3 m_velocity;
	glm::vec3 m_acceleration;
    int groupId; //the group this boid is in
    bool highPriority = false;

public:
	Boid(glm::vec3 pos, glm::vec3 dir, int id) : m_position(pos), m_velocity(dir), groupId(id) { }

	glm::vec3 position() const { return m_position; }
	glm::vec3 velocity() const { return m_velocity; }
	glm::vec3 acceleration() const { return m_acceleration; }

	glm::vec3 color() const;

	void calculateForces(Scene *scene); 
	void update(float timestep, Scene *scene);
    
    //get group id of boid
    int id() const {return groupId;}
};
