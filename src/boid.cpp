
// glm
#include <glm/gtc/random.hpp>

// project
#include "boid.hpp"
#include "scene.hpp"
#include "cgra/cgra_mesh.hpp"

//for testing
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/string_cast.hpp"

#include <algorithm>

using namespace glm;
using namespace std;

vec3 Boid::color() const {
	if(groupId == 0) return vec3(0.537,0.824,0.839); //flock 0
    else if(groupId == 1) return vec3(0.537,0.624,0.939); //flock 1
    else if(groupId == 2) return vec3(0.837,0.255,0.937); //predator
    else return vec3(0.537,0.624,0.939); //obstacle
}

void Boid::calculateForces(Scene *scene) {
	//-------------------------------------------------------------
	// [Assignment 3] :
	// Calculate the forces affecting the boid and update the
	// acceleration (assuming mass = 1).
	// Do NOT update velocity or position in this function.
	// Core : 
	//  - Cohesion
	//  - Alignment
	//  - Avoidance
	//  - Soft Bound (optional)
	// Completion : 
	//  - Cohesion and Alignment with only boids in the same flock
	//  - Predator Avoidance (boids only)
	//  - Predator Chase (predator only)
	// Challenge : 
	//  - Obstacle avoidance
	//-------------------------------------------------------------
    
    if(groupId == 3){
        m_acceleration = vec3(0.0);
        return;
    }
    
    vec3 avoidance = vec3(0.0);
    vec3 avgNeighbourPos = vec3(0.0); vec3 cohesion = vec3(0.0);
    vec3 avgNeighbourVelocity = vec3(0.0); vec3 alignment = vec3(0.0);
    int numNeighbours = 0;
    float chaseDist = MAXFLOAT;
    vec3 preyPosition = vec3(0.0);
    //find neighbours of boid
    for(Boid other: scene->boids()){
        if(&other == this) continue; //skip itself
        float d = distance(m_position, other.m_position);
        //within range and not a predator, don't use flocking rules for obstacles
        if(d < scene->neighbourDist && d > 0 && groupId != 2 && other.groupId != 3){
            //avoidance
            vec3 displacement = m_position - other.m_position; //vector pointing away from neighbour
            float distance = length(displacement);
            avoidance += displacement / (distance * distance); //collect avoidance
            //cohesion and alignment
            if(groupId == other.groupId){
                numNeighbours++;
                avgNeighbourPos += other.m_position; //collect neighbour position
                avgNeighbourVelocity += other.m_velocity; //collect neighbour velocity
            }
            //predator avoidance
            if(other.groupId == 2){
                avoidance *= 100; //avoid predators much more that regular boids
            }
        }
        //predators avoid other predators
        else if(d < scene->pNeighbourDist && d > 0 && groupId == 2 && other.groupId == 2){
            vec3 displacement = m_position - other.m_position; //vector pointing away from neighbour
            float distance = length(displacement);
            avoidance += displacement / (distance * distance); //collect avoidance
        }
        //find neighbour closest to predator to chase it
        if(groupId == 2 && other.groupId !=2 && d < scene->pNeighbourDist){
            if(d < chaseDist){
                chaseDist = d;
                preyPosition = other.m_position;
            }
        }
        
        //obstacle avoidance - apply force away from obstacle if boid gets close enough
        if(other.groupId == 3){
            //Ray Tracing algorithm
            vec3 O = m_position;
            vec3 D = normalize(m_velocity);
            vec3 C = other.m_position;
            float R = 8.0; //allowing extra for cylinder
            
            float a = dot(D, D);
            float b = 2 * dot(O-C, D);
            float c = dot(O-C, O-C) - (R*R);
            
            //discriminant - <0 no intersection, >=0 intersection
            float discriminant = (b*b) - 4*a*c;
            if(discriminant >= 0){
                
                float t0 = (-b - sqrt(discriminant)) / (2*a);
                float t1 = (-b + sqrt(discriminant)) / (2*a);
                
                //intersection
                if(t0 > 0 && t1 > 0 && t0 < 13){ //distance limit of 15 units away to apply force
                    highPriority = true;
                    
                    float OE = (dot(O*C, D)) / length(D);
                    vec3 E = O + OE * normalize(D);
                    vec3 f = normalize(E - C);

                    m_acceleration += f* 1000.0f;
                } else{
                    highPriority = false;
                }
            }
        }
    }
    
    //calulate cohesion and alignment
    if(numNeighbours > 0){
        avgNeighbourPos = avgNeighbourPos/(float)numNeighbours;
        cohesion = avgNeighbourPos - m_position; //vector pointing to average
        avgNeighbourVelocity = avgNeighbourVelocity/(float)numNeighbours;
        alignment = avgNeighbourVelocity - m_velocity; //vector pointing to average
    } else{
        cohesion = vec3(0.0);
        alignment = vec3(0.0);
    }
    
    //apply soft bound if enabled
    if(scene->selectedMode == 2){
        if(m_position.x < -scene->bound().x || m_position.x > scene->bound().x ||
           m_position.y < -scene->bound().y || m_position.y > scene->bound().y ||
           m_position.z < -scene->bound().z || m_position.z > scene->bound().z) {
            vec3 displacement = -m_position;
            float distance = length(displacement);
            m_acceleration += (displacement / (distance * distance)) * scene->softWeight * 1000.0f;
        }
    }
    
    //apply forces to acceleration
    if(groupId != 2){
        if(!highPriority) {
            m_acceleration += avoidance * scene->avoidWeight * scene->pAvoidWeight
                            + cohesion * scene->cohesionWeight
                            + alignment * scene->alignWeight;
        } else{
            m_acceleration += alignment * scene->alignWeight; //obstacle avoidance takes higher priority
        }
        
        
        if(length(m_acceleration) > scene->maxAccel) m_acceleration = scene->maxAccel * normalize(m_acceleration);
    } else{
        //predator chases prey that is closest to it
        vec3 displacement = preyPosition - m_position; //vector pointing towards neighbour
        float distance = length(displacement);
        vec3 chase = displacement / (distance * distance); //collect chasing force

        m_acceleration += avoidance * scene->pAvoidWeight + chase * scene->pChaseWeight;
        if(length(m_acceleration) > scene->pMaxAccel) m_acceleration = scene->pMaxAccel * normalize(m_acceleration);
    }

}

void Boid::update(float timestep, Scene *scene) {
	//-------------------------------------------------------------
	// [Assignment 3] :
	// Integrate the velocity of the boid using the timestep.
	// Update the position of the boid using the new velocity.
	// Take into account the bounds of the scene which may
	// require you to change the velocity (if bouncing) or
	// change the position (if wrapping).
	//-------------------------------------------------------------
    
    if(groupId == 3){
        m_velocity = vec3(0.0);
        return;
    }
    
    //update velocity
    m_velocity += m_acceleration * timestep;
    float speed = length(m_velocity);
    if(groupId != 2) speed = std::clamp(speed,scene->minSpeed,scene->maxSpeed); //clamp speed
    else speed = std::clamp(speed, scene->pMinSpeed,scene->pMaxSpeed); //clamp predator speed
    
    m_velocity = speed * normalize(m_velocity);
    
    //update position
    m_position += m_velocity * timestep;
    
    //containment
    if(scene->selectedMode == 0){ //wrap
        if(m_position.x < -scene->bound().x) m_position.x = scene->bound().x;
        else if(m_position.x > scene->bound().x) m_position.x = -scene->bound().x;
        if(m_position.y < -scene->bound().y) m_position.y = scene->bound().y;
        else if(m_position.y > scene->bound().y) m_position.y = -scene->bound().y;
        if(m_position.z < -scene->bound().z) m_position.z = scene->bound().z;
        else if(m_position.z > scene->bound().z) m_position.z = -scene->bound().z;
    }
    else if(scene->selectedMode == 1){ //bounce
        if(m_position.x < -scene->bound().x || m_position.x > scene->bound().x ||
           m_position.y < -scene->bound().y || m_position.y > scene->bound().y ||
           m_position.z < -scene->bound().z || m_position.z > scene->bound().z) {
            m_velocity = -m_velocity;
            m_position += m_velocity * timestep; //make a step to prevent boids getting stuck on boundary
        }
    }
}
