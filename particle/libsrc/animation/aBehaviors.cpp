#include "aBehaviors.h"
#include "aBehaviorController.h"
#include <math.h>


// Base Behavior
///////////////////////////////////////////////////////////////////////////////
Behavior::Behavior(const char* name) : m_name(name)
{
}

Behavior::Behavior(const Behavior& orig) : m_name(orig.m_name)
{
}

const std::string& Behavior::GetName() const
{
    return m_name;
}

// Behaviors derived from Behavior
//----------------------------------------------------------------------------//
// Seek behavior
///////////////////////////////////////////////////////////////////////////////
//  Global goal position is in m_Target
//  Agent's global position is in actor.getPosition()
// Given the actor, return a desired velocity in world coordinates
// Seek returns a maximum velocity towards the target

Seek::Seek(const AJoint& target) : Behavior("sek"), m_Target(target)
{

}


Seek::~Seek()
{
}

Seek::Seek(const Seek& orig) : Behavior(orig), m_Target(orig.m_Target)
{
}

vec3 Seek::calcDesiredVel(const BehaviorController& actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_Target.getLocalTranslation();
	vec3 actorPos = actor.getPosition();

	// TODO: add your code here to compute Vdesired = Vseek
 
	
	
	return Vdesired;
}


// Flee behavior
///////////////////////////////////////////////////////////////////////////////
//  Global goal position is in m_Target
//  Agent's global position is in actor.getPosition()
// Given the actor, return a desired velocity in world coordinates
// Flee calculates a a maximum velocity away from the target

Flee::Flee(const AJoint& target) : Behavior("flee"), m_Target(target)
{
}

Flee::~Flee()
{
}

Flee::Flee(const Flee& orig) :
Behavior(orig), m_Target(orig.m_Target)
{
}

vec3 Flee::calcDesiredVel(const BehaviorController& actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_Target.getLocalTranslation();
	vec3 actorPos = actor.getPosition();

	// TODO: add your code here to compute Vdesired = Vflee




	return Vdesired;

}

// Arrival behavior
///////////////////////////////////////////////////////////////////////////////
//  Global goal position is in goal
//  Agent's global position is in GPos
//  Arrival setting is in CAgent::KArrival
// Given the actor, return a desired velocity in world coordinates
// Arrival returns a velocity whose speed is proportional to the actors distance
// from the target

Arrival::Arrival(const AJoint& target) :
Behavior("arrival"),
m_Target(target)
{
}

Arrival::Arrival(const Arrival& orig) :
Behavior(orig), m_Target(orig.m_Target)
{
}

Arrival::~Arrival()
{
}

vec3 Arrival::calcDesiredVel(const BehaviorController& actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_Target.getLocalTranslation();
	vec3 actorPos = actor.getPosition();

	// TODO: add your code here to compute Vdesired = Varrival

	


	return Vdesired;
}


// Departure behavior
///////////////////////////////////////////////////////////////////////////////
//  Global goal position is in goal
//  Agent's global position is in GPos
//  Departure setting is in KDeparture
// Given the actor, return a desired velocity in world coordinates
// Departure calculates a repelent velocity based on the actor's 
// distance from the target

Departure::Departure(const AJoint& target) : Behavior("departure"), m_Target(target)
{
}

Departure::~Departure()
{
}

Departure::Departure(const Departure& orig) :
Behavior(orig), m_Target(orig.m_Target)
{
}


vec3 Departure::calcDesiredVel(const BehaviorController& actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_Target.getLocalTranslation();
	vec3 actorPos = actor.getPosition();

	// TODO: add your code here to compute Vdesired = Vdeparture
    




	return Vdesired;
}


// Avoid behavior
///////////////////////////////////////////////////////////////////////////////
//  Obstacles are in mObstacles and have class type Obstacle
//  Agent bounding sphere radius is in CAgent::radius
//  Avoidance settings are in CAgent::TAvoid and CAgent::KAvoid
// Given the actor, return a desired velocity in world coordinates
// If an actor is near an obstacle, avoid adds either a tangential or
// normal response velocity

Avoid::Avoid(const AJoint& target, const std::vector<Obstacle>& obstacles) :
Behavior("avoid"), m_Target(target), mObstacles(obstacles)
{
}

Avoid::Avoid(const Avoid& orig) :
Behavior(orig), m_Target(orig.m_Target), mObstacles(orig.mObstacles)
{
}

Avoid::~Avoid()
{
}

vec3 Avoid::calcDesiredVel(const BehaviorController& actor)
{

	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	m_actorPos = actor.getPosition();
	m_actorVel = actor.getVelocity();

 	// Step 1. compute initial value for Vdesired = Varrival so agent moves toward target
	vec3 Varrival(0, 0, 0);
	//TODO: add your code here



	// compute Vavoid 
	vec3 Vavoid(0, 0, 0);

	// Step 2. compute Lb
	//TODO: add your code here



	// Step 3. find closest obstacle 
	//TODO: add your code here



	// Step 4. determine whether agent will collide with closest obstacle (only consider obstacles in front of agent)
	//TODO: add your code here



	// Step 5.  if potential collision detected, compute Vavoid and set Vdesired = Varrival + Vavoid
	//TODO: add your code here




	return Vdesired;
	
}

void Avoid::display(const BehaviorController& actor)
{
	//  Draw Debug info
	vec3 angle = actor.getOrientation();
	vec3 vel = actor.getVelocity();
	vec3 dir = vec3(cos(angle[1]), 0, sin(angle[1]));
	vec3 probe = dir * (vel.Length()/BehaviorController::gMaxSpeed)*BehaviorController::TAvoid;
	
	glBegin(GL_LINES);
	glColor3f(0, 0, 1);
	glVertex3f(m_actorPos[0], m_actorPos[1], m_actorPos[2]);
	glVertex3f(m_obstaclePos[0], m_obstaclePos[1], m_obstaclePos[2]);
	glColor3f(0, 1, 1);
	glVertex3f(m_actorPos[0], m_actorPos[1], m_actorPos[2]);
	glVertex3f(m_actorPos[0] + probe[0], m_actorPos[1] + probe[1], m_actorPos[2] + probe[2]);
	glEnd();
}


// Wander Behavior
///////////////////////////////////////////////////////////////////////////////
// Wander setting is in KWander
// Given the actor, return a desired velocity in world coordinates
// Wander returns a velocity whose direction changes at random

Wander::Wander() : Behavior("wander"), m_Wander(1.0, 0.0, 0.0)
{
}

Wander::~Wander()
{
}

Wander::Wander(const Wander& orig) : Behavior(orig), m_Wander(orig.m_Wander)
{
}

vec3 Wander::calcDesiredVel(const BehaviorController& actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor.getPosition();

	// compute Vdesired = Vwander

	// Step. 1 find a random direction
	//TODO: add your code here




	// Step2. scale it with a noise factor
	//TODO: add your code here




	// Step3. change the current Vwander  to point to a random direction
	//TODO: add your code here;




	// Step4. scale the new wander velocity vector and add it to the nominal velocity
	//TODO: add your code here



	return Vdesired;
}


// Alignment behavior
///////////////////////////////////////////////////////////////////////////////
// agents[i] gives the pointer to the ith agent in the environment
//  Alignment settings are in CAgent::RNeighborhood and CAgent::KAlign
// Given the actor, return a desired velocity in world coordinates
// Alignment returns an average velocity of all active agents

Alignment::Alignment(const AJoint& target, const std::vector<BehaviorController>& agents) :
Behavior("alignment"), m_AgentList(agents), m_Target(target)
{
}

Alignment::~Alignment()
{
}

Alignment::Alignment(const Alignment& orig) :
Behavior(orig), m_AgentList(orig.m_AgentList), m_Target(orig.m_Target)
{
}

vec3 Alignment::calcDesiredVel(const BehaviorController& actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_Target.getLocalTranslation();
	vec3 actorPos = actor.getPosition();

	// compute Vdesired 

	// Step 1. compute value of Vdesired for fist agent (i.e. m_AgentList[0]) using an arrival behavior so it moves towards the target
	
	const BehaviorController& leader = m_AgentList[0]; // first agent is the leader
	//TODO: add your code here
	
	


	// Step 2. if not first agent compute Valign as usual
	//TODO: add your code here
	





	return Vdesired;
}

// Separation behavior
///////////////////////////////////////////////////////////////////////////////
//  agents[i] gives the pointer to the ith agent in the environment
//  Separation settings are in CAgent::RNeighborhood and KSeperate

// Given the actor, return a desired velocity in world coordinates
// Separation tries to maintain a constant distance between all agents
// within the neighborhood of the agent

Separation::Separation(const AJoint& target, const std::vector<BehaviorController>& agents) :
Behavior("separation"), m_AgentList(agents), m_Target(target)
{
}

Separation::~Separation()
{
}

Separation::Separation(const Separation& orig) :
Behavior(orig), m_AgentList(orig.m_AgentList), m_Target(orig.m_Target)
{
}


vec3 Separation::calcDesiredVel(const BehaviorController& actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_Target.getLocalTranslation();
	vec3 actorPos = actor.getPosition();

	// compute Vdesired = Vseparate
	//TODO: add your code here
	





	if (Vdesired.Length() < 5.0)
		Vdesired = 0.0;
	
	return Vdesired;

}


// Cohesion behavior
///////////////////////////////////////////////////////////////////////////////
//  agents[i] gives the pointer to the ith agent in the environment
//  Cohesion settings are in RNeighborhood and KCohesion
// Given the actor, return a desired velocity in world coordinates
// Cohesion moves actors towards the center of a group of agents

Cohesion::Cohesion(const std::vector<BehaviorController>& agents) :
Behavior("cohesion"), m_AgentList(agents)
{
}

Cohesion::~Cohesion()
{
}

Cohesion::Cohesion(const Cohesion& orig) : Behavior(orig), m_AgentList(orig.m_AgentList)
{
}

vec3 Cohesion::calcDesiredVel(const BehaviorController& actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor.getPosition();
	
	// compute Vdesired = Vseparate
	// TODO: add your code here 





	return Vdesired;
}

// Flocking behavior
///////////////////////////////////////////////////////////////////////////////
//  Utilize the Separation, Cohesion and Alignment behaviors to determine the desired velocity vector
// Given the actor, return a desired velocity in world coordinates
// Flocking combines separation, cohesion, and alignment

Flocking::Flocking(const AJoint& target, const std::vector<BehaviorController>& agents) :
Behavior("flocking"), m_Target(target), m_AgentList(agents)
{
}

Flocking::Flocking(const Flocking& orig) :
Behavior(orig), m_AgentList(orig.m_AgentList), m_Target(orig.m_Target)
{
}

Flocking::~Flocking()
{
}

vec3 Flocking::calcDesiredVel(const BehaviorController& actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor.getPosition();

	// compute Vdesired = Vflocking
	// TODO: add your code here 

	



	return Vdesired;
}

//	Leader behavior
///////////////////////////////////////////////////////////////////////////////
//  Utilize the Separation, Arrival behaviors to determine the desired velocity vector
//  You need to find the leader, who is always the first agent in CAgent::agents
// Given the actor, return a desired velocity in world coordinates
// If the actor is the leader, move towards the target; otherwise, 
// follow the leader without bunching together

Leader::Leader(const AJoint& target, std::vector<BehaviorController>& agents) :
Behavior("leader"), m_Target(target), m_AgentList(agents)
{
}

Leader::Leader(const Leader& orig) :
Behavior(orig), m_Target(orig.m_Target), m_AgentList(orig.m_AgentList)
{
}

Leader::~Leader()
{
}

vec3 Leader::calcDesiredVel(const BehaviorController& actor)
{
	
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor.getPosition();

	// TODO: compute Vdesired  = Vleader
	// followers should stay directly behind leader at a distance of -200 along the local z-axis

    const BehaviorController& leader = m_AgentList[0]; // first agent is the lead agent
	mat3 Rmat = leader.getGuide().getLocalRotation();  // is rotattion matrix of lead agent
	




	return Vdesired;
}

///////////////////////////////////////////////////////////////////////////////

