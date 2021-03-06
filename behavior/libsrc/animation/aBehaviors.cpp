#include "aBehaviors.h"

#include <math.h>
#include "GL/glew.h"
#include "GL/glut.h"
#include "aBehaviorController.h"

// Base Behavior
///////////////////////////////////////////////////////////////////////////////
Behavior::Behavior()
{
}

Behavior::Behavior( char* name) 
{
	m_name = name;
	m_pTarget = NULL;
}

Behavior::Behavior( Behavior& orig) 
{
	m_name = orig.m_name;
	m_pTarget = NULL;
}

string& Behavior::GetName() 
{
    return m_name;
}

// Behaviors derived from Behavior
//----------------------------------------------------------------------------//
// Seek behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity in world coordinates
// Seek returns a maximum velocity towards the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position


Seek::Seek( AJoint* target) 
{
	m_name = "seek";
	m_pTarget = target;

}

Seek::Seek( Seek& orig) 
{
	m_name = "seek";
	m_pTarget = orig.m_pTarget;
}


Seek::~Seek()
{
}

vec3 Seek::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();//global?
	vec3 actorPos = actor->getPosition();//global

	// TODO: add your code here to compute Vdesired
	Vdesired = targetPos - actorPos;
	Vdesired = Vdesired.Normalize() * BehaviorController::gMaxSpeed;

	return Vdesired;
}


// Flee behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity in world coordinates
// Flee calculates a a maximum velocity away from the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position

Flee::Flee( AJoint* target) 
{
	m_name = "flee";
	m_pTarget = target;
}

Flee::Flee( Flee& orig) 
{
	m_name = "flee";
	m_pTarget = orig.m_pTarget;
}

Flee::~Flee()
{
}

vec3 Flee::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();

	// TODO: add your code here to compute Vdesired
	Vdesired = actorPos - targetPos;
	Vdesired = Vdesired.Normalize() * BehaviorController::gMaxSpeed;

	return Vdesired;

}

// Arrival behavior
///////////////////////////////////////////////////////////////////////////////
// Given the actor, return a desired velocity in world coordinates
// Arrival returns a desired velocity vector whose speed is proportional to
// the actors distance from the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position
//  Arrival strength is in BehavioralController::KArrival


Arrival::Arrival( AJoint* target) 
{
	m_name = "arrival";
	m_pTarget = target;
}

Arrival::Arrival( Arrival& orig) 
{
	m_name = "arrival";
	m_pTarget = orig.m_pTarget;
}

Arrival::~Arrival()
{
}

vec3 Arrival::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();

	// TODO: add your code here to compute Vdesired
	Vdesired = BehaviorController::KArrival * (targetPos - actorPos);

	return Vdesired;
}


// Departure behavior
///////////////////////////////////////////////////////////////////////////////
// Given the actor, return a desired velocity in world coordinates
// Arrival returns a desired velocity vector whose speed is proportional to
// 1/(actor distance) from the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position
//  Departure strength is in BehavioralController::KDeparture

Departure::Departure(AJoint* target) 
{
	m_name = "departure";
	m_pTarget = target;
}

Departure::Departure( Departure& orig) 
{
	m_name = "departure";
	m_pTarget = orig.m_pTarget;
}

Departure::~Departure()
{
}

vec3 Departure::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();

	// TODO: add your code here to compute Vdesired
	vec3 error = targetPos - actorPos;
	float magerror = error.Length();
	Vdesired = BehaviorController::KDeparture * (-error / (magerror * magerror));

	return Vdesired;
}


// Avoid behavior
///////////////////////////////////////////////////////////////////////////////
//  For the given the actor, return a desired velocity in world coordinates
//  If an actor is near an obstacle, avoid adds a normal response velocity to the 
//  the desired velocity vector computed using arrival
//  Agent bounding sphere radius is in BehavioralController::radius
//  Avoidance parameters are  BehavioralController::TAvoid and BehavioralController::KAvoid

Avoid::Avoid(AJoint* target, vector<Obstacle>* obstacles) 
{
	m_name = "avoid";
	m_pTarget = target;
	mObstacles = obstacles;
}

Avoid::Avoid( Avoid& orig) 
{
	m_name = "avoid";
	m_pTarget = orig.m_pTarget;
	mObstacles = orig.mObstacles;
}

Avoid::~Avoid()
{
}

vec3 Avoid::calcDesiredVel( BehaviorController* actor)
{

	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 Vavoid(0, 0, 0);
	vec3 Varrival(0, 0, 0);
	vec3 actorPos = actor->getPosition();
	vec3 actorVel = actor->getVelocity();

	//TODO: add your code here

	// Step 1. compute initial value for Vdesired = Varrival so agent moves toward target
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 toTarget = targetPos - actorPos;
	vec3 toTarget_norm = toTarget;
	toTarget_norm = toTarget_norm.Normalize();
	Vdesired = BehaviorController::KArrival * toTarget;

	// Step 2. compute Lb
	//TODO: add your code here
	float Lb = BehaviorController::TAvoid *actorVel.Length();

	// Step 3. find closest obstacle 
	//TODO: add your code here
	Obstacle min_obs;
	vec3 obsPos;
	vec3 toObs;
	float min_dist = FLT_MAX;

	for (int i = 0; i < mObstacles->size(); i++) {
		Obstacle obs = mObstacles->at(i);
		obsPos = obs.m_Center.getLocalTranslation();
		//std::cout << "\nAVOID: local obspos: " << obsPos;
		toObs = obsPos - actorPos;
		vec3 toObs_norm = toObs;
		toObs_norm = toObs_norm.Normalize();
		float dist = toObs.Length();
		if (dist < min_dist && Dot(toTarget_norm, toObs_norm) > 0.f) {
			min_dist = dist;
			min_obs = obs;
		}
	}

	if (min_dist == FLT_MAX || toTarget.Length() < min_dist) {
		return Vdesired;
	}

	// Step 4. determine whether agent will collide with closest obstacle (only consider obstacles in front of agent)
	//TODO: add your code here
	//projection way
	bool collide = false;
	obsPos = min_obs.m_Center.getLocalTranslation();
	toObs = obsPos - actorPos;
	float toTarget_dist = toTarget.Length();
	float actorRadius = BehaviorController::gAgentRadius;
	float obsRadius = min_obs.m_Radius;
	float dotprod = Dot(toObs, toTarget);
	vec3 dZ_world = toTarget * (dotprod / (toTarget_dist * toTarget_dist));
	vec3 dX_world = toObs - dZ_world;
	if (dZ_world.Length() <= (Lb + (actorRadius + obsRadius))) {
		//std::cout << "\nAVOID: dZ_world length: " << dZ_world.Length() << "  obsRadius: " << obsRadius;
		if (dX_world.Length() <= (actorRadius + obsRadius)) {
			collide = true;
		}
	}

	//rotation way
	//vec3 ori = actor->getOrientation();
	//float y_ori = ori[_Y];
	//mat3 body2world = mat3::Rotation3D(_Y, y_ori);
	//mat3 world2body = body2world.Transpose();

	//obsPos = min_obs.m_Center.getLocalTranslation();
	//vec3 dworld = obsPos - actorPos;
	//vec3 dbody = world2body * dworld;

	//float absDx = fabs(dbody[0]);
	//float absDz = fabs(dbody[2]);


	//if (absDz <= (Lb + (actorRadius + obsRadius))) {
	//	std::cout << "\nAVOID: absDx: " << absDx << "  obsRadius: " << obsRadius;
	//	if (absDx <= (actorRadius + obsRadius)) {
	//		collide = true;
	//	}
	//}
	
	// Step 5.  if potential collision detected, compute Vavoid and set Vdesired = Varrival + Vavoid
	//TODO: add your code here
	if (collide) {
		//vec3 dxbody(dbody[0], 0, 0);
		//vec3 dX_world = body2world * dxbody;
		vec3 normal_avoid = -dX_world / dX_world.Length();
		float mag_avoid = BehaviorController::KAvoid * (actorRadius + obsRadius - dX_world.Length());
		mag_avoid = mag_avoid / (actorRadius + obsRadius);
		Vdesired = Vdesired + (mag_avoid * normal_avoid);
	}

	return Vdesired;
	
}

void Avoid::display( BehaviorController* actor)
{
	//  Draw Debug info
	vec3 angle = actor->getOrientation();
	vec3 vel = actor->getVelocity();
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
// For the given the actor, return a desired velocity in world coordinates
// Wander returns a desired velocity vector whose direction changes at randomly from frame to frame
// Wander strength is in BehavioralController::KWander

Wander::Wander() 
{
	m_name = "wander";
	m_Wander = vec3(1.0, 0.0, 0.0);
}

Wander::Wander( Wander& orig) 
{
	m_name = "wander";
	m_Wander = orig.m_Wander;
}

Wander::~Wander()
{
}

vec3 Wander::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vec3 actorVel = actor->getVelocity();

	// compute Vdesired = Vwander

	// Step. 1 find a random direction
	//TODO: add your code here
	float randx = (((double)rand() / (RAND_MAX)) * 2) - 1;
	float randy = (((double)rand() / (RAND_MAX)) * 2) - 1;
	float randz = (((double)rand() / (RAND_MAX)) * 2) - 1;
	vec3 n(randx, randy, randz);
	float random = (((double)rand() / (RAND_MAX)) * 2) - 1;
	mat3 randrot = mat3::Rotation3D(1, random * M_PI_2);
	n = randrot * n;

	// Step2. scale it with a noise factor
	//TODO: add your code here
	vec3 rnoise = BehaviorController::KNoise * (n / n.Length());
	
	// Step3. change the current Vwander  to point to a random direction
	//TODO: add your code here
	vec3 curr_vwander = BehaviorController::KWander * m_Wander;//read out with actual length
	vec3 new_vwander = (curr_vwander + rnoise ) / (curr_vwander + rnoise).Length();
	m_Wander = new_vwander;//save normalized
	
	float maxfrac = 0.1f;
	actorVel = BehaviorController::gMaxSpeed * maxfrac * actorVel.Normalize();

	Vdesired = actorVel + (BehaviorController::KWander * new_vwander);
	Vdesired = Vdesired.Normalize() * BehaviorController::gMaxSpeed * maxfrac;

	//Vdesired = BehaviorController::KWander * new_vwander;

	// Step4. scale the new wander velocity vector and add it to the nominal velocity
	//TODO: add your code here

	return Vdesired;
}


// Alignment behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity vector in world coordinates
// Alignment returns the average velocity of all active agents in the neighborhood
// agents[i] gives the pointer to the ith agent in the environment
// Alignment parameters are in BehavioralController::RNeighborhood and BehavioralController::KAlign


Alignment::Alignment(AJoint* target, vector<AActor>* agents) 
{
	m_name = "alignment";
	m_pAgentList = agents;
	m_pTarget = target;
}



Alignment::Alignment( Alignment& orig) 
{
	m_name = orig.m_name;
	m_pAgentList = orig.m_pAgentList;
	m_pTarget = orig.m_pTarget;

}

Alignment::~Alignment()
{
}

vec3 Alignment::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_pAgentList;
	

	// compute Vdesired 
	
	// Step 1. compute value of Vdesired for fist agent (i.e. m_AgentList[0]) using an arrival behavior so it moves towards the target
	 
	BehaviorController* leader = agentList[0].getBehaviorController(); // first agent is the leader
	//TODO: add your code here
	if (actor == leader) {
		Vdesired = BehaviorController::KArrival * (targetPos - actorPos);
	}
	else {
		// Step 2. if not first agent compute Valign as usual
		//TODO: add your code here
		vec3 totalVel(0, 0, 0);
		for (int i = 0; i < agentList.size(); i++) {
			BehaviorController* agent = agentList[i].getBehaviorController();
			vec3 thispos = agent->getPosition();
			float thisdist = (actorPos - thispos).Length();
			if (thisdist > BehaviorController::RNeighborhood) {
				continue;
			}

			vec3 thisvel = agent->getVelocity();
			totalVel += thisvel;
			//is there a weight somewhere?
		}
		totalVel = BehaviorController::KAlignment * totalVel / agentList.size();
		Vdesired = totalVel;
	}
	
	
	return Vdesired;
}

// Separation behavior
///////////////////////////////////////////////////////////////////////////////
// For the given te actor, return a desired velocity vector in world coordinates
// Separation tries to maintain a constant distance between all agents
// within the neighborhood
// agents[i] gives the pointer to the ith agent in the environment
// Separation settings are in BehavioralController::RNeighborhood and BehavioralController::KSeperate

 

Separation::Separation( AJoint* target,  vector<AActor>* agents) 
{
	m_name = "separation";
	m_AgentList = agents;
	m_pTarget = target;
}

Separation::~Separation()
{
}

Separation::Separation( Separation& orig) 
{
	m_name = "separation";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

vec3 Separation::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;

		//TODO: add your code here
		vec3 total(0, 0, 0);
		for (int i = 0; i < agentList.size(); i++) {
			BehaviorController* agent = agentList[i].getBehaviorController();
			vec3 thispos = agent->getPosition();
			vec3 thisdisp = actorPos - thispos;
			float thisdist = (thisdisp).Length();
			if (thisdist > BehaviorController::RNeighborhood) {
				continue;
			}

			//is there a weight somewhere?
			if (thisdist > 0.1) {
				total = total + ((thisdisp) / (thisdist * thisdist));
			}

		}
		total = BehaviorController::KSeparation * total;
		Vdesired = total;

	if (Vdesired.Length() < 5.0)
		Vdesired = 0.0;
	
	return Vdesired;
}


// Cohesion behavior
///////////////////////////////////////////////////////////////////////////////
// For the given actor, return a desired velocity vector in world coordinates
// Cohesion moves actors towards the center of the group of agents in the neighborhood
//  agents[i] gives the pointer to the ith agent in the environment
//  Cohesion parameters are in BehavioralController::RNeighborhood and BehavioralController::KCohesion


Cohesion::Cohesion( vector<AActor>* agents) 
{
	m_name = "cohesion";
	m_AgentList = agents;
}

Cohesion::Cohesion( Cohesion& orig) 
{
	m_name = "cohesion";
	m_AgentList = orig.m_AgentList;
}

Cohesion::~Cohesion()
{
}

vec3 Cohesion::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;
	
	// compute Vdesired = Vcohesion
	// TODO: add your code here 
	vec3 centermass(0, 0, 0);
	for (int i = 0; i < agentList.size(); i++) {
		BehaviorController* agent = agentList[i].getBehaviorController();
		vec3 thispos = agent->getPosition();
		float thisdist = (actorPos - thispos).Length();
		if (thisdist > BehaviorController::RNeighborhood) {
			continue;
		}
		//is there a weight somewhere?
		centermass += thispos;
	}
	centermass = centermass / agentList.size();

	Vdesired = BehaviorController::KCohesion * (centermass - actorPos);

	return Vdesired;
}

// Flocking behavior
///////////////////////////////////////////////////////////////////////////////
// For the given actor, return a desired velocity vector  in world coordinates
// Flocking combines separation, cohesion, and alignment behaviors
//  Utilize the Separation, Cohesion and Alignment behaviors to determine the desired velocity vector


Flocking::Flocking( AJoint* target,  vector<AActor>* agents) 
{
	m_name = "flocking";
	m_AgentList = agents;
	m_pTarget = target;
}

Flocking::Flocking( Flocking& orig) 
{
	m_name = "flocking";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

Flocking::~Flocking()
{
}

vec3 Flocking::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 Vseparate = vec3(0.0, 0.0, 0.0);
	vec3 Vcohesion = vec3(0.0, 0.0, 0.0);
	vec3 Valignment = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vector<AActor>& agentList = *m_AgentList;

	// compute Vdesired = Vflocking
	// TODO: add your code here 
	float c_separate = 4;
	float c_cohesion = 4;
	float c_alignment = 1;


	///SEPARATE
	vec3 total(0, 0, 0);
	for (int i = 0; i < agentList.size(); i++) {
		BehaviorController* agent = agentList[i].getBehaviorController();
		vec3 thispos = agent->getPosition();
		vec3 thisdisp = actorPos - thispos;
		float thisdist = (thisdisp).Length();
		if (thisdist > BehaviorController::RNeighborhood) {
			continue;
		}

		//is there a weight somewhere?
		if (thisdist > 0.1) {
			total = total + ((thisdisp) / (thisdist * thisdist));
		}

	}
	Vseparate = BehaviorController::KSeparation * total;
	if (Vseparate.Length() < 5.0) {
		Vseparate = 0.0;
	}


	///COHESION
	vec3 centermass(0, 0, 0);
	for (int i = 0; i < agentList.size(); i++) {
		BehaviorController* agent = agentList[i].getBehaviorController();
		vec3 thispos = agent->getPosition();
		float thisdist = (actorPos - thispos).Length();
		if (thisdist > BehaviorController::RNeighborhood) {
			continue;
		}
		//is there a weight somewhere?
		centermass += thispos;
	}
	centermass = centermass / agentList.size();

	Vcohesion = BehaviorController::KCohesion * (centermass - actorPos);



	///ALIGNMENT
    BehaviorController* leader = agentList[0].getBehaviorController(); // first agent is the leader
																	   //TODO: add your code here
	if (actor == leader) {
		Valignment = BehaviorController::KArrival * (targetPos - actorPos);
	}
	else {
		// Step 2. if not first agent compute Valign as usual
		//TODO: add your code here
		vec3 totalVel(0, 0, 0);
		for (int i = 0; i < agentList.size(); i++) {
			BehaviorController* agent = agentList[i].getBehaviorController();
			vec3 thispos = agent->getPosition();
			float thisdist = (actorPos - thispos).Length();
			if (thisdist > BehaviorController::RNeighborhood) {
				continue;
			}

			vec3 thisvel = agent->getVelocity();
			totalVel += thisvel;
			//is there a weight somewhere?
		}
		totalVel = BehaviorController::KAlignment * totalVel / agentList.size();
		Valignment = totalVel;
	}

	Vdesired = c_separate * Vseparate + c_cohesion * Vcohesion + c_alignment * Valignment;

	return Vdesired;
}

//	Leader behavior
///////////////////////////////////////////////////////////////////////////////
// For the given actor, return a desired velocity vector in world coordinates
// If the agent is the leader, move towards the target; otherwise, 
// follow the leader at a set distance behind the leader without getting to close together
//  Utilize Separation and Arrival behaviors to determine the desired velocity vector
//  You need to find the leader, who is always agents[0]

Leader::Leader( AJoint* target, vector<AActor>* agents) 
{
	m_name = "leader";
	m_AgentList = agents;
	m_pTarget = target;
}

Leader::Leader( Leader& orig) 
{
	m_name = "leader";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

Leader::~Leader()
{
}

vec3 Leader::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vseparate = vec3(0, 0, 0);
	vec3 Varrival = vec3(0, 0, 0);
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;

	// TODO: compute Vdesired  = Vleader
	// followers should stay directly behind leader at a distance of -200 along the local z-axis

	float CSeparation = 4.0;  float CArrival = 2.0;

	BehaviorController* leader = agentList[0].getBehaviorController(); // first agent is the leader
	mat3 Rmat = leader->getGuide().getLocalRotation();  // is rotattion matrix of lead agent
	//vec3 leaderORI = leader->getOrientation();
	//mat3 Rmat = mat3::Rotation3D(1, leaderORI[_Y]);

	///SEPARATE
	vec3 total(0, 0, 0);
	if (actor != leader) {
		for (int i = 1; i < agentList.size(); i++) {
			BehaviorController* agent = agentList[i].getBehaviorController();
			vec3 thispos = agent->getPosition();
			vec3 thisdisp = actorPos - thispos;
			float thisdist = (thisdisp).Length();
			if (thisdist > BehaviorController::RNeighborhood) {
				continue;
			}

			//is there a weight somewhere?
			if (thisdist > 0.1) {
				total = total + ((thisdisp) / (thisdist * thisdist));
			}

		}
		Vseparate = BehaviorController::KSeparation * total;
		if (Vseparate.Length() < 5.0) {
			Vseparate = 0.0;
		}
	}

	///ARRIVAL
	vec3 targetPos;
	if (actor != leader) {
		vec3 leaderforward_norm = ( m_pTarget->getLocalTranslation() - leader->getPosition() ).Normalize();

		vec3 distbehind = -200 * leaderforward_norm;
		targetPos = leader->getPosition() + distbehind;
	} else  {
		//leader
		targetPos = m_pTarget->getLocalTranslation();

	}

	Varrival = BehaviorController::KArrival * (targetPos - actorPos);
	Vdesired = CSeparation * Vseparate + CArrival * Varrival;


	return Vdesired;
}

///////////////////////////////////////////////////////////////////////////////

