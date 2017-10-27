#include "aBehaviorController.h"

#include "aVector.h"
#include "aRotation.h"
#include <Windows.h>
#include <algorithm>

#include "GL/glew.h"
#include "GL/glut.h"



#define Truncate(a, b, c) (a = max<double>(min<double>(a,c),b))

double BehaviorController::gMaxSpeed = 1000.0; 
double BehaviorController::gMaxAngularSpeed = 200.0;  
double BehaviorController::gMaxForce = 2000.0;  
double BehaviorController::gMaxTorque = 2000.0;
double BehaviorController::gKNeighborhood = 500.0;   
//Tsettle_V = 0.4s, Tsettle_Ori = 0.25, zeta = 1(no osc),
//Tsettle = 4Tc, Tc = 1/(zeta*naturalfreq) ,  natural freq(Wn) = 10 for VelKv, 16 for OriKv
//Kv = 2*zeta*Wn (a1), Kp = Wn squared (a0)
double BehaviorController::gOriKv = 32.0;    
double BehaviorController::gOriKp = 256.0;  
double BehaviorController::gVelKv = 20.0;    

double BehaviorController::gAgentRadius = 80.0;  
double BehaviorController::gMass = 1;
double BehaviorController::gInertia = 1;
double BehaviorController::KArrival = 1.0; 
double BehaviorController::KDeparture = 12000.0;
double BehaviorController::KNoise = 15.0;
double BehaviorController::KWander = 80.0;   
double BehaviorController::KAvoid = 600.0;  
double BehaviorController::TAvoid = 1000.0;   
double BehaviorController::RNeighborhood = 100000;
double BehaviorController::KSeparation = 12000.0; 
double BehaviorController::KAlignment = 1.0;  
double BehaviorController::KCohesion = 1.0;  

const double M2_PI = M_PI * 2.0;

BehaviorController::BehaviorController() 
{
	m_state.resize(m_stateDim);
	m_stateDot.resize(m_stateDim);
	m_controlInput.resize(m_controlDim);

	vec3 m_Pos0 = vec3(0, 0, 0);
	vec3 m_Vel0 = vec3(0, 0, 0);
	vec3 m_lastVel0 = vec3(0, 0, 0);
	vec3 m_Euler = vec3(0, 0, 0);
	vec3 m_VelB = vec3(0, 0, 0);
	vec3 m_AVelB = vec3(0, 0, 0);
	
	m_Vdesired = vec3(0, 0, 0);
	m_lastThetad = 0.0;

	m_Active = true; 
	mpActiveBehavior = NULL;
	mLeader = false;

	reset();
}

AActor* BehaviorController::getActor()
{
	return m_pActor;
}

void BehaviorController::setActor(AActor* actor)

{
	m_pActor = actor;
	m_pSkeleton = m_pActor->getSkeleton();

}


void BehaviorController::createBehaviors(vector<AActor>& agentList, vector<Obstacle>& obstacleList)
{
	
	m_AgentList = &agentList;
	m_ObstacleList = &obstacleList;

	m_BehaviorList.clear();
	m_BehaviorList[SEEK] = new Seek(m_pBehaviorTarget);
	m_BehaviorList[FLEE] = new Flee(m_pBehaviorTarget);
	m_BehaviorList[ARRIVAL] = new Arrival(m_pBehaviorTarget);
	m_BehaviorList[DEPARTURE] = new Departure(m_pBehaviorTarget);
	m_BehaviorList[WANDER] = new Wander();
	m_BehaviorList[COHESION] = new Cohesion(m_AgentList);
	m_BehaviorList[ALIGNMENT] = new Alignment(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[SEPARATION] = new Separation(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[LEADER] = new Leader(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[FLOCKING] = new Flocking(m_pBehaviorTarget, m_AgentList);
	m_BehaviorList[AVOID] = new Avoid(m_pBehaviorTarget, m_ObstacleList);
}

BehaviorController::~BehaviorController()
{
	mpActiveBehavior = NULL;
}

void BehaviorController::reset()
{
	vec3 startPos;
	startPos[0] = ((double)rand()) / RAND_MAX;
	startPos[1] =  ((double)rand()) / RAND_MAX, 
	startPos[2] = ((double)rand()) / RAND_MAX;
	startPos = startPos - vec3(0.5, 0.5, 0.5);

	startPos[1] = 0; // set equal to zero for 2D case (assume y is up)

	m_Guide.setLocalTranslation(startPos * 500.0);
	
	for (int i = 0; i < m_stateDim; i++)
	{
		m_state[i] = 0.0;
		m_stateDot[i] = 0.0;
	}

	m_force = 0.0;
	m_torque = 0.0;
	m_thetad = 0.0;
	m_vd = 0.0;
}

///////////////////////////////////////////////////

inline void ClampAngle(double& angle)
{
	while (angle > M_PI)
	{
		angle -= M2_PI;
	}
	while (angle < -M_PI)
	{
		angle += M2_PI;
	}
}

void BehaviorController::sense(double deltaT)
{
	if (mpActiveBehavior)
	{
		// TODO: find the agents in the neighborhood of the current character.
	}
	
}

void BehaviorController::control(double deltaT)
// Given the active behavior this function calculates a desired velocity vector (Vdesired).  
// The desired velocity vector is then used to compute the desired speed (vd) and direction (thetad) commands

{

	if (mpActiveBehavior)
	{
		m_Vdesired = mpActiveBehavior->calcDesiredVel(this);
		m_Vdesired[1] = 0;

		//  force and torque inputs are computed from vd and thetad as follows:
		//              Velocity P controller : force = mass * Kv * (vd - v)
		//              Heading PD controller : torque = Inertia * (-Kv * thetaDot -Kp * (thetad - theta))
		//  where the values of the gains Kv and Kp are different for each controller

		///TEST
		//m_Vdesired = vec3(10, 0, 0);
		// TODO: insert your code here to compute m_force and m_torque
		m_vd = m_Vdesired.Length();
		m_force[_Z] = gMass * gVelKv * (m_vd - m_state[VEL][_Z]);
		//std::cout << "\nFORCE BODY: " << m_force[_Z];
		//std::cout << "  m_vd: " << m_vd;
		//std::cout << "  VEL BODY: " << m_state[VEL][_Z];
		//std::cout << "  POS: " << m_state[POS];

		//convert vdesired to body axes(since it's in world) by rotation or projection
		mat3 world2body = mat3::Rotation3D(1, m_state[ORI][_Y]);
		world2body = world2body.Inverse();

		vec3 vdesired_body = world2body * m_Vdesired;
		m_thetad = m_state[ORI][_Y] + atan2(vdesired_body[_Z], vdesired_body[_X]) - M_PI_2;
		m_torque[_Y] = gInertia * (-gOriKv * m_state[AVEL][_Y] - gOriKp * (m_thetad - m_state[ORI][_Y]));
		//std::cout << "   TORQUE BODY: " << m_torque[_Y];
		//std::cout << "   thetad: " << m_thetad;
		//std::cout << "   ORI: " << m_state[ORI][_Y];
		//std::cout << "   AVEL: " << m_state[ORI][_Y];
		
		
			
		// when agent desired agent velocity and actual velocity < 2.0 then stop moving
		if (m_vd < 2.0 &&  m_state[VEL][_Z] < 2.0)
		{
			m_force[2] = 0.0;
			m_torque[1] = 0.0;
		}
	}
	else
	{
		m_force[2] = 0.0;
		m_torque[1] = 0.0;
	}

	// set control inputs to current force and torque values
	m_controlInput[0] = m_force;
	m_controlInput[1] = m_torque;
}

void BehaviorController::act(double deltaT)
{
	computeDynamics(m_state, m_controlInput, m_stateDot, deltaT);
	
	int EULER = 0;
	int RK2 = 1;
	updateState(deltaT, EULER);
}


void BehaviorController::computeDynamics(vector<vec3>& state, vector<vec3>& controlInput, vector<vec3>& stateDot, double deltaT)
// Compute stateDot vector given the control input and state vectors
//  This function sets derive vector to appropriate values after being called
{
	// Compute the stateDot vector given the values of the current state vector and control input vector
	// TODO: add your code here
	//f=m*a
	//f_zbody = m * dV_zbody
	//toa=I*angAccel
	//toa_ybody = Iyy * angVel_ybody

	// m_state[0] = m_Pos0 = [x 0 z]T for the 2D planar case
	// m_state[1] = m_Euler = [ 0 theta 0]T for the 2D planar case 
	// m_state[2] = m_VelB = [ Vx 0 Vz]T for the 2D planar case
	// m_state[3] = m_AVelB =  [ 0 thetaDot 0]T for the 2D planar case 

	// m_stateDot[0] = m_Vel0 = [ Vx 0 Vz]T for the 2D planar case
	// m_stateDot[1] = m_AVelB = [ 0 thetaDot 0]T for the 2D planar case
	// m_stateDot[2] = body acceleration = [ accelx 0 accelz]T for the 2D planar case
	// m_stateDot[3] = body angular acceleration = = [ 0 thetaDot2 0]T for the 2D planar case

	// m_force[0] = body force in the x direction
	// m_force[1] = body force in the y direction (for the 2D planar case = 0.0)
	// m_force[2] = body force in the z direction

	// m_torque[0] = body torque about the x axis (for 2D planar case = 0.0)
	// m_torque[1] = body torque about the y axis 
	// m_torque[2] = body torque about the z axis (for 2D planar case = 0.0)

	// m_controlInput[0] = m_force = [force_xbody 0 force_zbody]T for the 2D planar case
	// m_controlInput[1] = m_torque = [0 torque_ybody 0]T for the 2D planar case
	
	//finding vel0
	float ybody2world_theta = state[ORI][_Y];//is this degress or rads?
	mat3 bodytoworld = mat3::Rotation3D(1, ybody2world_theta);//1 means y axis,2nd arg takes rads
	vec3 vel0 = bodytoworld * state[VEL];//0 is world
	stateDot[0] = vel0;

	//finding ang vel body
	stateDot[1][_Y] = state[AVEL][_Y];
	//std::cout << "\ncomputeDynamics angVel: " << stateDot[1][_Y];

	//finding body accel
	float bodyforce = controlInput[0][_Z];
	float mass = gMass;
	float accel = bodyforce / mass;
	stateDot[2][_Z] = accel;
	//std::cout << "   accel: " << stateDot[2][_Z];

	//fining body ang accel
	float bodytorque = controlInput[1][_Y];
	float Iyy = gInertia;
	float angAccel = bodytorque / Iyy;
	stateDot[3][_Y] = angAccel;
	//std::cout << "   angAccel: " << m_stateDot[3][_Y];
}

void BehaviorController::updateState(float deltaT, int integratorType)
{
	//  Update the state vector given the m_stateDot vector using Euler (integratorType = 0) or RK2 (integratorType = 1) integratio
	//  this should be similar to what you implemented in the particle system assignment
	const int EULER = 0;
	const int RK2 = 1;
	// m_state[0] = m_Pos0 = [x 0 z]T for the 2D planar case
	// m_state[1] = m_Euler = [ 0 theta 0]T for the 2D planar case 
	// m_state[2] = m_VelB = [ Vx 0 Vz]T for the 2D planar case
	// m_state[3] = m_AVelB =  [ 0 thetaDot 0]T for the 2D planar case 

	// m_stateDot[0] = m_Vel0 = [ Vx 0 Vz]T for the 2D planar case
	// m_stateDot[1] = m_AVelB = [ 0 thetaDot 0]T for the 2D planar case
	// m_stateDot[2] = body acceleration = [ accelx 0 accelz]T for the 2D planar case
	// m_stateDot[3] = body angular acceleration = = [ 0 thetaDot2 0]T for the 2D planar case
	// TODO: add your code here
	std::vector<vec3> nextState(4);
	std::vector<vec3> nextStateDot(4);
	switch (integratorType)
	{
	case EULER:
		m_state[POS] = m_state[POS] + deltaT * m_stateDot[0];
		m_state[ORI] = m_state[ORI] + deltaT * m_stateDot[1];
		m_state[VEL] = m_state[VEL] + deltaT * m_stateDot[2];
		m_state[AVEL] = m_state[AVEL] + deltaT * m_stateDot[3];
		break;
	case RK2:
		//nextState
		nextState[0] = m_state[0] + deltaT * m_stateDot[0];
		nextState[1] = m_state[1] + deltaT * m_stateDot[1];
		nextState[2] = m_state[2] + deltaT * m_stateDot[2];
		nextState[3] = m_state[3] + deltaT * m_stateDot[3];

		//nextStateDot
		computeDynamics(nextState, m_controlInput, nextStateDot, deltaT);

		m_state[0] = m_state[0] + deltaT * 0.5f * (m_stateDot[0] + nextStateDot[0]);
		m_state[1] = m_state[1] + deltaT * 0.5f * (m_stateDot[1] + nextStateDot[1]);
		m_state[2] = m_state[2] + deltaT * 0.5f * (m_stateDot[2] + nextStateDot[2]);
		m_state[3] = m_state[3] + deltaT * 0.5f * (m_stateDot[3] + nextStateDot[3]);
		break;
	default:
		break;
	}


	//  given the new values in m_state, these are the new component state values 
	m_Pos0 = m_state[POS];
	m_Euler = m_state[ORI];
	m_VelB = m_state[VEL];
	m_AVelB = m_state[AVEL];
	m_Vel0 = m_stateDot[0];

	//  Perform validation check to make sure all values are within MAX values
	// TODO: add your code here
	if (m_state[VEL][_Z] > gMaxSpeed) {
		m_state[VEL][_Z] = gMaxSpeed;
	}

	if (m_state[AVEL][_Y] > gMaxAngularSpeed) {
		m_state[AVEL][_Y] = gMaxAngularSpeed;
	}

	if (m_controlInput[0][_Z] > gMaxForce) {
		m_controlInput[0][_Z] = gMaxForce;
	}

	if (m_controlInput[1][_Y] > gMaxTorque) {
		m_controlInput[1][_Y] = gMaxTorque;
	}

	// update the guide orientation
	// compute direction from nonzero velocity vector
	vec3 dir;
	if (m_Vel0.Length() < 1.0)
	{
		dir = m_lastVel0;
		dir.Normalize();;
		m_state[ORI] = atan2(dir[_Z], dir[_X]);
	}
	else
	{
		dir = m_Vel0;
		m_lastVel0 = m_Vel0;
	}

	dir.Normalize();
	vec3 up(0.0, 1.0, 0.0);
	vec3 right = up.Cross(dir);
	right.Normalize();
	mat3 rot(right, up, dir);
	m_Guide.setLocalRotation(rot.Transpose());
	m_Guide.setLocalTranslation(m_Guide.getLocalTranslation() + m_Vel0*deltaT);

}


void BehaviorController::setTarget(AJoint& target)
{
	m_pBehaviorTarget = &target;
	for (unsigned int i = 0; i < m_BehaviorList.size(); i++)
	{
		BehaviorType index = (BehaviorType) i;
		m_BehaviorList[index]->setTarget(m_pBehaviorTarget);
	}
}

void BehaviorController::setActiveBehavior(Behavior* pBehavior)
{
	mpActiveBehavior = pBehavior;
}

void BehaviorController::setActiveBehaviorType(BehaviorType type)
{
	m_BehaviorType = type;
	Behavior* pActiveBehavior = m_BehaviorList[type];
	setActiveBehavior(pActiveBehavior);

}

void BehaviorController::display()
{ // helps with debugging behaviors.  red line is actual velocity vector, green line is desired velocity vector
	
	vec3 pos = getPosition();
	double scale = 1.0;
	vec3 vel = scale* getVelocity();
	double velMag = vel.Length();
	vec3 dvel = scale* getDesiredVelocity();
	vec3 angle = getOrientation() * (180.0 / 3.14159);

	glBegin(GL_LINES);
	glColor3f(1, 0, 0);
	glVertex3f(pos[0], pos[1], pos[2]);
	glVertex3f(pos[0] + vel[0], pos[1] + vel[1], pos[2] + vel[2]);
	glColor3f(0, 1, 0);
	glVertex3f(pos[0], pos[1], pos[2]);
	glVertex3f(pos[0] + dvel[0], pos[1] + dvel[1], pos[2] + dvel[2]);
	glEnd();

	if (this->isLeader())
		glColor3f(0, 0, 1);
	else glColor3f(0.5, 0, 0);

	glPushMatrix();
	glTranslatef(pos[0], pos[1], pos[2]);
	glRotatef(angle[1], 0, 1, 0);
	glutSolidCone(40, 80, 10, 10);
	glutSolidSphere(35, 10, 10);
	glPopMatrix();

	BehaviorType active = getActiveBehaviorType();
	Behavior* pBehavior = m_BehaviorList[active];
	pBehavior->display(this);

}

