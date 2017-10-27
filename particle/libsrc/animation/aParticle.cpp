// AParticle.cpp: implementation of the AParticle class.
//
//////////////////////////////////////////////////////////////////////

#include "AParticle.h"

#ifndef GRAVITY
#define GRAVITY 9.8f
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

AParticle::AParticle()
{
	int dim = 12;
	m_dim = dim;

	m_state.resize(m_dim);
	m_stateDot.resize(m_dim);

	m_mass = 1.0;
	setMass(m_mass);

	m_alive = true;
	m_lifeSpan = 10.0;
	setLifeSpan(m_lifeSpan);

	m_gravity = vec3(0.0, -GRAVITY, 0.0);

 }

AParticle::~AParticle()
{

}

void AParticle::setState(vector<float>& newState)
{
	for (int i = 0; i < m_dim; i++)
		m_state[i] = newState[i];
	
	m_Pos[0] = m_state[0];
	m_Pos[1] = m_state[1]; 
	m_Pos[2] = m_state[2];

	m_Vel[0] = m_state[3];
	m_Vel[1] = m_state[4];
	m_Vel[2] = m_state[5];

}

void AParticle::setState(float *newState)
{
	for (int i = 0; i < m_dim; i++)
		m_state[i] = newState[i];
	
	m_Pos[0] = m_state[0];
	m_Pos[1] = m_state[1];
	m_Pos[2] = m_state[2];

	m_Vel[0] = m_state[3];
	m_Vel[1] = m_state[4];
	m_Vel[2] = m_state[5];
}
vector<float> AParticle::getState()
{
	return m_state;
}

vector<float> AParticle::getStateDot()
{
	return m_stateDot;
}

//Get the state vector dimension (multiples of 3)
int AParticle::getDim()
{
	return m_dim;
}

//Set the state vector dimension (multiples of 3)
void AParticle::setDim(int dim)
{
	m_dim = dim;
}


//Set mass
void AParticle::setMass(float mass)
{
	m_state[9] = mass;
	m_mass = mass;
}

//Get mass
float AParticle::getMass()
{
	return m_state[9];
}

bool AParticle::isAlive()
{
	if (m_state[10] <= 0.0)
		m_alive = false;

	return m_alive;
}

void AParticle::setAlive()
{
	m_state[10] = m_lifeSpan;
	m_alive = true;

}

// kills particle and sets time to live to 0
void AParticle::kill()
{
	m_state[10] = 0.0;
	m_alive = false;

}

//Get time to live
float AParticle::getTimeToLive()
{
	return m_state[10];
}

//Set time to live
void AParticle::setLifeSpan(float time)
{
	m_lifeSpan = time;
	m_state[10] = time;
}

void AParticle::addForce(vec3 force)
{
	m_state[6] += force[0];
	m_state[7] += force[1];
	m_state[8] += force[2];
}

void AParticle::computeForces(int mode)
{
	// zero out all forces
	m_state[6] = 0.0;
	m_state[7] = 0.0;
	m_state[8] = 0.0;


	// default is gravity force
	addForce(m_mass*m_gravity);


}

void AParticle::computeDynamics(vector<float>& state, vector<float>& stateDot, float deltaT)
{
	//TODO: Add your code here
	//compute m_stateDot vdot and xdot where f=ma is vdot and xdot is v from m_state 
	m_stateDot[0] = m_state[3];
	m_stateDot[1] = m_state[4];
	m_stateDot[2] = m_state[5];
	m_stateDot[3] = m_state[6] / m_state[9];//just in case we implement mass changes or force changes over time
	m_stateDot[4] = m_state[7] / m_state[9];
	m_stateDot[5] = m_state[8] / m_state[9];
	m_stateDot[6] = m_state[6]; //for now, no changes in force over time
	m_stateDot[7] = m_state[7];
	m_stateDot[8] = m_state[8];
	m_stateDot[9] = m_state[9];//for now, no changes in mass over time;
	m_stateDot[10] = m_state[10];//for now, no changes in TTL as a function of something
}

void AParticle::updateState(float deltaT, int integratorType)
{

 	computeDynamics(m_state, m_stateDot, deltaT);

	//TODO:  Add your code here to update the state using EULER and Runge Kutta2  integration
	switch (integratorType)
	{
		case EULER:
			// Add your code here
			m_state[0] = m_state[0] + deltaT * m_stateDot[0];
			m_state[1] = m_state[1] + deltaT * m_stateDot[1];
			m_state[2] = m_state[2] + deltaT * m_stateDot[2];
			m_state[3] = m_state[3] + deltaT * m_stateDot[3];
			m_state[4] = m_state[4] + deltaT * m_stateDot[4];
			m_state[5] = m_state[5] + deltaT * m_stateDot[5];
			m_state[10] -= deltaT;
			//for now, force, mass remain unchanged
			break;
		case RK2:
		{
			// Add your code here
			vec3 velNext;
			vec3 accNext;
			velNext[0] = m_state[3] + deltaT * m_stateDot[3];
			velNext[1] = m_state[4] + deltaT * m_stateDot[4];
			velNext[2] = m_state[5] + deltaT * m_stateDot[5];
			accNext[0] = m_stateDot[6] / m_stateDot[9];//just in case force or mass changes are implemented, for now they are constant
			accNext[1] = m_stateDot[7] / m_stateDot[9];
			accNext[2] = m_stateDot[8] / m_stateDot[9];

			m_state[0] = m_state[0] + deltaT * 0.5 * (m_stateDot[0] + velNext[0]);
			m_state[1] = m_state[1] + deltaT * 0.5 * (m_stateDot[1] + velNext[1]);
			m_state[2] = m_state[2] + deltaT * 0.5 * (m_stateDot[2] + velNext[2]);
			m_state[3] = m_state[3] + deltaT * 0.5 * (m_stateDot[3] + accNext[0]);
			m_state[4] = m_state[4] + deltaT * 0.5 * (m_stateDot[4] + accNext[1]);
			m_state[5] = m_state[5] + deltaT * 0.5 * (m_stateDot[5] + accNext[2]);
			
			m_state[10] -= deltaT;
			break;
		}
	}
	m_Pos[0] = m_state[0];
	m_Pos[1] = m_state[1];
	m_Pos[2] = m_state[2];

	m_Vel[0] = m_state[3];
	m_Vel[1] = m_state[4];
	m_Vel[2] = m_state[5];
}

void AParticle::update(float deltaT, int forceMode)
{
	m_deltaT = deltaT;
	if (m_state[10] <= 0.0)
	{
		m_alive = false;
		return;
	}
	computeForces(forceMode);
    updateState(deltaT, EULER);
	
}

void AParticle::initialize(AParticleSystem& parent)
{
	mStartAlpha = parent.mStartAlpha;
	mEndAlpha = parent.mEndAlpha;
	mAlpha = mStartAlpha;

	mStartColor = parent.mStartColor + AJitterVec(parent.mColorJitter);
	mEndColor = parent.mEndColor + AJitterVec(parent.mColorJitter);
	mColor = mStartColor;

	mStartScale = parent.mStartScale + AJitterVal(parent.mScaleJitter);
	mEndScale = parent.mEndScale + AJitterVal(parent.mScaleJitter);
	mScale = mStartScale;

	m_Pos = parent.mStartPos + AJitterVec(parent.mPositionJitter);
	m_Vel = parent.mStartVel + AJitterVec(parent.mVelocityJitter);

	m_gravity = parent.mGravity;

	m_state[0] = m_Pos[0];
	m_state[1] = m_Pos[1];
	m_state[2] = m_Pos[2];
	m_state[3] = m_Vel[0];
	m_state[4] = m_Vel[1];
	m_state[5] = m_Vel[2];
	m_state[6] = m_mass * m_gravity[0];
	m_state[7] = m_mass * m_gravity[1];
	m_state[8] = m_mass * m_gravity[2];
	m_state[9] = m_mass;
	m_state[10] = m_lifeSpan;

	m_alive = true;
}
