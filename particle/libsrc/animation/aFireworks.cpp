// ParticleSystem.cpp: implementation of the AFireworks class.
//
//////////////////////////////////////////////////////////////////////

#include "aFireworks.h"
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <ctime>

#ifndef RAD
#define PI 3.14159265358979f
#define RAD (PI / 180.0f)
#endif
#ifndef GRAVITY
#define GRAVITY 9.8f
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

AFireworks::AFireworks()
{
    m_deltaT = 0.033f;
	m_rocketMass = 50.0;
	m_sparkMass = 1.0;

	m_attractorPos = vec3(0.0, 500.0, 0.0);
	m_repellerPos = vec3(0.0, 500.0, 0.0);
	m_windForce = vec3(250.0, 0.0, 0.0);

}

AFireworks::~AFireworks()
{

	for (unsigned int i = 0; i < sparks.size(); i++)
	{
		ASpark* pSpark = sparks[i];
		delete pSpark;
	}
	for (unsigned int i = 0; i < rockets.size(); i++)
	{
		ARocket* pRocket = rockets[i];
		delete pRocket;
	}

	rockets.clear();
	sparks.clear();

}

void AFireworks::display()
{

	for (unsigned int i = 0; i < sparks.size(); i++)
	{
		ASpark* pSpark = sparks[i];
		pSpark->display();
	}
	
	for (unsigned int i = 0; i < rockets.size(); i++)
	{
		ARocket* pRocket = rockets[i];
		pRocket->display();
	}

}

int AFireworks::getNumParticles()
{
	return sparks.size() + rockets.size();
}


void AFireworks::fireRocket(float posx, float* color)
	/*	fireRocket is called by AFireworksViewer::onKeyboard() function, when user presses the space key.
	*  Input:	float posx. X launch position of the rocket. Y launch position is always set to 0.
	*			float* color. color[0], color[1] and color[2] are the RGB color of this rocket.
	*                        It changes according to posx and it is provided for you.
	*  In this function, you want to generate a ARocket object and add it to the  rockets vector
	*  The initial state of the ARocket object is as follows:
	*      Position is posx and posy.
	*      launch angle is a random value between 80 and 110 degrees, assume the X axis is 0 degrees.
	*      Speed is a random number from 60 to 120.
	*      vertical force is mass * GRAVITY.
	*      rocket mass is 50.
	*      timeSpan (i.e. time to live) is 10.0
	*/
{
	float posy = 0.0f;
	ARocket *newRocket = new ARocket(color);

	// need to compute and set values for initial state of rocket  (including time to live)
	float stateVals[12] = { 0.0 };
	
	//TODO: Add your code here
	//srand(time(NULL));
	int speedmax = 120;
	int speedmin = 60;
	float speed = rand() % (speedmax - speedmin + 1) + speedmin;

	int anglemax = 110;
	int anglemin = 80;
	float angle = (rand() % (anglemax - anglemin + 1) + anglemin);	
	angle = angle * (PI / 180.f);

	float mass = 50;
	float ttl = 10;

	stateVals[0] = posx;
	stateVals[1] = posy;
	stateVals[2] = 0;

	stateVals[3] = speed*cos(angle);
	stateVals[4] = speed*sin(angle);
	stateVals[5] = 0;

	stateVals[6] = 0;
	stateVals[7] = mass*GRAVITY;
	stateVals[8] = 0;

	stateVals[9] = mass;
	stateVals[10] = ttl;

	newRocket->setState(stateVals);
	newRocket->setAttractor(m_attractorPos);
	newRocket->setRepeller(m_repellerPos);
	newRocket->setWind(m_windForce);
	rockets.push_back(newRocket);
}



void AFireworks::explode(float rocketPosx, float rocketPosy, float rocketPosz, 
	                     float rocketVelx, float rocketVely, float rocketVelz, float* rocketColor)

	/*	explode is called in AFireworks::update() when a rocket reaches its top height.
	*  It is called ARocket::TOTALEXPLOSIONS times to generate a series of rings of sparks.
	*  Input: float posx. X position where a ring of sparks are generated.
	*		   float posy. Y position where a ring of sparks are generated.
	*		   float posy. Z position where a ring of sparks are generated.
	*  Input: float velx. X velocity of rocket when sparks are generated.
	*		   float vely. Y velocity of rocket when sparks are generated.
	*		   float velz. Z velocity of rocket when sparks are generated.
	*         float* color. color[0], color[1] and color[2] are the RGB color of the rocket. It will also be the color of the sparks it generates.
	*
	*  In this function, you want to generate a number of sparks that are uniformily distributed on a ring at [posx, posy, posz]
	*  then append them to the sparks vector using a push_back function call. Since this is a 2D demo you can set posz = 0.0
	*  The initial state vector for each spark should accommodate the constraints below:
	*   They should be evenly distribute on a ring.
	*
	*  At the time of the explosion:
	*      the number of sparks generated should be based on a random number between 10 and 60.
	*      the position of the sparks is determined by [posx, posy, posz]
	*      the magnitude of the inital velocity of each spark should  be based on a random value between 20 and 40
	*      the direction of the initial velocity of each spark should be uniformly distributed between 0 and 360 degrees
	*      the total velocity of the spark at the time of its creation is the sum of the rocket velocity and the intial spark velocity
	*  force on every spark is just the gravity.
	*  spark mass is 50.
	*  Total timeToLive of every spark is 10.
	*/
		
	// The default mass of the rocket is 50.0, it should explode for a total of explosionCount = TOTALEXPLOSIONS time steps
{


	float stateVals[12] = { 0.0 };
	int numSparks = MAXSPARKS;
	float velocity = MAXVELOCITY;

	// TODO: Add  code here to randomize the number of sparks and their initial velocity
//	srand(time(NULL));
	int sparksmax = 60;
	int sparksmin = 10;
	numSparks = rand() % (sparksmax - sparksmin + 1) + sparksmin;

	int speedmax = 40;
	int speedmin = 20;
	float speed = rand() % (speedmax - speedmin + 1) + speedmin;

	float radstep = 2 * PI / numSparks;
	float mass = 1;
	float ttl = 10;

	//randforce
	vec3 randforce;
	int randmax = 10;
	int randmin = 0;

	for (int i = 0; i < numSparks; i++)
	{
		ASpark* newSpark = new ASpark(rocketColor);
		// TODO: Add your code here
		float angle = radstep * i;
		stateVals[0] = rocketPosx;
		stateVals[1] = rocketPosy;
		stateVals[2] = rocketPosz;

		stateVals[3] = speed*cos(angle);
		stateVals[4] = speed*sin(angle);
		stateVals[5] = 0;

		stateVals[6] = 0;
		stateVals[7] = mass*GRAVITY;
		stateVals[8] = 0;

		stateVals[9] = mass;
		stateVals[10] = ttl;

		randforce[0] = rand() % (randmax - randmin + 1) + randmin;
		randforce[1] = rand() % (randmax - randmin + 1) + randmin;
		randforce[2] = 0;

		newSpark->setState(stateVals);
		newSpark->setAttractor(m_attractorPos);
		newSpark->setRepeller(m_repellerPos);
		newSpark->setWind(m_windForce);
		newSpark->m_randForce = randforce;
		sparks.push_back(newSpark);
	}
}


// One simulation step 
void AFireworks::update(float deltaT, int extForceMode)
{
	//Step 1. Iterate through every ASpark in sparks. If the spark is dead(life is 0), erase it from sparks.
	//        Code is given. It is also an example of erasing an element from a vector.
	ASpark* pSpark;
	int index = 0;
	m_deltaT = deltaT;

	for (unsigned int i = 0; i < sparks.size(); i++)
	{
		pSpark = sparks[index];
		if (!pSpark->m_alive)
		{
			sparks.erase(sparks.begin() + index);
			delete pSpark;
		}
		else index++;
	}


	//Step 2. TODO: Iterate through every ARocket in rockets. If the rocket is dead, erase it from rockets.
	          //If the rocket explosionCount does not equal - 1 then explode another ring of sparks.
	
	ARocket* pRocket;	
	index = 0;

	// Add you code here
	for (unsigned int i = 0; i < rockets.size(); i++)
	{
		pRocket = rockets[index];
		if (pRocket->m_mode == EXPLOSION) {
			float color[3];
			color[0] = pRocket->m_color[0];
			color[1] = pRocket->m_color[1];
			color[2] = pRocket->m_color[2];
			explode(pRocket->m_state[0], pRocket->m_state[1], pRocket->m_state[2],
				pRocket->m_state[3], pRocket->m_state[4], pRocket->m_state[5],
				color);
			//rocket update will handle decrement of explosionCount
		}

		if (!pRocket->m_alive)
		{
			rockets.erase(rockets.begin() + index);
			delete pRocket;
		}
		else index++;
	}

	//Step 3. update valid sparks and rockets.
	//        Code is given.

	for (unsigned int i = 0; i < sparks.size(); i++)
	{
		ASpark* pSpark = sparks[i];
		pSpark->update(m_deltaT, extForceMode);
	}
	
	for (unsigned int i = 0; i < rockets.size(); i++)
	{
		ARocket* pRocket = rockets[i];
		pRocket->update(m_deltaT, extForceMode);
	}
}
