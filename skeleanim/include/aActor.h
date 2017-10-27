#ifndef Actor_H_
#define Actor_H_

#pragma once

#include "aTransform.h"

#include "aJoint.h"
#include "aSkeleton.h"
#include "aBVHController.h"
#include "aIKController.h"

class AActor : ASkeleton
{

public:
	AActor();
	AActor(const AActor* actor); 
	virtual ~AActor();

	virtual AActor& operator=(const AActor& actor); 
	void clear();
	void update();


	ASkeleton* getSkeleton();
	BVHController* getBVHController();
	IKController* getIKController();

protected:
	// the actor owns the skeleton and controllers
	ASkeleton mSkeleton;
	BVHController mBVHController;
	IKController mIKController;


};

#endif