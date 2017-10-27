#include "aActor.h"

#pragma warning(disable : 4018)



/****************************************************************
*
*    	    Actor functions
*
****************************************************************/

AActor::AActor() 
{
	mBVHController.setActor(this);
	mIKController.setActor(this);

	// code to update additional Actor data goes here

}

AActor::AActor(const AActor* actor)
{
	*this = *actor;
}

AActor& AActor::operator = (const AActor& actor)
{
	// Performs a deep copy
	if (&actor == this)
	{
		return *this;
	}

	mSkeleton = actor.mSkeleton;

	// code to update additional Actor data goes here


	return *this;
}

AActor::~AActor()
{
	clear();

}

void AActor::clear()
{
	mSkeleton.clear();

	// code to update additional Actor data goes here
}

void AActor::update()
{
	if (!mRoot)
		return; // Nothing loaded
	else mSkeleton.update();

	//if (!mRoot) return; // Nothing loaded
	//mRoot->updateTransform();

	// code to update additional Actor data goes here

}

ASkeleton* AActor::getSkeleton()
{
	return &mSkeleton;
}


BVHController* AActor::getBVHController()
{
	return &mBVHController;
}

IKController* AActor::getIKController()
{
	return &mIKController;
}
