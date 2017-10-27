#include "aIKController.h"
#include "GL/glut.h"

#include "aActor.h"

#pragma warning (disable : 4018)

int IKController::gIKmaxIterations = 5;
double IKController::gIKEpsilon = 0.1;

// AIKchain class functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////
AIKchain::AIKchain()
{
	mWeight0 = 0.1;
}

AIKchain::~AIKchain()
{

}

AJoint* AIKchain::getJoint(int index)
{
	return mChain[index];
}

void AIKchain::setJoint(int index, AJoint* pJoint)
{
	mChain[index] = pJoint;
}

double AIKchain::getWeight(int index)
{
	return mWeights[index];
}

void AIKchain::setWeight(int index, double weight)
{
	mWeights[index] = weight;
}

int AIKchain::getSize()
{
	return mChain.size();
}

std::vector<AJoint*>& AIKchain::getChain()
{
	return mChain;
}

std::vector<double>& AIKchain::getWeights()
{
	return mWeights;
}

void AIKchain::setChain(std::vector<AJoint*> chain)
{
	mChain = chain;
}

void AIKchain::setWeights(std::vector<double> weights)
{
	mWeights = weights;
}

// AIKController class functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////

IKController::IKController()
{
	m_pActor = NULL;
	m_pSkeleton = NULL;
	mvalidLimbIKchains = false;
	mvalidCCDIKchains = false;

	// Limb IK
	m_pEndJoint = NULL;
	m_pMiddleJoint = NULL;
	m_pBaseJoint = NULL;
	m_rotationAxis = vec3(0.0, 1.0, 0.0);

	ATransform desiredTarget = ATransform();
	mTarget0.setLocal2Parent(desiredTarget);  // target associated with end joint
	mTarget1.setLocal2Parent(desiredTarget);  // optional target associated with middle joint - used to specify rotation of middle joint about end/base axis
	mTarget0.setLocal2Global(desiredTarget);
	mTarget1.setLocal2Global(desiredTarget);

	//CCD IK
	mWeight0 = 0.1;  // default joint rotation weight value

}

IKController::~IKController()
{
}

ASkeleton* IKController::getSkeleton()
{
	return m_pSkeleton;
}

const ASkeleton* IKController::getSkeleton() const
{
	return m_pSkeleton;
}

ASkeleton* IKController::getIKSkeleton()
{
	return &mIKSkeleton;
}

const ASkeleton* IKController::getIKSkeleton() const
{
	return &mIKSkeleton;
}

AActor* IKController::getActor()
{
	return m_pActor;
}

void IKController::setActor(AActor* actor)

{
	m_pActor = actor;
	m_pSkeleton = m_pActor->getSkeleton();
}


AIKchain IKController::createIKchain(int endJointID, int desiredChainSize, ASkeleton* pSkeleton)
{
	// TODO: given the end joint ID and the desired size (i.e. length) of the IK chain, 
	// 1. add the corresponding skeleton joint pointers to the AIKChain "chain" vector data member starting with the end joint
	// 2. also add weight values to the associated AIKChain "weights" vector data member for use in the CCD IK implemention
	// Note: desiredChainSize = -1 should create an IK chain of maximum length (i.e. where the last chain joint is the joint before the root joint)
	bool getMaxSize = false;

	int EndJointID = endJointID;
	std::vector<AJoint*> chain;
	std::vector<double> weights;

	chain.clear();
	weights.clear();
	if (desiredChainSize == -1)
		getMaxSize = true;

	if ((EndJointID >= 0) && (EndJointID < pSkeleton->getNumJoints()))
	{
		AJoint* pJoint = pSkeleton->getJointByID(endJointID);
		// TODO: add code here to generate chain of desired size or terminate at the joint before root joint, so that root will not change during IK	
		// also add weight values to corresponding weights vector  (default value = 0.1)
		AJoint* pRoot = pSkeleton->getRootNode();
		int count = 0;
		while (pJoint != pRoot || count < desiredChainSize) {//desired size isnt getting in to this func, just OR it for now to bypass that check
			chain.push_back(pJoint);
			weights.push_back(mWeight0);
			pJoint = pJoint->getParent();
			count++;
		}
	}
	AIKchain result;
	result.setChain(chain);
	result.setWeights(weights);

	return result;
}



bool IKController::IKSolver_Limb(int endJointID, const ATarget& target)
{
	// Implements the analytic/geometric IK method assuming a three joint limb  

	if (!mvalidLimbIKchains)
	{
		mvalidLimbIKchains = createLimbIKchains();
		//assert(mvalidLimbIKchains);
	}

	// copy transforms from base skeleton
	mIKSkeleton.copyTransforms(m_pSkeleton);

	vec3 desiredRootPosition;

	switch (endJointID)
	{
	case mLhandID:
		mLhandTarget = target;
		computeLimbIK(mLhandTarget, mLhandIKchain, -axisY, &mIKSkeleton);
		break;
	case mRhandID:
		mRhandTarget = target;
		computeLimbIK(mRhandTarget, mRhandIKchain, axisY, &mIKSkeleton);
		break;
	case mLfootID:
		mLfootTarget = target;
		computeLimbIK(mLfootTarget, mLfootIKchain, axisX, &mIKSkeleton);
		break;
	case mRfootID:
		mRfootTarget = target;
		computeLimbIK(mRfootTarget, mRfootIKchain, axisX, &mIKSkeleton);
		break;
	case mRootID:
		desiredRootPosition = target.getGlobalTranslation();
		mIKSkeleton.getJointByID(mRootID)->setLocalTranslation(desiredRootPosition);
		mIKSkeleton.update();
		computeLimbIK(mLhandTarget, mLhandIKchain, -axisY, &mIKSkeleton);
		computeLimbIK(mRhandTarget, mRhandIKchain, axisY, &mIKSkeleton);
		computeLimbIK(mLfootTarget, mLfootIKchain, axisX, &mIKSkeleton);
		computeLimbIK(mRfootTarget, mRfootIKchain, axisX, &mIKSkeleton);
		break;
	default:
		mIKchain = createIKchain(endJointID, 3, &mIKSkeleton);
		computeLimbIK(target, mIKchain, axisY, &mIKSkeleton);
		break;
	}

	// update IK Skeleton transforms
	mIKSkeleton.update();

	// copy IK skeleton transforms to main skeleton
	m_pSkeleton->copyTransforms(&mIKSkeleton);

	return true;
}



int IKController::createLimbIKchains()
{
	bool validChains = false;
	int desiredChainSize = 3;

	// create IK chains for Lhand, Rhand, Lfoot and Rfoot 
	mLhandIKchain = createIKchain(mLhandID, desiredChainSize, &mIKSkeleton);
	mRhandIKchain = createIKchain(mRhandID, desiredChainSize, &mIKSkeleton);
	mLfootIKchain = createIKchain(mLfootID, desiredChainSize, &mIKSkeleton);
	mRfootIKchain = createIKchain(mRfootID, desiredChainSize, &mIKSkeleton);

	if (mLhandIKchain.getSize() == 3 && mRhandIKchain.getSize() == 3 && mLfootIKchain.getSize() == 3 && mRfootIKchain.getSize() == 3)
	{
		validChains = true;

		// initalize end joint target transforms for Lhand, Rhand, Lfoot and Rfoot based on current position and orientation of joints
		mIKSkeleton.copyTransforms(m_pSkeleton);
		mLhandTarget.setLocal2Global(mIKSkeleton.getJointByID(mLhandID)->getLocal2Global());
		mRhandTarget.setLocal2Global(mIKSkeleton.getJointByID(mRhandID)->getLocal2Global());
		mLfootTarget.setLocal2Global(mIKSkeleton.getJointByID(mLfootID)->getLocal2Global());
		mRfootTarget.setLocal2Global(mIKSkeleton.getJointByID(mRfootID)->getLocal2Global());
	}

	return validChains;
}




int IKController::computeLimbIK(ATarget target, AIKchain& IKchain, const vec3 midJointAxis, ASkeleton* pIKSkeleton)
{
	// TODO: Implement the analytic/geometric IK method assuming a three joint limb  
	// The actual position of the end joint should match the target position within some episilon error 
	// the variable "midJointAxis" contains the rotation axis for the middle joint

	bool result = false;
	int outofrange = 0;
	int endJointID;
	mTarget0 = target;

	if (IKchain.getSize() > 0)
		endJointID = IKchain.getJoint(0)->getID();
	else endJointID = -1;

	if ((endJointID >= 0) && (endJointID < pIKSkeleton->getNumJoints()))
	{
		bool result = true;
		m_pEndJoint = IKchain.getJoint(0);
		m_pMiddleJoint = IKchain.getJoint(1);
		m_pBaseJoint = IKchain.getJoint(2);

		//TODO:
		// 1. compute error vector between target and end joint
		vec3 target_pos = target.getGlobalTranslation();
		vec3 endjoint_pos = m_pEndJoint->getGlobalTranslation();
		vec3 global_errvec = target_pos - endjoint_pos;

		// 2. compute vector between end Joint and base joint
		vec3 basejoint_pos = m_pBaseJoint->getGlobalTranslation();
		vec3 base2end_vec = endjoint_pos - basejoint_pos;

		// 3. compute vector between target and base joint
		vec3 base2target_vec = target_pos - basejoint_pos;

		// 4. Compute desired angle for middle joint 
		float L2 = m_pEndJoint->getLocalTranslation().Length();
		float L1 = m_pMiddleJoint->getLocalTranslation().Length();
		float Rd = base2target_vec.Length();
		float lawcos_ratio = (L1*L1 + L2*L2 - Rd*Rd) / (2 * L1 * L2);
		if (lawcos_ratio > 1.f) {
			lawcos_ratio = 1.f;
		} else if (lawcos_ratio < -1.f) {
			lawcos_ratio = -1.f;
		}
		if (Rd > (L1 + L2)) {
			outofrange = 1;
		}

		float phi = acos(lawcos_ratio);
		float thetamid = -(phi - M_PI);

		// 5. given desired angle and midJointAxis, compute new local middle joint rotation matrix and update joint transform
		mat3 newmid_rot;
		newmid_rot = newmid_rot.Rotation3D(midJointAxis, thetamid);
		m_pMiddleJoint->setLocalRotation(newmid_rot);
		m_pMiddleJoint->updateTransform();

		// 6. compute vector between target and base joint
		endjoint_pos = m_pEndJoint->getGlobalTranslation();
		base2end_vec = endjoint_pos - basejoint_pos;

		// 7. Compute base joint rotation axis (in global coords) and desired angle
		vec3 basetarget_cross = base2end_vec.Cross(base2target_vec);
		//when change in theta is tiny sin(theta) is approx theta
		float rotaxis_theta = basetarget_cross.Length() / ( base2end_vec.Length() * base2target_vec.Length() ) ;
		vec3 rotaxis = basetarget_cross / basetarget_cross.Length();

		// 8. transform base joint rotation axis to local coordinates
		mat3 baserot_global2local = m_pBaseJoint->getGlobalRotation().Inverse();
		vec3 rotaxis_local = baserot_global2local * rotaxis;

		// 9. given desired angle and local rotation axis, compute new local rotation matrix and update base joint transform
		quat q;
		q.FromAxisAngle(rotaxis_local, rotaxis_theta);
		mat3 rotmat = q.ToRotation();
		if (abs(rotaxis_theta) < FLT_EPSILON) {
			rotmat.Identity();
		}
		mat3 newlocalrotmat = m_pBaseJoint->getLocalRotation() * rotmat;
		m_pBaseJoint->setLocalRotation(newlocalrotmat);
		m_pBaseJoint->updateTransform();
	}
	return outofrange;

}

bool IKController::IKSolver_CCD(int endJointID, const ATarget& target)
{
	// Implements the CCD IK method assuming a three joint limb 

	bool validChains = false;

	if (!mvalidCCDIKchains)
	{
		mvalidCCDIKchains = createCCDIKchains();
		//assert(mvalidCCDIKchains);
	}

	// copy transforms from base skeleton
	mIKSkeleton.copyTransforms(m_pSkeleton);

	vec3 desiredRootPosition;

	switch (endJointID)
	{
	case mLhandID:
		mLhandTarget = target;
		computeCCDIK(mLhandTarget, mLhandIKchain, &mIKSkeleton);
		break;
	case mRhandID:
		mRhandTarget = target;
		computeCCDIK(mRhandTarget, mRhandIKchain, &mIKSkeleton);
		break;
	case mLfootID:
		mLfootTarget = target;
		computeCCDIK(mLfootTarget, mLfootIKchain, &mIKSkeleton);
		break;
	case mRfootID:
		mRfootTarget = target;
		computeCCDIK(mRfootTarget, mRfootIKchain, &mIKSkeleton);
		break;
	case mRootID:
		desiredRootPosition = target.getGlobalTranslation();
		mIKSkeleton.getJointByID(mRootID)->setLocalTranslation(desiredRootPosition);
		mIKSkeleton.update();
		computeCCDIK(mLhandTarget, mLhandIKchain, &mIKSkeleton);
		computeCCDIK(mRhandTarget, mRhandIKchain, &mIKSkeleton);
		computeCCDIK(mLfootTarget, mLfootIKchain, &mIKSkeleton);
		computeCCDIK(mRfootTarget, mRfootIKchain, &mIKSkeleton);
		break;
	default:
		mIKchain = createIKchain(endJointID, -1, &mIKSkeleton);
		computeCCDIK(target, mIKchain, &mIKSkeleton);
		break;
	}

	// update IK Skeleton transforms
	mIKSkeleton.update();

	// copy IK skeleton transforms to main skeleton
	m_pSkeleton->copyTransforms(&mIKSkeleton);

	return true;
}

int IKController::createCCDIKchains()
{
	bool validChains = false;

	int desiredChainSize = -1;  // default of -1 creates IK chain of maximum length from end joint to child joint of root


								// create IK chains for Lhand, Rhand, Lfoot and Rfoot 
	mLhandIKchain = createIKchain(mLhandID, desiredChainSize, &mIKSkeleton);
	mRhandIKchain = createIKchain(mRhandID, desiredChainSize, &mIKSkeleton);
	mLfootIKchain = createIKchain(mLfootID, desiredChainSize, &mIKSkeleton);
	mRfootIKchain = createIKchain(mRfootID, desiredChainSize, &mIKSkeleton);

	if (mLhandIKchain.getSize() > 1 && mRhandIKchain.getSize() > 1 && mLfootIKchain.getSize() > 1 && mRfootIKchain.getSize() > 1)
	{
		validChains = true;

		// initalize end joint target transforms for Lhand, Rhand, Lfoot and Rfoot based on current position and orientation of joints
		mIKSkeleton.copyTransforms(m_pSkeleton);
		mLhandTarget.setLocal2Global(mIKSkeleton.getJointByID(mLhandID)->getLocal2Global());
		mRhandTarget.setLocal2Global(mIKSkeleton.getJointByID(mRhandID)->getLocal2Global());
		mLfootTarget.setLocal2Global(mIKSkeleton.getJointByID(mLfootID)->getLocal2Global());
		mRfootTarget.setLocal2Global(mIKSkeleton.getJointByID(mRfootID)->getLocal2Global());
	}

	return validChains;
}


int IKController::computeCCDIK(ATarget target, AIKchain& IKchain, ASkeleton* pIKSkeleton)
{

	// TODO: Implement CCD IK  
	// The actual position of the end joint should match the desiredEndPos within some episilon error 

	bool result = false;

	mTarget0 = target;
	vec3 desiredEndPos = mTarget0.getGlobalTranslation();  // Get desired position of EndJoint

	int chainSize = IKchain.getSize();
	if (chainSize == 0) // There are no joints in the IK chain for manipulation
		return false;

	double epsilon = gIKEpsilon;
	int maxIterations = gIKmaxIterations;
	int numIterations = 0;

	m_pEndJoint = IKchain.getJoint(0);
	int endJointID = m_pEndJoint->getID();
	m_pBaseJoint = IKchain.getJoint(chainSize - 1);

	pIKSkeleton->copyTransforms(m_pSkeleton);

	if ((endJointID >= 0) && (endJointID < pIKSkeleton->getNumJoints()))
	{
		result = true;
		//TODO:
		for (numIterations = 0; numIterations < maxIterations; numIterations++) {
			AJoint* joint = m_pEndJoint->getParent();
			while(joint != m_pBaseJoint->getParent()) {
				// 1. compute axis and angle for each joint in the IK chain (distal to proximal) in global coordinates
				vec3 endpos = m_pEndJoint->getGlobalTranslation();
				vec3 jointpos = joint->getGlobalTranslation();
				
				vec3 r = endpos - jointpos;
				vec3 e = desiredEndPos - endpos;
				
				float rdotr = r * r;
				float rdote = r * e;
				vec3 rcrosse = r.Cross(e);
				float len_rcrosse = rcrosse.Length();

				float tanratio = len_rcrosse / (rdotr + rdote);
				float theta = tanratio;//we are doing small changes in theta... so tan(delta theta) is about equal to delta theta
				vec3 a = rcrosse / len_rcrosse;

				// 2. once you have the desired axis and angle, convert axis to local joint coords 
				mat3 joint_global2local = joint->getGlobalRotation().Inverse();
				vec3 a_local = joint_global2local * a;


				// 3. multiply angle by corresponding joint weight value
				float weightedtheta = theta * mWeight0;//since we're looping 5 times, spread changes throughout limb


				// 4. compute new local joint rotation matrix
				quat q;
				q.FromAxisAngle(a_local, weightedtheta);
				mat3 rotmat = q.ToRotation();
				if (abs(weightedtheta) < FLT_EPSILON) {
					rotmat.Identity();
				}
				mat3 newlocalrotmat = joint->getLocalRotation() * rotmat;
				joint->setLocalRotation(newlocalrotmat);

				// 5. update joint transform
				joint->updateTransform();

				// 6. repeat same operations above for each joint in the IKchain from end to base joint
				joint = joint->getParent();
			}
		}
		
	}
	return result;

}

//int IKController::computePseudoInvIK(ATarget target, AIKchain& IKchain, ASkeleton* pIKSkeleton)
//{
//	bool result = false;
//
//	mTarget0 = target;
//	vec3 desiredEndPos = mTarget0.getGlobalTranslation();  // Get desired position of EndJoint
//
//	int chainSize = IKchain.getSize();
//	if (chainSize == 0) // There are no joints in the IK chain for manipulation
//		return false;
//
//	double epsilon = gIKEpsilon;
//	int maxIterations = gIKmaxIterations;
//	int numIterations = 0;
//
//	m_pEndJoint = IKchain.getJoint(0);
//	int endJointID = m_pEndJoint->getID();
//	m_pBaseJoint = IKchain.getJoint(chainSize - 1);
//
//	pIKSkeleton->copyTransforms(m_pSkeleton);
//
//	if ((endJointID >= 0) && (endJointID < pIKSkeleton->getNumJoints()))
//	{
//		result = true;
//		AJoint* joint = m_pEndJoint;
//		//set up blank jacobian and v matrices and go through joints and calc B and L and put in jacobian
//		while (joint != m_pBaseJoint->getParent()) {
//			vec3 eulerangles;
//			mat3 localrot = joint->getLocalRotation();
//			localrot.ToEulerAngles(joint->getRotationOrder, eulerangles);
//			//meh
//
//
//
//			joint = joint->getParent();
//		}
//		//compute right pseudo inverse, mult by v matrix and you will get changes in theta for each axis for each joint
//		//update each joint by post multiplying its euler rot changes
//	}
//
//}
//bool IKController::IKSolver_PseudoInv(int endJointID, const ATarget& target)
//{
//	bool result = false;
//
//	// TODO: Implement Pseudo Inverse-based IK  
//	// The actual position of the end joint should match the target position after the skeleton is updated with the new joint angles
//	//instead of computeCCDIK need a new computePseudoInvIK
//	bool validChains = false;
//
//	if (!mvalidCCDIKchains)
//	{
//		mvalidCCDIKchains = createCCDIKchains();
//		//assert(mvalidCCDIKchains);
//	}
//
//	// copy transforms from base skeleton
//	mIKSkeleton.copyTransforms(m_pSkeleton);
//
//	vec3 desiredRootPosition;
//
//	switch (endJointID)
//	{
//	case mLhandID:
//		mLhandTarget = target;
//		computePseudoInvIK(mLhandTarget, mLhandIKchain, &mIKSkeleton);
//		break;
//	case mRhandID:
//		mRhandTarget = target;
//		computePseudoInvIK(mRhandTarget, mRhandIKchain, &mIKSkeleton);
//		break;
//	case mLfootID:
//		mLfootTarget = target;
//		computePseudoInvIK(mLfootTarget, mLfootIKchain, &mIKSkeleton);
//		break;
//	case mRfootID:
//		mRfootTarget = target;
//		computePseudoInvIK(mRfootTarget, mRfootIKchain, &mIKSkeleton);
//		break;
//	case mRootID:
//		desiredRootPosition = target.getGlobalTranslation();
//		mIKSkeleton.getJointByID(mRootID)->setLocalTranslation(desiredRootPosition);
//		mIKSkeleton.update();
//		computePseudoInvIK(mLhandTarget, mLhandIKchain, &mIKSkeleton);
//		computePseudoInvIK(mRhandTarget, mRhandIKchain, &mIKSkeleton);
//		computePseudoInvIK(mLfootTarget, mLfootIKchain, &mIKSkeleton);
//		computePseudoInvIK(mRfootTarget, mRfootIKchain, &mIKSkeleton);
//		break;
//	default:
//		mIKchain = createIKchain(endJointID, -1, &mIKSkeleton);
//		computePseudoInvIK(target, mIKchain, &mIKSkeleton);
//		break;
//	}
//
//	// update IK Skeleton transforms
//	mIKSkeleton.update();
//
//	// copy IK skeleton transforms to main skeleton
//	m_pSkeleton->copyTransforms(&mIKSkeleton);
//
//	return true;
//	return result;
//}

bool IKController::IKSolver_Other(int endJointID, const ATarget& target)
{

	bool result = false;

	// TODO: Put Optional IK implementation or enhancements here
	//just do limbIK followed by ccd, return a outofrange bool from limbik
	if (!mvalidLimbIKchains)
	{
		mvalidLimbIKchains = createLimbIKchains();
		//assert(mvalidLimbIKchains);
	}

	// copy transforms from base skeleton
	mIKSkeleton.copyTransforms(m_pSkeleton);

	vec3 desiredRootPosition;
	int outofrange = 0;

	switch (endJointID)
	{
	case mLhandID:
		mLhandTarget = target;
		outofrange = computeLimbIK(mLhandTarget, mLhandIKchain, -axisY, &mIKSkeleton);
		if (!mvalidCCDIKchains) //convert chains to ccd ik chains
			mvalidCCDIKchains = createCCDIKchains();
		if (outofrange) {
			computeCCDIK(mLhandTarget, mLhandIKchain, &mIKSkeleton);
		}

		break;
	case mRhandID:
		mRhandTarget = target;
		outofrange = computeLimbIK(mRhandTarget, mRhandIKchain, axisY, &mIKSkeleton);
		if (!mvalidCCDIKchains) //convert chains to ccd ik chains
			mvalidCCDIKchains = createCCDIKchains();
		if (outofrange) {
			computeCCDIK(mRhandTarget, mRhandIKchain, &mIKSkeleton);
		}
		break;
	case mLfootID:
		mLfootTarget = target;
		outofrange = computeLimbIK(mLfootTarget, mLfootIKchain, axisX, &mIKSkeleton);
		if (!mvalidCCDIKchains) //convert chains to ccd ik chains
			mvalidCCDIKchains = createCCDIKchains();
		if (outofrange) {
			computeCCDIK(mLfootTarget, mLfootIKchain, &mIKSkeleton);
		}
		break;
	case mRfootID:
		mRfootTarget = target;
		outofrange = computeLimbIK(mRfootTarget, mRfootIKchain, axisX, &mIKSkeleton);
		if (!mvalidCCDIKchains) //convert chains to ccd ik chains
			mvalidCCDIKchains = createCCDIKchains();
		if (outofrange) {
			computeCCDIK(mRfootTarget, mRfootIKchain, &mIKSkeleton);
		}
		break;
	case mRootID:
		desiredRootPosition = target.getGlobalTranslation();
		mIKSkeleton.getJointByID(mRootID)->setLocalTranslation(desiredRootPosition);
		mIKSkeleton.update();
		computeLimbIK(mLhandTarget, mLhandIKchain, -axisY, &mIKSkeleton);
		computeLimbIK(mRhandTarget, mRhandIKchain, axisY, &mIKSkeleton);
		computeLimbIK(mLfootTarget, mLfootIKchain, axisX, &mIKSkeleton);
		computeLimbIK(mRfootTarget, mRfootIKchain, axisX, &mIKSkeleton);
		break;
	default:
		mIKchain = createIKchain(endJointID, 3, &mIKSkeleton);
		outofrange = computeLimbIK(target, mIKchain, axisY, &mIKSkeleton);
		if (!mvalidCCDIKchains) //convert chains to ccd ik chains
			mvalidCCDIKchains = createCCDIKchains();
		if (outofrange) {
			computeCCDIK(target, mIKchain, &mIKSkeleton);
		}
		break;
	}

	// update IK Skeleton transforms
	mIKSkeleton.update();

	// copy IK skeleton transforms to main skeleton
	m_pSkeleton->copyTransforms(&mIKSkeleton);

	return true;
	return result;
}