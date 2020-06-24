///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

// WARNING: this file is compiled by all ReactPhysics3D plug-ins, so put only the code here that is "the same" for all versions.

#include "stdafx.h"
#include "PINT_CommonReactPhysics3D.h"

udword ReactPhysics3D::BatchRaycasts(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts)
{
	ASSERT(mPhysicsWorld);

	udword NbHits = 0;
	
	MyRaycastCallback resultCallback;

	while(nb--)
	{
		const rp3d::Vector3 from = ToRP3DVector3(raycasts->mOrigin);
		const rp3d::Vector3 target = ToRP3DVector3(raycasts->mOrigin + raycasts->mDir * raycasts->mMaxDist);

		rp3d::Ray ray(from, target);

		resultCallback.resetResult();
		mPhysicsWorld->raycast(ray, &resultCallback);

		if(resultCallback.hitBody != nullptr)
		{
			NbHits++;
			FillRaycastResultStruct(*dest, resultCallback, raycasts->mMaxDist);
		}
		else
		{
			dest->mObject = null;
		}

		raycasts++;
		dest++;
	}

	return NbHits;
}

static inline_ void FillRaycastResultStruct(PintRaycastHit& hit, const MyRaycastCallback& result, float max_dist)
{
//	hit.mObject			= result.m_collisionObject;
	hit.mObject			= (PintObjectHandle)(result.hitBody);
	hit.mImpact			= ToPoint(result.worldHitPoint);
	hit.mNormal			= ToPoint(result.worldHitNormal);
//	hit.mDistance		= origin.Distance(hit.mImpact);
	hit.mDistance		= result.hitFraction * max_dist;
	hit.mTriangleIndex	= result.hitTriangleIndex;
}

udword ReactPhysics3D::BatchBoxSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps)
{
	ASSERT(mPhysicsWorld);

	udword NbHits = 0;

	/*
	while(nb--)
	{
		const btBoxShape BoxShape(ToBtVector3(sweeps->mBox.mExtents));

		const btQuaternion Rot = ToBtQuaternion(Quat(sweeps->mBox.mRot));

		const btTransform from(Rot, ToBtVector3(sweeps->mBox.mCenter));
		const btTransform to(Rot, ToBtVector3(sweeps->mBox.mCenter + sweeps->mDir * sweeps->mMaxDist));

//		btCollisionWorld::ClosestConvexResultCallback resultCallback(from.getOrigin(), to.getOrigin());
		MyClosestConvexResultCallback resultCallback(from.getOrigin(), to.getOrigin());

		mDynamicsWorld->convexSweepTest(&BoxShape, from, to, resultCallback);

		if(resultCallback.m_hitCollisionObject)
		{
			NbHits++;
			FillResultStruct(*dest, resultCallback, sweeps->mMaxDist);
		}
		else
		{
			dest->mObject = null;
		}

		sweeps++;
		dest++;
	}
	*/

	return NbHits;
}

udword ReactPhysics3D::BatchSphereSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintSphereSweepData* sweeps)
{
	ASSERT(mPhysicsWorld);

	udword NbHits = 0;

	/*
	const btQuaternion Rot = btQuaternion::getIdentity();
	while(nb--)
	{
		const btSphereShape SphereShape(sweeps->mSphere.mRadius);

		const btTransform from(Rot, ToBtVector3(sweeps->mSphere.mCenter));
		const btTransform to(Rot, ToBtVector3(sweeps->mSphere.mCenter + sweeps->mDir * sweeps->mMaxDist));

//		btCollisionWorld::ClosestConvexResultCallback resultCallback(from.getOrigin(), to.getOrigin());
		MyClosestConvexResultCallback resultCallback(from.getOrigin(), to.getOrigin());

		mDynamicsWorld->convexSweepTest(&SphereShape, from, to, resultCallback);

		if(resultCallback.m_hitCollisionObject)
		{
			NbHits++;
			FillResultStruct(*dest, resultCallback, sweeps->mMaxDist);
		}
		else
		{
			dest->mObject = null;
		}

		sweeps++;
		dest++;
	}
	*/
	return NbHits;
}

static Quat ShortestRotation(const Point& v0, const Point& v1)
{
	const float d = v0|v1;
	const Point cross = v0^v1;

	Quat q = d>-1.0f ? Quat(1.0f + d, cross.x, cross.y, cross.z)
					: fabsf(v0.x)<0.1f ? Quat(0.0f, 0.0f, v0.z, -v0.y) : Quat(0.0f, v0.y, -v0.x, 0.0f);

	q.Normalize();

	return q;
}

udword ReactPhysics3D::BatchCapsuleSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintCapsuleSweepData* sweeps)
{
	ASSERT(mPhysicsWorld);

	udword NbHits = 0;

	/*
	while(nb--)
	{
		const Point Center = (sweeps->mCapsule.mP0 + sweeps->mCapsule.mP1)*0.5f;
		Point CapsuleAxis = sweeps->mCapsule.mP1 - sweeps->mCapsule.mP0;
		const float Height = CapsuleAxis.Magnitude();
		CapsuleAxis /= Height;
//		const Quat q = ShortestRotation(Point(1.0f, 0.0f, 0.0f), CapsuleAxis);
		const Quat q = ShortestRotation(Point(0.0f, 1.0f, 0.0f), CapsuleAxis);

		const btCapsuleShape CapsuleShape(sweeps->mCapsule.mRadius, Height);

		const btQuaternion Rot = ToBtQuaternion(q);

		const btTransform from(Rot, ToBtVector3(Center));
		const btTransform to(Rot, ToBtVector3(Center + sweeps->mDir * sweeps->mMaxDist));

//		btCollisionWorld::ClosestConvexResultCallback resultCallback(from.getOrigin(), to.getOrigin());
		MyClosestConvexResultCallback resultCallback(from.getOrigin(), to.getOrigin());

		mDynamicsWorld->convexSweepTest(&CapsuleShape, from, to, resultCallback);

		if(resultCallback.m_hitCollisionObject)
		{
			NbHits++;
			FillResultStruct(*dest, resultCallback, sweeps->mMaxDist);
		}
		else
		{
			dest->mObject = null;
		}

		sweeps++;
		dest++;
	}
	*/

	return NbHits;
}

udword ReactPhysics3D::CreateConvexObject(const PINT_CONVEX_DATA_CREATE& desc)
{
	//rp3d::ConvexMeshShape* shape = new rp3d::ConvexMeshShape(&desc.mVerts->x, desc.mNbVerts, sizeof(Point));
	//ASSERT(shape);

	///*if(desc.mRenderer)
	//	shape->setUserPointer(desc.mRenderer);*/

	//const udword CurrentSize = mConvexObjects.size();
	//mConvexObjects.push_back(shape);
	//return CurrentSize;

	return 1;
}

udword ReactPhysics3D::BatchConvexSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintConvexSweepData* sweeps)
{
	ASSERT(mPhysicsWorld);

	udword NbHits = 0;

	/*
	while(nb--)
	{
		btConvexHullShape* CapsuleShape = mConvexObjects[sweeps->mConvexObjectIndex];

		const Point& Center = sweeps->mTransform.mPos;
		const Quat& q = sweeps->mTransform.mRot;

//		const Point Center = (sweeps->mCapsule.mP0 + sweeps->mCapsule.mP1)*0.5f;
//		Point CapsuleAxis = sweeps->mCapsule.mP1 - sweeps->mCapsule.mP0;
//		const float Height = CapsuleAxis.Magnitude();
//		CapsuleAxis /= Height;
////		const Quat q = ShortestRotation(Point(1.0f, 0.0f, 0.0f), CapsuleAxis);
//		const Quat q = ShortestRotation(Point(0.0f, 1.0f, 0.0f), CapsuleAxis);
//
//		const btCapsuleShape CapsuleShape(sweeps->mCapsule.mRadius, Height);

		const btQuaternion Rot = ToBtQuaternion(q);

		const btTransform from(Rot, ToBtVector3(Center));
		const btTransform to(Rot, ToBtVector3(Center + sweeps->mDir * sweeps->mMaxDist));

//		btCollisionWorld::ClosestConvexResultCallback resultCallback(from.getOrigin(), to.getOrigin());
		MyClosestConvexResultCallback resultCallback(from.getOrigin(), to.getOrigin());

		mDynamicsWorld->convexSweepTest(CapsuleShape, from, to, resultCallback);

		if(resultCallback.m_hitCollisionObject)
		{
			NbHits++;
			FillResultStruct(*dest, resultCallback, sweeps->mMaxDist);
		}
		else
		{
			dest->mObject = null;
		}

		sweeps++;
		dest++;
	}
	*/
	return NbHits;
}

PR ReactPhysics3D::GetWorldTransform(PintObjectHandle handle)
{
	rp3d::RigidBody* body = (rp3d::RigidBody*)handle;

//	body->getCenterOfMassTransform()

	rp3d::Transform trans = body->getTransform();
	return ToPR(trans);
}


void ReactPhysics3D::SetWorldTransform(PintObjectHandle handle, const PR& pose)
{
	rp3d::RigidBody* body = (rp3d::RigidBody*)handle;

	rp3d::Transform transform(ToRP3DVector3(pose.mPos), ToRP3DQuaternion(pose.mRot));

	body->setTransform(transform);
}

void ReactPhysics3D::ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos)
{
	rp3d::RigidBody* body = (rp3d::RigidBody*)handle;

	if(body->getType() == rp3d::BodyType::STATIC || body->getType() == rp3d::BodyType::DYNAMIC)
		return;

	rp3d::Transform trans = body->getTransform();
//	trans = body->getCenterOfMassTransform();

	const rp3d::Vector3 rel_pos = ToRP3DVector3(pos) - trans.getPosition();

	if(action_type==PINT_ACTION_FORCE)
	{
		body->applyForceAtWorldPosition(ToRP3DVector3(action), rel_pos);
	}
	else if(action_type==PINT_ACTION_IMPULSE)
	{
		//body->applyImpulse(ToRP3DVector3(action), rel_pos);
	}
	else ASSERT(0);
}

Point ReactPhysics3D::GetAngularVelocity(PintObjectHandle handle)
{
	const rp3d::RigidBody* body = (const rp3d::RigidBody*)handle;
	return ToPoint(body->getAngularVelocity());
}

void ReactPhysics3D::SetAngularVelocity(PintObjectHandle handle, const Point& angular_velocity)
{
	rp3d::RigidBody* body = (rp3d::RigidBody*)handle;
	body->setAngularVelocity(ToRP3DVector3(angular_velocity));
}

float ReactPhysics3D::GetMass(PintObjectHandle handle)
{
	const rp3d::RigidBody* body = (const rp3d::RigidBody*)handle;
	return body->getMass();
}

Point ReactPhysics3D::GetLocalInertia(PintObjectHandle handle)
{
	const rp3d::RigidBody* body = (const rp3d::RigidBody*)handle;
	const rp3d::Vector3 inertia = body->getLocalInertiaTensor();
	return Point(inertia.x, inertia.y, inertia.z);
}
