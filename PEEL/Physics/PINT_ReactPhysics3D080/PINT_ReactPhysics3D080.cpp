///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include <reactphysics3d/reactphysics3d.h>

#include "..\PINT_Common\PINT_Common.h"
#include "..\PINT_Common\PINT_CommonReactPhysics3D.h"

///////////////////////////////////////////////////////////////////////////////

//static	const	bool	gEnableCollisionBetweenJointed	= true;
static	const	float	gGlobalBoxSize					= 10000.0f;
static			float	gLinearDamping					= 0.1f;
static			float	gAngularDamping					= 0.05f;
static			float	gErp							= 0.2f;
static			float	gErp2							= 0.1f;
static			float	gCollisionMargin				= 0.04f;
static			bool	gUseSplitImpulse				= true;
static			bool	gRandomizeOrder					= false;
static			bool	gWarmStarting					= true;
static			bool	gShareShapes					= true;
static			bool	gUseDbvt						= true;
static			bool	gUseCCD							= false;
static			bool	gEnableSleeping					= false;
static			bool	gEnableFrictionDirCaching		= true;
static			udword	gSolverIterationCount			= 4;
static	const	bool	gUseCustomMemoryAllocator		= false;

#define	NB_DEBUG_VIZ_PARAMS	6
static			bool	gDebugVizParams[NB_DEBUG_VIZ_PARAMS] = {0};
static	const	char*	gDebugVizNames[NB_DEBUG_VIZ_PARAMS] =
{
	"Enable debug visualization",
	"Draw wireframe",
	"Draw AABBs",
	"Draw contact points",
	"Draw constraints",
	"Draw constraints limits",
};

/*
static			int	gDebugVizMask[NB_DEBUG_VIZ_PARAMS] =
{
	btIDebugDraw::DBG_NoDebug,
	btIDebugDraw::DBG_DrawWireframe,
	btIDebugDraw::DBG_DrawAabb,
	btIDebugDraw::DBG_DrawContactPoints,
	btIDebugDraw::DBG_DrawConstraints,
	btIDebugDraw::DBG_DrawConstraintLimits,
};
*/

/*
		DBG_DrawWireframe = 1,
		DBG_DrawAabb=2,
		DBG_DrawFeaturesText=4,
		DBG_DrawContactPoints=8,
		DBG_NoDeactivation=16,
		DBG_NoHelpText = 32,
		DBG_DrawText=64,
		DBG_ProfileTimings = 128,
		DBG_EnableSatComparison = 256,
		DBG_DisableBulletLCP = 512,
		DBG_EnableCCD = 1024,
		DBG_DrawConstraints = (1 << 11),
		DBG_DrawConstraintLimits = (1 << 12),
		DBG_FastWireframe = (1<<13),
		DBG_MAX_DEBUG_DRAW_MODE
*/

///////////////////////////////////////////////////////////////////////////////

static udword	gNbAllocs = 0;
static udword	gCurrentMemory = 0;

	struct Header
	{
		udword	mMagic;
		udword	mSize;
	};

static void* __btAlignedAllocFunc(size_t size, int alignment)
{
	char* memory = (char*)_aligned_malloc(size+16, 16);
	Header* H = (Header*)memory;
	H->mMagic = 0x12345678;
	H->mSize = size;
	gNbAllocs++;
	gCurrentMemory+=size;
	return memory + 16;
}

static void __btAlignedFreeFunc(void* ptr)
{
	if(!ptr)
		return;
	char* bptr = (char*)ptr;
	Header* H = (Header*)(bptr - 16);
	ASSERT(H->mMagic==0x12345678);
	const udword Size = H->mSize;
	_aligned_free(H);
	gNbAllocs--;
	gCurrentMemory-=Size;
}

static void* __btAllocFunc(size_t size)
{
	char* memory = (char*)_aligned_malloc(size+16, 16);
	Header* H = (Header*)memory;
	H->mMagic = 0x12345678;
	H->mSize = size;
	gNbAllocs++;
	gCurrentMemory+=size;
	return memory + 16;
}

static void __btFreeFunc(void* ptr)
{
	if(!ptr)
		return;
	char* bptr = (char*)ptr;
	Header* H = (Header*)(bptr - 16);
	ASSERT(H->mMagic==0x12345678);
	const udword Size = H->mSize;
	_aligned_free(H);
	gNbAllocs--;
	gCurrentMemory-=Size;
}

///////////////////////////////////////////////////////////////////////////////

/*
class MyDebugDrawer : public btIDebugDraw
{
	public:

	MyDebugDrawer() : mRenderer(null), mDebugMode(0)
	{
	}

	virtual void	drawLine(const btVector3& from,const btVector3& to,const btVector3& color)
	{
		ASSERT(mRenderer);
		mRenderer->DrawLine(ToPoint(from), ToPoint(to), ToPoint(color));
	}

	virtual void	drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color)
	{
		ASSERT(mRenderer);
		mRenderer->DrawLine(ToPoint(PointOnB), ToPoint(PointOnB+normalOnB), ToPoint(color));
	}

	virtual void	reportErrorWarning(const char* warningString)
	{
	}

	virtual void	draw3dText(const btVector3& location,const char* textString)
	{
	}

	virtual void	setDebugMode(int debugMode)
	{
		mDebugMode = debugMode;
	}

	virtual int		getDebugMode() const
	{
		return mDebugMode;
	}

	void updateDebugMode()
	{
		mDebugMode = 0;
		if(gDebugVizParams[0])
		{
			for(udword i=1;i<NB_DEBUG_VIZ_PARAMS;i++)
			{
				if(gDebugVizParams[i])
					mDebugMode |= gDebugVizMask[i];
			}
		}
	}

	PintRender*	mRenderer;
	int			mDebugMode;

}gDebugDrawer;
*/
///////////////////////////////////////////////////////////////////////////////

ReactPhysics3D::ReactPhysics3D() : mPhysicsWorld			(null)
{
}

ReactPhysics3D::~ReactPhysics3D()
{

}

void ReactPhysics3D::GetCaps(PintCaps& caps) const
{
	caps.mSupportRigidBodySimulation	= true;
	caps.mSupportKinematics				= false;
	caps.mSupportCollisionGroups		= true;
	caps.mSupportCompounds				= true;
	caps.mSupportConvexes				= false;
	caps.mSupportMeshes					= true;
	caps.mSupportSphericalJoints		= true;
	caps.mSupportHingeJoints			= true;
	caps.mSupportFixedJoints			= true;
	caps.mSupportPrismaticJoints		= true;
	caps.mSupportPhantoms				= false;
	caps.mSupportRaycasts				= true;
	caps.mSupportBoxSweeps				= false;
	caps.mSupportSphereSweeps			= false;
	caps.mSupportCapsuleSweeps			= false;
	caps.mSupportConvexSweeps			= false;
	caps.mSupportSphereOverlaps			= false;
	caps.mSupportBoxOverlaps			= false;
	caps.mSupportCapsuleOverlaps		= false;
	caps.mSupportConvexOverlaps			= false;
}
  
void ReactPhysics3D::Init(const PINT_WORLD_CREATE& desc)
{
	for(udword i=0;i<32;i++)
		mGroupMasks[i] = 0xffffffff;

	gNbAllocs = 0;
	gCurrentMemory = 0;
	if(gUseCustomMemoryAllocator)
	{
		/*
		btAlignedAllocSetCustomAligned(__btAlignedAllocFunc, __btAlignedFreeFunc);
		btAlignedAllocSetCustom(__btAllocFunc, __btFreeFunc);
		*/
	}

	// Create the physics world
	rp3d::PhysicsWorld::WorldSettings worldSettings;
	worldSettings.gravity = ToRP3DVector3(desc.mGravity);
	mPhysicsWorld = mPhysicsCommon.createPhysicsWorld(worldSettings);
    mPhysicsWorld->setNbIterationsVelocitySolver(gSolverIterationCount);
	mPhysicsWorld->setContactsPositionCorrectionTechnique(gUseSplitImpulse ? rp3d::ContactsPositionCorrectionTechnique::SPLIT_IMPULSES : rp3d::ContactsPositionCorrectionTechnique::BAUMGARTE_CONTACTS);
	//mDynamicsWorld->setContactsPositionCorrectionTechnique(rp3d::BAUMGARTE_CONTACTS);
	//mDynamicsWorld->setGravity(ToBtVector3(desc.mGravity));

	/*
	btContactSolverInfo& SolverInfo = mDynamicsWorld->getSolverInfo();
	if(gEnableFrictionDirCaching)
		SolverInfo.m_solverMode |= SOLVER_ENABLE_FRICTION_DIRECTION_CACHING; //don't recalculate friction values each frame
	else
		SolverInfo.m_solverMode &= ~SOLVER_ENABLE_FRICTION_DIRECTION_CACHING;
	if(gRandomizeOrder)
		SolverInfo.m_solverMode |= SOLVER_RANDMIZE_ORDER;
	else
		SolverInfo.m_solverMode &= ~SOLVER_RANDMIZE_ORDER;
	if(gWarmStarting)
		SolverInfo.m_solverMode |= SOLVER_USE_WARMSTARTING;
	else
		SolverInfo.m_solverMode &= ~SOLVER_USE_WARMSTARTING;
	SolverInfo.m_numIterations	= gSolverIterationCount;
	SolverInfo.m_splitImpulse	= gUseSplitImpulse;
	SolverInfo.m_erp			= gErp;
	SolverInfo.m_erp2			= gErp2;
	*/

/*
		discreteWorld->getSolverInfo().m_linearSlop = gSlop;
		discreteWorld->getSolverInfo().m_warmstartingFactor = gWarmStartingParameter;
*/
	
	//mDynamicsWorld->setDebugDrawer(&gDebugDrawer);
}

void ReactPhysics3D::SetGravity(const Point& gravity)
{
	ASSERT(mPhysicsWorld);
	mPhysicsWorld->setGravity(ToRP3DVector3(gravity));
}

void ReactPhysics3D::Close()
{
	//cleanup in the reverse order of creation/initialization

	/*
	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for(i=mDynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = mDynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		mDynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}
	*/

	/*
	for(udword j=0;j<mMeshShapes.size();j++)
	{
		InternalMeshShape& shape = mMeshShapes[j];
		DELETESINGLE(shape.mMeshData);
	}

	//delete collision shapes
	for(udword j=0;j<mCollisionShapes.size();j++)
	{
		rp3d::CollisionShape* shape = mCollisionShapes[j];
		DELETESINGLE(shape);
	}

	//delete polygon vertex arrays
	for (udword j = 0; j<mPolygonVertexArrays.size(); j++)
	{
		rp3d::PolygonVertexArray* array = mPolygonVertexArrays[j];
		DELETESINGLE(array);
	}

	//delete polygon meshes
	for (udword j = 0; j<mPolyhedronMeshes.size(); j++)
	{
		rp3d::PolyhedronMesh* mesh = mPolyhedronMeshes[j];
		DELETESINGLE(mesh);
	}

	
	
	for(udword j=0;j<mConvexObjects.size();j++)
	{
		rp3d::ConvexMeshShape* shape = mConvexObjects[j];
		DELETESINGLE(shape);
	}
	*/

	mPhysicsCommon.destroyPhysicsWorld(mPhysicsWorld);
	//DELETESINGLE(mDynamicsWorld);

	if(gNbAllocs)
		printf("ReactPhysics3D 0.8.0: %d leaks found (%d bytes)\n", gNbAllocs, gCurrentMemory);
}

udword ReactPhysics3D::Update(float dt)
{
	if(mPhysicsWorld)
	{
		mPhysicsWorld->update(dt);
	}
	return gCurrentMemory;
}

static void DrawLeafShape(PintRender& renderer, const rp3d::Collider* collider, const PR& pose)
{
	ASSERT(collider->getUserData());
	if(collider->getUserData())
	{
		PintShapeRenderer* psr = (PintShapeRenderer*)collider->getUserData();
		psr->Render(pose);
	}
/*	else
	{
		const int Type = shape->getShapeType();
		if(Type==SPHERE_SHAPE_PROXYTYPE)
		{
			const btSphereShape* SphereShape = static_cast<const btSphereShape*>(shape);
			const float Radius = SphereShape->getRadius();
			renderer.DrawSphere(Radius, pose);
		}
		else if(Type==BOX_SHAPE_PROXYTYPE)
		{
			const btBoxShape* BoxShape = static_cast<const btBoxShape*>(shape);
			const btVector3& Extents = BoxShape->getHalfExtentsWithoutMargin();
			renderer.DrawBox(ToPoint(Extents), pose);
		}
		else if(Type==CONVEX_HULL_SHAPE_PROXYTYPE)
		{
			const btConvexHullShape* ConvexShape = static_cast<const btConvexHullShape*>(shape);
			ASSERT(0);
		}
		else ASSERT(0);
	}*/
}

Point ReactPhysics3D::GetMainColor()
{
	return Point(1.0f, 0.4f, 0.4f);
}

void ReactPhysics3D::Render(PintRender& renderer)
{
	const int nbRigidBodies = mPhysicsWorld->getNbRigidBodies();

	//const int size = mDynamicsWorld->getNumCollisionObjects();
	for(int i=0; i < nbRigidBodies; i++)
	{
		//btCollisionObject* obj = mDynamicsWorld->getCollisionObjectArray()[i];
		rp3d::RigidBody* body = mPhysicsWorld->getRigidBody(i);

		rp3d::Transform trans = body->getTransform();
		const PR BodyPose = ToPR(trans);

		// For each collider of the body
		const int nbColliders = body->getNbColliders();
		for (int j=0; j < nbColliders; j++) {

			rp3d::Collider* CurrentCollider = body->getCollider(j);

			const rp3d::Transform colliderToWorldTransform = CurrentCollider->getLocalToWorldTransform();

			const PR ComboPose = ToPR(colliderToWorldTransform);

			DrawLeafShape(renderer, CurrentCollider, ComboPose);
		}
	}

	// TODO : Implement this
	/*
	// Draw contact points
	std::vector<rp3d::Vector3> contactPoints = computeContactPointsOfWorld();
	for (int i = 0; i < contactPoints.size(); i++) {
		PR pose = PR(ToPoint(contactPoints[i]), ToQuat(rp3d::Quaternion::identity()));
		renderer.DrawSphere(0.1f, pose);
	}
	*/
	
	//gDebugDrawer.mRenderer = &renderer;
	//mDynamicsWorld->debugDrawWorld();
}

// Return all the contact points of the scene
std::vector<rp3d::Vector3> ReactPhysics3D::computeContactPointsOfWorld() const {

	std::vector<rp3d::Vector3> contactPoints;

	/*
	// Get the list of contact manifolds from the world
	std::vector<const rp3d::ContactManifold*> manifolds = mDynamicsWorld->getContactsList();

	// For each contact manifold
	std::vector<const rp3d::ContactManifold*>::const_iterator it;
	for (it = manifolds.begin(); it != manifolds.end(); ++it) {

		const rp3d::ContactManifold* manifold = *it;

		// For each contact point of the manifold
		for (int i = 0; i<manifold->getNbContactPoints(); i++) {

			rp3d::ContactPoint* contactPoint = manifold->getContactPoints();
			while (contactPoint != nullptr) {
				rp3d::Vector3 point = manifold->getShape1()->getLocalToWorldTransform() * contactPoint->getLocalPointOnShape1();
				contactPoints.push_back(point);

				contactPoint = contactPoint->getNext();
			}
		}

	}
	*/

	return contactPoints;
}

//static inline_ void RegisterShape()

rp3d::SphereShape* ReactPhysics3D::FindSphereShape(const PINT_SPHERE_CREATE& create)
{
	const float Radius = create.mRadius;
	if(gShareShapes)
	{
		const int size = mSphereShapes.size();
		for(int i=0;i<size;i++)
		{
			rp3d::SphereShape* CurrentShape = mSphereShapes[i];
			if(CurrentShape->getRadius()==Radius)
			{
				return CurrentShape;
			}
		}
	}

	rp3d::SphereShape* shape = mPhysicsCommon.createSphereShape(create.mRadius);
	ASSERT(shape);
	mSphereShapes.push_back(shape);
	mCollisionShapes.push_back(shape);

	/*
	if (create.mRenderer)
		shape->setUserPointer(create.mRenderer);
	*/

	return shape;
}

rp3d::BoxShape* ReactPhysics3D::FindBoxShape(const PINT_BOX_CREATE& create)
{
	if(gShareShapes)
	{
		const int size = mBoxShapes.size();
		for(int i=0;i<size;i++)
		{
			const InternalBoxShape& CurrentShape = mBoxShapes[i];
			if(		CurrentShape.mExtents.x==create.mExtents.x
				&&	CurrentShape.mExtents.y==create.mExtents.y
				&&	CurrentShape.mExtents.z==create.mExtents.z)
			{
				return CurrentShape.mShape;
			}
		}
	}

	// Extents without the collision margin
	rp3d::Vector3 extents = ToRP3DVector3(create.mExtents);
	//extents -= rp3d::Vector3(gCollisionMargin, gCollisionMargin, gCollisionMargin);

	rp3d::BoxShape* shape = mPhysicsCommon.createBoxShape(extents);
	ASSERT(shape);
	InternalBoxShape Internal;
	Internal.mShape		= shape;
	Internal.mExtents	= create.mExtents;
	mBoxShapes.push_back(Internal);
	mCollisionShapes.push_back(shape);

	//if(create.mRenderer)
	//	shape->setUserPointer(create.mRenderer);

	return shape;
}

rp3d::CapsuleShape* ReactPhysics3D::FindCapsuleShape(const PINT_CAPSULE_CREATE& create)
{
	const float Radius = create.mRadius;
	const float HalfHeight = create.mHalfHeight;
	const float height = HalfHeight * 2.0f;
	if(gShareShapes)
	{
		const int size = mCapsuleShapes.size();
		for(int i=0;i<size;i++)
		{
			rp3d::CapsuleShape* CurrentShape = mCapsuleShapes[i];
			if(CurrentShape->getRadius()==Radius && CurrentShape->getHeight()== height)
			{
				return CurrentShape;
			}
		}
	}

	rp3d::CapsuleShape* shape = mPhysicsCommon.createCapsuleShape(Radius, height);
	ASSERT(shape);
	mCapsuleShapes.push_back(shape);
	mCollisionShapes.push_back(shape);

	/*if(create.mRenderer)
		shape->setUserPointer(create.mRenderer);*/

	return shape;
}

rp3d::ConvexMeshShape* ReactPhysics3D::FindConvexShape(const PINT_CONVEX_CREATE& create)
{
	//// TODO : Find a way to handle shared convex mesh shape here
	///*
	//if(gShareShapes)
	//{
	//	const int size = mConvexShapes.size();
	//	for(int i=0;i<size;i++)
	//	{
	//		rp3d::ConvexMeshShape* CurrentShape = mConvexShapes[i];
	//		if(CurrentShape->get==create.mRenderer)
	//		{
	//			return CurrentShape;
	//		}
	//	}
	//}
	//*/
	//create.
	//rp3d::PolygonVertexArray* polygonVertexArray =
	//	new rp3d::PolygonVertexArray(create.mNbVerts, &create.mVerts->x, sizeof(Point),
	//		&(mIndices[0][0]), sizeof(int),
	//		getNbFaces(0), mPolygonFaces,
	//		rp3d::PolygonVertexArray::VertexDataType::VERTEX_FLOAT_TYPE,
	//		rp3d::PolygonVertexArray::IndexDataType::INDEX_INTEGER_TYPE);

	//// Create the polyhedron mesh
	//rp3d::PolyhedronMesh polyhedronMesh = new rp3d::PolyhedronMesh(polygonVertexArray);

	//rp3d::ConvexMeshShape* shape = new rp3d::ConvexMeshShape(polyhedronMesh);
	//ASSERT(shape);
	//mConvexShapes.push_back(shape);
	//mCollisionShapes.push_back(shape);
	//mPolygonVertexArrays.push_back(polygonVertexArray);
	//mPolyhedronMeshes.push_back(polyhedronMesh);

	///*if(create.mRenderer)
	//	shape->setUserPointer(create.mRenderer);*/

	//return shape;
	return nullptr;
}

rp3d::CollisionShape* ReactPhysics3D::CreateReactPhysics3DShape(const PINT_SHAPE_CREATE& desc)
{
	rp3d::CollisionShape* collisionShape = null;
	if(desc.mType==PINT_SHAPE_SPHERE)
	{
		const PINT_SPHERE_CREATE& SphereCreate = static_cast<const PINT_SPHERE_CREATE&>(desc);
		collisionShape = FindSphereShape(SphereCreate);
	}
	else if(desc.mType==PINT_SHAPE_BOX)
	{
		const PINT_BOX_CREATE& BoxCreate = static_cast<const PINT_BOX_CREATE&>(desc);
		collisionShape = FindBoxShape(BoxCreate);
	}
	else if(desc.mType==PINT_SHAPE_CAPSULE)
	{
		const PINT_CAPSULE_CREATE& CapsuleCreate = static_cast<const PINT_CAPSULE_CREATE&>(desc);
		collisionShape = FindCapsuleShape(CapsuleCreate);
	}
	/*else if(desc.mType==PINT_SHAPE_CONVEX)
	{
		const PINT_CONVEX_CREATE& ConvexCreate = static_cast<const PINT_CONVEX_CREATE&>(desc);
		collisionShape = FindConvexShape(ConvexCreate);
	}*/
	else if(desc.mType==PINT_SHAPE_MESH)
	{
		const PINT_MESH_CREATE& MeshCreate = static_cast<const PINT_MESH_CREATE&>(desc);

		// ### share meshes

		// Vertex and Indices array for the triangle mesh (data in shared and not copied)
        rp3d::TriangleVertexArray* vertexArray =
                new rp3d::TriangleVertexArray(MeshCreate.mSurface.mNbVerts, (float*)&MeshCreate.mSurface.mVerts->x, sizeof(Point),
                                              MeshCreate.mSurface.mNbFaces, (int*)MeshCreate.mSurface.mDFaces, 3 * sizeof(udword),
                                              rp3d::TriangleVertexArray::VertexDataType::VERTEX_FLOAT_TYPE,
                                              rp3d::TriangleVertexArray::IndexDataType::INDEX_INTEGER_TYPE);

		rp3d::TriangleMesh* triangleMesh = mPhysicsCommon.createTriangleMesh();
        // Add the triangle vertex array of the subpart to the triangle mesh
        triangleMesh->addSubpart(vertexArray);

		rp3d::ConcaveMeshShape* shape = mPhysicsCommon.createConcaveMeshShape(triangleMesh);

		ASSERT(shape);

		InternalMeshShape MeshData;
		MeshData.mShape		= shape;
		MeshData.mMeshData	= triangleMesh;
		mMeshShapes.push_back(MeshData);

		mCollisionShapes.push_back(shape);

		/*
		if(desc.mRenderer)
			shape->setUserPointer(desc.mRenderer);
		*/

		collisionShape = shape;
	}

	else ASSERT(0);

	return collisionShape;
}

PintObjectHandle ReactPhysics3D::CreateObject(const PINT_OBJECT_CREATE& desc)
{
	udword NbShapes = desc.GetNbShapes();
	if(!NbShapes)
		return null;

	ASSERT(mPhysicsWorld);

	const PINT_SHAPE_CREATE* CurrentShape = desc.mShapes;

	const rp3d::Transform startTransform(ToRP3DVector3(desc.mPosition), ToRP3DQuaternion(desc.mRotation));

	// Create the rigid body
	rp3d::RigidBody* body = mPhysicsWorld->createRigidBody(startTransform);
	ASSERT(body);
	body->setIsAllowedToSleep(gEnableSleeping);
	const bool isDynamic = (desc.mMass != 0.0f);

	// Damping
	body->setLinearDamping(gLinearDamping);
	body->setAngularDamping(gAngularDamping);

	// For each collider to create
	while(CurrentShape)
	{
		const rp3d::Transform LocalPose(ToRP3DVector3(CurrentShape->mLocalPos), ToRP3DQuaternion(CurrentShape->mLocalRot));

		rp3d::CollisionShape* shape = CreateReactPhysics3DShape(*CurrentShape);
		if(shape)
		{
			rp3d::Collider* collider = body->addCollider(shape, LocalPose);

			if (CurrentShape->mRenderer)
				collider->setUserData(CurrentShape->mRenderer);

			if(CurrentShape->mMaterial)
			{
				collider->getMaterial().setBounciness(CurrentShape->mMaterial->mRestitution);
				collider->getMaterial().setFrictionCoefficient(CurrentShape->mMaterial->mDynamicFriction);

				if (NbShapes == 1) {
					float mass = desc.mMassForInertia > 0 ? desc.mMassForInertia : desc.mMass;
					collider->getMaterial().setMassDensity(mass / collider->getCollisionShape()->getVolume());
				}
			}

			// Collision groups
			if(desc.mCollisionGroup)
			{
				ASSERT(desc.mCollisionGroup<32);
				unsigned short collisionFilterGroup = unsigned short(1 << desc.mCollisionGroup);
				unsigned short collisionFilterMask = unsigned short(mGroupMasks[desc.mCollisionGroup]);
				collider->setCollisionCategoryBits(collisionFilterGroup);
				collider->setCollideWithMaskBits(collisionFilterMask);
			}

		}

		CurrentShape = CurrentShape->mNext;
	}

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	//btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	//sword collisionFilterGroup, collisionFilterMask;

	if(isDynamic)
	{
		body->updateMassPropertiesFromColliders();
		body->setMass(desc.mMass);

		body->setLinearVelocity(ToRP3DVector3(desc.mLinearVelocity));
		body->setAngularVelocity(ToRP3DVector3(desc.mAngularVelocity));

		if (desc.mCOMLocalOffset.IsNonZero()) {
			body->setLocalCenterOfMass(body->getLocalCenterOfMass() + ToRP3DVector3(desc.mCOMLocalOffset));
		}

		//collisionFilterGroup = short(btBroadphaseProxy::DefaultFilter);
		//collisionFilterMask = short(btBroadphaseProxy::AllFilter);

		/*if(desc.mCollisionGroup)
		{
			const udword btGroup = RemapCollisionGroup(desc.mCollisionGroup);
			ASSERT(btGroup<32);
			collisionFilterGroup = short(1<<btGroup);
			collisionFilterMask = short(mGroupMasks[btGroup]);
		}*/
	}
	else
	{
		body->setType(rp3d::BodyType::STATIC);

		/*body->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);

		collisionFilterGroup = short(btBroadphaseProxy::StaticFilter);
		collisionFilterMask = short(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);*/
	}

	//if(desc.mAddToWorld)
	//	mDynamicsWorld->create(body, collisionFilterGroup, collisionFilterMask);

	if(gUseCCD)
	{
//		body->setCcdMotionThreshold(1e-7);
//		body->setCcdSweptSphereRadius(0.9*CUBE_HALF_EXTENTS);

		//body->setCcdMotionThreshold(0.0001f);
		//body->setCcdSweptSphereRadius(0.4f);
	}

	return body;
}

bool ReactPhysics3D::ReleaseObject(PintObjectHandle handle)
{
	//### shapes and stuff?
	//btRigidBody* body = (btRigidBody*)handle;
	//mDynamicsWorld->removeRigidBody(body);

	rp3d::RigidBody* body = (rp3d::RigidBody*)handle;
	mPhysicsWorld->destroyRigidBody(body);

	//DELETESINGLE(body);
	return true;
}

/*
static rp3d::Transform CreateFrame(const Point& local_pivot, const Point& local_axis)
{
	rp3d::Transform frame;

	Point Right, Up;
	ComputeBasis(local_axis, Right, Up);

	frame.setPosition(ToRP3DVector3(local_pivot));

	rp3d::Matrix3x3 basis;
	basis[0] = ToRP3DVector3(local_axis);
	basis[1] = ToRP3DVector3(Right);
	basis[2] = ToRP3DVector3(Up);
	frame.setBasis(basis);

	return frame;
}
*/

PintJointHandle ReactPhysics3D::CreateJoint(const PINT_JOINT_CREATE& desc)
{
	ASSERT(mPhysicsWorld);
	void* constraint = null;

	rp3d::RigidBody* body0 = (rp3d::RigidBody*)desc.mObject0;
	rp3d::RigidBody* body1 = (rp3d::RigidBody*)desc.mObject1;

	switch(desc.mType)
	{
		case PINT_JOINT_SPHERICAL:
		{
			const PINT_SPHERICAL_JOINT_CREATE& jc = static_cast<const PINT_SPHERICAL_JOINT_CREATE&>(desc);

			const rp3d::Transform& body0ToWorld = body0->getTransform();
			rp3d::BallAndSocketJointInfo jointInfo(body0, body1, body0ToWorld * ToRP3DVector3(jc.mLocalPivot0));
			jointInfo.isCollisionEnabled = false;

			constraint = mPhysicsWorld->createJoint(jointInfo);
			ASSERT(constraint);
		}
		break;

		case PINT_JOINT_HINGE:
		{
			const PINT_HINGE_JOINT_CREATE& jc = static_cast<const PINT_HINGE_JOINT_CREATE&>(desc);

			const rp3d::Transform& body0ToWorld = body0->getTransform();
			rp3d::HingeJointInfo jointInfo(body0, body1, body0ToWorld * ToRP3DVector3(jc.mLocalPivot0),
				body0ToWorld.getOrientation() * ToRP3DVector3(jc.mLocalAxis0));
			jointInfo.isCollisionEnabled = false;

			if (jc.mMinLimitAngle != MIN_FLOAT || jc.mMaxLimitAngle != MAX_FLOAT) {
				jointInfo.isLimitEnabled = true;
				jointInfo.minAngleLimit = jc.mMinLimitAngle;
				jointInfo.maxAngleLimit = jc.mMaxLimitAngle;
			}

			constraint = mPhysicsWorld->createJoint(jointInfo);
		}
		break;

		case PINT_JOINT_FIXED:
		{
			const PINT_FIXED_JOINT_CREATE& jc = static_cast<const PINT_FIXED_JOINT_CREATE&>(desc);

			const rp3d::Transform& body0ToWorld = body0->getTransform();
			rp3d::FixedJointInfo jointInfo(body0, body1, body0ToWorld * ToRP3DVector3(jc.mLocalPivot0));
			jointInfo.isCollisionEnabled = false;

			constraint = mPhysicsWorld->createJoint(jointInfo);

			ASSERT(constraint);
		}
		break;

		case PINT_JOINT_PRISMATIC:
		{
			const PINT_PRISMATIC_JOINT_CREATE& jc = static_cast<const PINT_PRISMATIC_JOINT_CREATE&>(desc);

			const rp3d::Transform& body0ToWorld = body0->getTransform();
			rp3d::SliderJointInfo jointInfo(body0, body1, body0ToWorld * ToRP3DVector3(jc.mLocalPivot0),
											body0ToWorld.getOrientation() * ToRP3DVector3(jc.mLocalAxis0));
			jointInfo.isCollisionEnabled = false;

			if(jc.mMinLimit <= jc.mMaxLimit) {
				jointInfo.isLimitEnabled = true;
				jointInfo.minTranslationLimit = jc.mMinLimit;
				jointInfo.maxTranslationLimit = jc.mMaxLimit;
			}

			constraint = mPhysicsWorld->createJoint(jointInfo);

			ASSERT(constraint);
		}
		break;
	}

	return constraint;
}

void ReactPhysics3D::SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups)
{
	for(udword i=0;i<nb_groups;i++)
	{
		ASSERT(groups[i].mGroup0 < 32);
		ASSERT(groups[i].mGroup1 < 32);

		udword Mask0 = mGroupMasks[groups[i].mGroup0];
		Mask0 ^= 1<<groups[i].mGroup1;
		mGroupMasks[groups[i].mGroup0] = Mask0;

		udword Mask1 = mGroupMasks[groups[i].mGroup1];
		Mask1 ^= 1<<groups[i].mGroup0;
		mGroupMasks[groups[i].mGroup1] = Mask0;
	}
}





static ReactPhysics3D* gBullet = null;
static void gBullet_GetOptionsFromGUI();

void ReactPhysics3D_Init(const PINT_WORLD_CREATE& desc)
{
	gBullet_GetOptionsFromGUI();

	ASSERT(!gBullet);
	gBullet = ICE_NEW(ReactPhysics3D);
	gBullet->Init(desc);
}

void ReactPhysics3D_Close()
{
	if(gBullet)
	{
		gBullet->Close();
		delete gBullet;
		gBullet = null;
	}
}

ReactPhysics3D* GetReactPhysics3D()
{
	return gBullet;
}

///////////////////////////////////////////////////////////////////////////////

static Container*	gBulletGUI = null;
static IceEditBox*	gEditBox_SolverIter = null;
static IceEditBox*	gEditBox_LinearDamping = null;
static IceEditBox*	gEditBox_AngularDamping = null;
static IceEditBox*	gEditBox_Erp = null;
static IceEditBox*	gEditBox_Erp2 = null;
static IceEditBox*	gEditBox_CollisionMargin = null;
static IceCheckBox*	gCheckBox_DebugVis[NB_DEBUG_VIZ_PARAMS] = {0};

enum BulletGUIElement
{
	BULLET_GUI_MAIN,
	//
	BULLET_GUI_ENABLE_SLEEPING,
	BULLET_GUI_ENABLE_FRICTION_DIR_CACHING,
	BULLET_GUI_SHARE_SHAPES,
	BULLET_GUI_USE_SPLIT_IMPULSE,
	BULLET_GUI_RANDOMIZE_ORDER,
	BULLET_GUI_WARM_STARTING,
	BULLET_GUI_USE_DBVT,
	BULLET_GUI_USE_CCD,
	//
	BULLET_GUI_SOLVER_ITER,
	BULLET_GUI_LINEAR_DAMPING,
	BULLET_GUI_ANGULAR_DAMPING,
	BULLET_GUI_ERP,
	BULLET_GUI_ERP2,
	BULLET_GUI_COLLISION_MARGIN,
	//
	BULLET_GUI_ENABLE_DEBUG_VIZ,	// MUST BE LAST
};

static void gCheckBoxCallback(const IceCheckBox& check_box, bool checked, void* user_data)
{
	const udword id = check_box.GetID();
	switch(id)
	{
		case BULLET_GUI_ENABLE_SLEEPING:
			gEnableSleeping = checked;
			break;
		case BULLET_GUI_ENABLE_FRICTION_DIR_CACHING:
			gEnableFrictionDirCaching = checked;
			break;
		case BULLET_GUI_SHARE_SHAPES:
			gShareShapes = checked;
			break;
		case BULLET_GUI_USE_SPLIT_IMPULSE:
			gUseSplitImpulse = checked;
			break;
		case BULLET_GUI_RANDOMIZE_ORDER:
			gRandomizeOrder = checked;
			break;
		case BULLET_GUI_WARM_STARTING:
			gWarmStarting = checked;
			break;
		case BULLET_GUI_USE_DBVT:
			gUseDbvt = checked;
			break;
		case BULLET_GUI_USE_CCD:
			gUseCCD = checked;
			break;
		case BULLET_GUI_ENABLE_DEBUG_VIZ:
			{
				gDebugVizParams[0] = checked;
				for(udword i=1;i<NB_DEBUG_VIZ_PARAMS;i++)
				{
					gCheckBox_DebugVis[i]->SetEnabled(checked);
				}
			}
			break;
	}

	if(id>BULLET_GUI_ENABLE_DEBUG_VIZ && id<BULLET_GUI_ENABLE_DEBUG_VIZ+NB_DEBUG_VIZ_PARAMS)
	{
		gDebugVizParams[id-BULLET_GUI_ENABLE_DEBUG_VIZ] = checked;
	}

	//gDebugDrawer.updateDebugMode();
}

static void gBullet_GetOptionsFromGUI()
{
	if(gEditBox_SolverIter)
	{
		sdword tmp;
		bool status = gEditBox_SolverIter->GetTextAsInt(tmp);
		ASSERT(status);
		ASSERT(tmp>=0);
		gSolverIterationCount = udword(tmp);
	}

	if(gEditBox_LinearDamping)
	{
		float tmp;
		bool status = gEditBox_LinearDamping->GetTextAsFloat(tmp);
		ASSERT(status);
		ASSERT(tmp>=0.0f);
		gLinearDamping = tmp;
	}

	if(gEditBox_AngularDamping)
	{
		float tmp;
		bool status = gEditBox_AngularDamping->GetTextAsFloat(tmp);
		ASSERT(status);
		ASSERT(tmp>=0.0f);
		gAngularDamping = tmp;
	}

	if(gEditBox_Erp)
	{
		float tmp;
		bool status = gEditBox_Erp->GetTextAsFloat(tmp);
		ASSERT(status);
		gErp = tmp;
	}

	if(gEditBox_Erp2)
	{
		float tmp;
		bool status = gEditBox_Erp2->GetTextAsFloat(tmp);
		ASSERT(status);
		gErp2 = tmp;
	}

	if(gEditBox_CollisionMargin)
	{
		float tmp;
		bool status = gEditBox_CollisionMargin->GetTextAsFloat(tmp);
		ASSERT(status);
		ASSERT(tmp>=0.0f);
		gCollisionMargin = tmp;
	}
}

IceWindow* ReactPhysics3D_InitGUI(IceWidget* parent, PintGUIHelper& helper)
{
	IceWindow* Main = helper.CreateMainWindow(gBulletGUI, parent, BULLET_GUI_MAIN, "Bullet 2.79 options");

	const sdword YStep = 20;
	const sdword YStepCB = 16;
	sdword y = 4;

	{
		const udword CheckBoxWidth = 200;

		helper.CreateCheckBox(Main, BULLET_GUI_ENABLE_SLEEPING, 4, y, CheckBoxWidth, 20, "Enable sleeping", gBulletGUI, gEnableSleeping, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, BULLET_GUI_ENABLE_FRICTION_DIR_CACHING, 4, y, CheckBoxWidth, 20, "Enable friction dir caching", gBulletGUI, gEnableFrictionDirCaching, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, BULLET_GUI_SHARE_SHAPES, 4, y, CheckBoxWidth, 20, "Share shapes", gBulletGUI, gShareShapes, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, BULLET_GUI_USE_SPLIT_IMPULSE, 4, y, CheckBoxWidth, 20, "Use split impulse", gBulletGUI, gUseSplitImpulse, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, BULLET_GUI_RANDOMIZE_ORDER, 4, y, CheckBoxWidth, 20, "Randomize order", gBulletGUI, gRandomizeOrder, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, BULLET_GUI_WARM_STARTING, 4, y, CheckBoxWidth, 20, "Warm starting", gBulletGUI, gWarmStarting, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, BULLET_GUI_USE_DBVT, 4, y, CheckBoxWidth, 20, "Use DBVT (else SAP)", gBulletGUI, gUseDbvt, gCheckBoxCallback);
		y += YStepCB;

		helper.CreateCheckBox(Main, BULLET_GUI_USE_CCD, 4, y, CheckBoxWidth, 20, "Enable CCD", gBulletGUI, gUseCCD, gCheckBoxCallback);
		y += YStepCB;
	}
	Common_CreateDebugVizUI(Main, 290, 20, gCheckBoxCallback, BULLET_GUI_ENABLE_DEBUG_VIZ, NB_DEBUG_VIZ_PARAMS, gDebugVizParams, gDebugVizNames, gCheckBox_DebugVis, gBulletGUI);

	y += YStep;

	sdword OffsetX = 90;
	const sdword EditBoxWidth = 60;
	const sdword LabelOffsetY = 2;
	{
		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Num solver iter:", gBulletGUI);
		gEditBox_SolverIter = helper.CreateEditBox(Main, BULLET_GUI_SOLVER_ITER, 4+OffsetX, y, EditBoxWidth, 20, _F("%d", gSolverIterationCount), gBulletGUI, EDITBOX_INTEGER_POSITIVE, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Linear damping:", gBulletGUI);
		gEditBox_LinearDamping = helper.CreateEditBox(Main, BULLET_GUI_LINEAR_DAMPING, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gLinearDamping), gBulletGUI, EDITBOX_FLOAT_POSITIVE, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Angular damping:", gBulletGUI);
		gEditBox_AngularDamping = helper.CreateEditBox(Main, BULLET_GUI_ANGULAR_DAMPING, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gAngularDamping), gBulletGUI, EDITBOX_FLOAT_POSITIVE, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "ERP:", gBulletGUI);
		gEditBox_Erp = helper.CreateEditBox(Main, BULLET_GUI_ERP, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gErp), gBulletGUI, EDITBOX_FLOAT, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "ERP2:", gBulletGUI);
		gEditBox_Erp2 = helper.CreateEditBox(Main, BULLET_GUI_ERP2, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gErp2), gBulletGUI, EDITBOX_FLOAT, null);
		y += YStep;

		helper.CreateLabel(Main, 4, y+LabelOffsetY, 90, 20, "Collision margin:", gBulletGUI);
		gEditBox_CollisionMargin = helper.CreateEditBox(Main, BULLET_GUI_COLLISION_MARGIN, 4+OffsetX, y, EditBoxWidth, 20, helper.Convert(gCollisionMargin), gBulletGUI, EDITBOX_FLOAT_POSITIVE, null);
		y += YStep;
	}

	y += YStep;

	return Main;
}

void ReactPhysics3D_CloseGUI()
{
	Common_CloseGUI(gBulletGUI);

	gEditBox_SolverIter = null;
	gEditBox_LinearDamping = null;
	gEditBox_AngularDamping = null;
	gEditBox_Erp = null;
	gEditBox_Erp2 = null;
	gEditBox_CollisionMargin = null;
	for(udword i=0;i<NB_DEBUG_VIZ_PARAMS;i++)
		gCheckBox_DebugVis[i] = null;
}

///////////////////////////////////////////////////////////////////////////////

class ReactPhysics3DPlugIn : public PintPlugin
{
	public:
	virtual	IceWindow*	InitGUI(IceWidget* parent, PintGUIHelper& helper)	{ return ReactPhysics3D_InitGUI(parent, helper);	}
	virtual	void		CloseGUI()											{ ReactPhysics3D_CloseGUI();						}
	virtual	void		Init(const PINT_WORLD_CREATE& desc)					{ ReactPhysics3D_Init(desc);						}
	virtual	void		Close()												{ ReactPhysics3D_Close();							}
	virtual	Pint*		GetPint()											{ return GetReactPhysics3D();						}
};
static ReactPhysics3DPlugIn gPlugIn;

PintPlugin*	GetPintPlugin()
{
	return &gPlugIn;
}


