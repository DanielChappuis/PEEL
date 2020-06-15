///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_REACTPHYSICS3D_H
#define PINT_REACTPHYSICS3D_H

#include "..\Pint.h"
#include <reactphysics3d/reactphysics3d.h>

	class ReactPhysics3D : public Pint
	{
		public:
														ReactPhysics3D();
		virtual											~ReactPhysics3D();

		// Pint
		virtual	const char*								GetName()				const	{ return "ReactPhysics3D 0.8.0";	}
		virtual	void									GetCaps(PintCaps& caps)	const;
		virtual	void									Init(const PINT_WORLD_CREATE& desc);
		virtual	void									SetGravity(const Point& gravity);
		virtual	void									Close();
		virtual	udword									Update(float dt);
		virtual	Point									GetMainColor();
		virtual	void									Render(PintRender& renderer);

		virtual	void									SetDisabledGroups(udword nb_groups, const PintDisabledGroups* groups);
		virtual	PintObjectHandle						CreateObject(const PINT_OBJECT_CREATE& desc);
		virtual	bool									ReleaseObject(PintObjectHandle handle);
		virtual	PintJointHandle							CreateJoint(const PINT_JOINT_CREATE& desc);

		virtual	udword									BatchRaycasts(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintRaycastData* raycasts);
		virtual	udword									BatchBoxSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintBoxSweepData* sweeps);
		virtual	udword									BatchSphereSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintSphereSweepData* sweeps);
		virtual	udword									BatchCapsuleSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintCapsuleSweepData* sweeps);

		virtual	PR										GetWorldTransform(PintObjectHandle handle);
		virtual	void									ApplyActionAtPoint(PintObjectHandle handle, PintActionType action_type, const Point& action, const Point& pos);

		virtual	udword									CreateConvexObject(const PINT_CONVEX_DATA_CREATE& desc);
		virtual	udword									BatchConvexSweeps(PintSQThreadContext context, udword nb, PintRaycastHit* dest, const PintConvexSweepData* sweeps);
		//~Pint

		private:
				rp3d::PhysicsCommon						mPhysicsCommon;
				rp3d::PhysicsWorld*						mPhysicsWorld;
				udword									mGroupMasks[32];

				struct InternalBoxShape
				{
					rp3d::BoxShape*	mShape;
					Point		mExtents;	// Necessary for sharing, as ReactPhysics3D doesn't store the un-scaled extents
				};

				struct InternalMeshShape
				{
					rp3d::ConcaveMeshShape*		mShape;
					rp3d::TriangleMesh*	mMeshData;
				};

				std::vector<rp3d::CollisionShape*>		mCollisionShapes;
				std::vector<rp3d::PolygonVertexArray*>  mPolygonVertexArrays;
				std::vector<rp3d::PolyhedronMesh*>      mPolyhedronMeshes;
				std::vector<rp3d::SphereShape*>			mSphereShapes;
				std::vector<InternalBoxShape>			mBoxShapes;
				std::vector<rp3d::CapsuleShape*>		mCapsuleShapes;
				std::vector<rp3d::ConvexMeshShape*>		mConvexShapes;
				std::vector<InternalMeshShape>			mMeshShapes;

				std::vector<rp3d::ConvexMeshShape*>		mConvexObjects;

				rp3d::CollisionShape*					CreateReactPhysics3DShape(const PINT_SHAPE_CREATE& desc);
				rp3d::SphereShape*						FindSphereShape(const PINT_SPHERE_CREATE& create);
				rp3d::BoxShape*							FindBoxShape(const PINT_BOX_CREATE& create);
				rp3d::CapsuleShape*						FindCapsuleShape(const PINT_CAPSULE_CREATE& create);
				rp3d::ConvexMeshShape*					FindConvexShape(const PINT_CONVEX_CREATE& create);

				std::vector<rp3d::Vector3> computeContactPointsOfWorld() const;
	};

	IceWindow*	ReactPhysics3D_InitGUI(IceWidget* parent, PintGUIHelper& helper);
	void		ReactPhysics3D_CloseGUI();
	void		ReactPhysics3D_Init(const PINT_WORLD_CREATE& desc);
	void		ReactPhysics3D_Close();
	ReactPhysics3D*		GetReactPhysics3D();

	

	extern "C"	__declspec(dllexport)	PintPlugin*	GetPintPlugin();

#endif