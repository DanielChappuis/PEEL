///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Render.h"
#include "TestScenes.h"
#include "TestScenesHelpers.h"
#include "PintObjectsManager.h"
#include "MyConvex.h"
#include "Loader_RepX.h"
#include "Loader_Bin.h"
#include "Random.h"
#include "GUI_Helpers.h"

///////////////////////////////////////////////////////////////////////////////

class SceneSweepVsShapes : public TestBase
{
	public:
							SceneSweepVsShapes() : mQueryShapeType(PINT_SHAPE_UNDEFINED), mSceneShapeType(PINT_SHAPE_UNDEFINED), mTypeData(4), mSize(32), mDynamic(false)	{}
	virtual					~SceneSweepVsShapes()			{}
	virtual	TestCategory	GetCategory()			const	{ return CATEGORY_SWEEP;	}
	virtual	bool			ProfileUpdate()					{ return true;				}

	virtual void			GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(50.00f, 50.00f, 50.00f), Point(-0.59f, -0.55f, -0.59f));
		desc.mCamera[1] = CameraPose(Point(52.39f, 11.74f, 47.06f), Point(-0.69f, -0.17f, 0.70f));
	}

	virtual bool			CommonSetup()
	{
		mCreateDefaultEnvironment = false;
		TestBase::CommonSetup();
		if(mQueryShapeType==PINT_SHAPE_BOX)
			return GenerateArrayOfVerticalBoxSweeps(*this, 50.0f, mSize, mSize, gSQMaxDist);
		if(mQueryShapeType==PINT_SHAPE_SPHERE)
			return GenerateArrayOfVerticalSphereSweeps(*this, 50.0f, mSize, mSize, gSQMaxDist);
		if(mQueryShapeType==PINT_SHAPE_CAPSULE)
			return GenerateArrayOfVerticalCapsuleSweeps(*this, 50.0f, mSize, mSize, gSQMaxDist);
		return true;
	}

	virtual bool			Setup(Pint& pint, const PintCaps& caps)
	{
		if(mQueryShapeType==PINT_SHAPE_BOX && !caps.mSupportBoxSweeps)
			return false;
		if(mQueryShapeType==PINT_SHAPE_SPHERE && !caps.mSupportSphereSweeps)
			return false;
		if(mQueryShapeType==PINT_SHAPE_CAPSULE && !caps.mSupportCapsuleSweeps)
			return false;

		const float Altitude = 10.0f;
		const float Scale = 50.0f;
		const float Mass = mDynamic ? 1.0f : 0.0f;
		return GenerateArrayOfObjects(pint, caps, mSceneShapeType, mTypeData, mSize, mSize, Altitude, Scale, Mass);
	}

	virtual void			CommonUpdate(float dt)
	{
		TestBase::CommonUpdate(dt);
		if(mQueryShapeType==PINT_SHAPE_BOX)
			UpdateBoxSweeps(*this, mCurrentTime);
//		if(mQueryShapeType==PINT_SHAPE_SPHERE)
//			UpdateSphereSweeps(*this, mCurrentTime);
		if(mQueryShapeType==PINT_SHAPE_CAPSULE)
			UpdateCapsuleSweeps(*this, mCurrentTime);
	}

	virtual	udword			Update(Pint& pint, float dt)
	{
		if(mQueryShapeType==PINT_SHAPE_BOX)
			return DoBatchBoxSweeps(*this, pint);
		if(mQueryShapeType==PINT_SHAPE_SPHERE)
			return DoBatchSphereSweeps(*this, pint);
		if(mQueryShapeType==PINT_SHAPE_CAPSULE)
			return DoBatchCapsuleSweeps(*this, pint);
		return 0;
	}

			PintShape		mQueryShapeType;
			PintShape		mSceneShapeType;
			udword			mTypeData;
			udword			mSize;
			bool			mDynamic;
};

static const char* gDesc_SceneSweepsVsShapes = "(Configurable test) - a grid of shape-sweeps against a grid of static or dynamic shapes. Select undefined scene-shape to create an empty scene and measure the operating overhead of shape-sweeps.";

class ConfigurableSceneSweepVsShapes : public SceneSweepVsShapes
{
			IceComboBox*	mComboBox_ConvexIndex;
			IceComboBox*	mComboBox_QueryShapeType;
			IceComboBox*	mComboBox_SceneShapeType;
			IceEditBox*		mEditBox_Size;
			IceCheckBox*	mCheckBox_Dynamic;
	public:
							ConfigurableSceneSweepVsShapes() :
								mComboBox_ConvexIndex	(null),
								mComboBox_QueryShapeType(null),
								mComboBox_SceneShapeType(null),
								mEditBox_Size			(null),
								mCheckBox_Dynamic		(null)	{}
	virtual					~ConfigurableSceneSweepVsShapes()	{							}
	virtual	const char*		GetName()			const	{ return "SceneSweepVsShapes";		}
	virtual	const char*		GetDescription()	const	{ return gDesc_SceneSweepsVsShapes;	}

	virtual	void			InitUI(PintGUIHelper& helper)
	{
		WindowDesc WD;
		WD.mParent	= null;
		WD.mX		= 50;
		WD.mY		= 50;
		WD.mWidth	= 300;
		WD.mHeight	= 200;
		WD.mLabel	= "SceneSweepVsShapes";
		WD.mType	= WINDOW_DIALOG;
		IceWindow* UI = ICE_NEW(IceWindow)(WD);
		RegisterUIElement(UI);
		UI->SetVisible(true);

		Container* UIElems = GetUIElements();

		const sdword EditBoxWidth = 60;
		const sdword LabelWidth = 100;
		const sdword OffsetX = LabelWidth + 10;
		const sdword LabelOffsetY = 2;
		const sdword YStep = 20;
		sdword y = 0;
		{
			mCheckBox_Dynamic = helper.CreateCheckBox(UI, 0, 4, y, 400, 20, "Dynamic scene shapes", UIElems, false, null, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Grid size:", UIElems);
			mEditBox_Size = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "32", UIElems, EDITBOX_INTEGER_POSITIVE, null, null);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Query shape:", UIElems);
			mComboBox_QueryShapeType = CreateShapeTypeComboBox(UI, 4+OffsetX, y, true, SSM_UNDEFINED|SSM_SPHERE|SSM_CAPSULE|SSM_BOX);
			RegisterUIElement(mComboBox_QueryShapeType);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Scene shape:", UIElems);
			mComboBox_SceneShapeType = CreateShapeTypeComboBox(UI, 4+OffsetX, y, true, SSM_UNDEFINED|SSM_SPHERE|SSM_CAPSULE|SSM_BOX|SSM_CONVEX);
			RegisterUIElement(mComboBox_SceneShapeType);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Convex:", UIElems);
			mComboBox_ConvexIndex = CreateConvexObjectComboBox(UI, 4+OffsetX, y, true);
			RegisterUIElement(mComboBox_ConvexIndex);
			y += YStep;
		}

		y += YStep;
		AddResetButton(UI, 4, y, 300-16);
	}

	virtual bool	CommonSetup()
	{
		mDynamic = mCheckBox_Dynamic ? mCheckBox_Dynamic->IsChecked() : false;

		mSize = GetFromEditBox(mSize, mEditBox_Size);

		if(mComboBox_QueryShapeType)
			mQueryShapeType = PintShape(mComboBox_QueryShapeType->GetSelectedIndex());

		if(mComboBox_SceneShapeType)
			mSceneShapeType = PintShape(mComboBox_SceneShapeType->GetSelectedIndex());

		if(mComboBox_ConvexIndex)
			mTypeData = ConvexIndex(mComboBox_ConvexIndex->GetSelectedIndex());

		return SceneSweepVsShapes::CommonSetup();
	}

}ConfigurableSceneSweepVsShapes;

///////////////////////////////////////////////////////////////////////////////

static PintShapeRenderer* CommonSetup_SceneConvexSweep(MyConvex& convex, TestBase& test, udword nb_x, udword nb_y, float altitude, float scale_x, float scale_y, const Point& dir, const Point& offset, float max_dist, udword convex_id)
{
	convex.LoadFile(convex_id);

	PintShapeRenderer* renderer = CreateConvexRenderer(convex.mNbVerts, convex.mVerts);

	Matrix3x3 Rot;
	Rot.Identity();

	const float OneOverNbX = OneOverNb(nb_x);
	const float OneOverNbY = OneOverNb(nb_y);
	for(udword y=0;y<nb_y;y++)
	{
		const float CoeffY = 2.0f * ((float(y)*OneOverNbY) - 0.5f);
		for(udword x=0;x<nb_x;x++)
		{
			const float CoeffX = 2.0f * ((float(x)*OneOverNbX) - 0.5f);

			const Point Origin(offset.x + CoeffX * scale_x, offset.y + altitude, offset.z + CoeffY * scale_y);

			test.RegisterConvexSweep(0, renderer, PR(Origin, Rot), dir, max_dist);
		}
	}

	test.mCreateDefaultEnvironment = false;
	return renderer;
}

static bool Setup_SceneConvexSweep(PintShapeRenderer* renderer, MyConvex& convex, TestBase& test, Pint& pint, const PintCaps& caps)
{
	if(!caps.mSupportConvexSweeps)
		return false;

	PINT_CONVEX_DATA_CREATE Desc(convex.mNbVerts, convex.mVerts);
	Desc.mRenderer	= renderer;
	const udword h = pint.CreateConvexObject(Desc);
	ASSERT(h==0);

	return true;
}

static const char* gDesc_SceneConvexSweepVsStaticConvexes = "(Configurable test) - a grid of convex-sweeps against a grid of static convexes.";

class SceneConvexSweepVsStaticConvexes : public SceneSweepVsShapes
{
			MyConvex			mConvex;
			PintShapeRenderer*	mRenderer;

			IceComboBox*		mComboBox_ConvexIndex;
			IceEditBox*			mEditBox_Size;
			sdword				mGridSize;
	public:
								SceneConvexSweepVsStaticConvexes() :
									mRenderer				(null),
									mComboBox_ConvexIndex	(null),
									mEditBox_Size			(null),
									mGridSize				(0)
									{
										mSceneShapeType = PINT_SHAPE_CONVEX;
										mDynamic = false;
										mTypeData = CONVEX_INDEX_0;
									}

	virtual	const char*		GetName()			const	{ return "SceneConvexSweepVsStaticConvexes";		}
	virtual	const char*		GetDescription()	const	{ return gDesc_SceneConvexSweepVsStaticConvexes;	}

	virtual	void			InitUI(PintGUIHelper& helper)
	{
		WindowDesc WD;
		WD.mParent	= null;
		WD.mX		= 50;
		WD.mY		= 50;
		WD.mWidth	= 256;
		WD.mHeight	= 160;
		WD.mLabel	= "SceneConvexSweepVsStaticConvexes";
		WD.mType	= WINDOW_DIALOG;
		IceWindow* UI = ICE_NEW(IceWindow)(WD);
		RegisterUIElement(UI);
		UI->SetVisible(true);

		Container* UIElems = GetUIElements();

		const sdword OffsetX = 70;
		const sdword EditBoxWidth = 60;
		const sdword LabelWidth = 60;
		const sdword LabelOffsetY = 2;
		const sdword YStep = 20;
		sdword y = 0;
		{
			y += YStep;
			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Convex:", UIElems);
			mComboBox_ConvexIndex = CreateConvexObjectComboBox(UI, 4+OffsetX, y, true);
			RegisterUIElement(mComboBox_ConvexIndex);
			y += YStep;
		}

		{
			y += YStep;
			{
				helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Grid size:", UIElems);
				mEditBox_Size = helper.CreateEditBox(UI, 1, 4+OffsetX, y, EditBoxWidth, 20, "32", UIElems, EDITBOX_INTEGER_POSITIVE, null, null);
				y += YStep;
			}
		}

		y += YStep;
		AddResetButton(UI, 4, y, 256-16);
	}

	virtual void				CommonRelease()
	{
		mConvex.Release();
		SceneSweepVsShapes::CommonRelease();
	}

	virtual bool				CommonSetup()
	{
		SceneSweepVsShapes::CommonSetup();

		ConvexIndex Index = CONVEX_INDEX_0;
		if(mComboBox_ConvexIndex)
			Index = ConvexIndex(mComboBox_ConvexIndex->GetSelectedIndex());
		mTypeData = Index;

		const udword Size = GetFromEditBox(32, mEditBox_Size);
		mGridSize = Size;

		const float Altitude = 30.0f;
		const Point Dir(0.0f, -1.0f, 0.0f);
		const Point Offset(0.0f, 0.0f, 0.0f);
		mRenderer = CommonSetup_SceneConvexSweep(mConvex, *this, Size, Size, Altitude, 50.0f, 50.0f, Dir, Offset, gSQMaxDist, mTypeData);
		return mRenderer!=null;
	}

	virtual bool				Setup(Pint& pint, const PintCaps& caps)
	{
		if(!Setup_SceneConvexSweep(mRenderer, mConvex, *this, pint, caps))
			return false;

		return GenerateArrayOfConvexes(pint, caps, true, 50.0f, mGridSize, mGridSize, mTypeData);
	}

	virtual void				CommonUpdate(float dt)
	{
		SceneSweepVsShapes::CommonUpdate(dt);
		UpdateConvexSweeps(*this, mCurrentTime);
	}

	virtual udword				Update(Pint& pint, float dt)
	{
		return DoBatchConvexSweeps(*this, pint);
	}
}SceneConvexSweepVsStaticConvexes;

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SweepAccuracy = "Tests the accuracy of linear sweep tests. Ideally all engines should return the same impact distance.";

START_SQ_TEST(SweepAccuracy, CATEGORY_SWEEP, gDesc_SweepAccuracy)

	virtual void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(8.55f, -0.99f, 0.74f), Point(-0.93f, 0.37f, -0.07f));
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportCapsuleSweeps)
			return false;

			const float Radius = 0.5f;
			const float HalfHeight_ = 1.0f;
	/*
			PINT_CAPSULE_CREATE ShapeDesc(Radius, HalfHeight_);
			ShapeDesc.mRenderer		= CreateCapsuleRenderer(Radius, HalfHeight_*2.0f);
	*/

	/*
			PINT_SPHERE_CREATE ShapeDesc(Radius);
			ShapeDesc.mRenderer		= CreateSphereRenderer(Radius);
	*/
			PINT_BOX_CREATE ShapeDesc(Radius, Radius, Radius);
			ShapeDesc.mRenderer		= CreateBoxRenderer(ShapeDesc.mExtents);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes			= &ShapeDesc;
			ObjectDesc.mMass			= 0.0f;
			ObjectDesc.mPosition.Zero();
			ObjectDesc.mRotation = Quat(1.1f, 2.2f, 3.3f, 4.4f);
			ObjectDesc.mRotation.Normalize();
			CreatePintObject(pint, ObjectDesc);

		const Point Dir(0.0f, -1.0f, 0.0f);
		const float CapsuleRadius = 1.4f;
		const float HalfHeight = 1.8f;

		UnregisterAllCapsuleSweeps();
		const Point P0(HalfHeight, 10.0f, HalfHeight);
		const Point P1(-HalfHeight, 10.0f, -HalfHeight);
		RegisterCapsuleSweep(LSS(Segment(P0, P1), CapsuleRadius), Dir, gSQMaxDist);

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual void	CommonUpdate(float dt)
	{
		TestBase::CommonUpdate(dt);

		const float Offset = sinf(mCurrentTime)*50.0f;

		const float Altitude = 100.0f;
		const float HalfHeight = 1.8f;
		udword Nb = GetNbRegisteredCapsuleSweeps();
		PintCapsuleSweepData* Data = GetRegisteredCapsuleSweeps();
		while(Nb--)
		{
			const Point P0(1.3f + HalfHeight, Altitude+Offset, HalfHeight);
			const Point P1(1.3f + -HalfHeight, Altitude+Offset, -HalfHeight);

			Data->mCapsule.mP0 = P0;
			Data->mCapsule.mP1 = P1;
			Data++;
		}
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchCapsuleSweeps(*this, pint);
	}

END_TEST(SweepAccuracy)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SweepAccuracy2 = "Tests the accuracy of linear sweep tests. Ideally all engines should return the same impact distance.";

START_SQ_TEST(SweepAccuracy2, CATEGORY_SWEEP, gDesc_SweepAccuracy2)

	virtual void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(10.42f, 0.02f, 5.22f), Point(-0.82f, 0.15f, -0.55f));
	}

	virtual bool	CommonSetup()
	{
		CreateSingleTriangleMesh(*this, 5000.0f);
		mCreateDefaultEnvironment = false;
		return TestBase::CommonSetup();
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportMeshes || !caps.mSupportCapsuleSweeps)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

	/*		const float Radius = 0.5f;
			const float HalfHeight_ = 1.0f;
			PINT_BOX_CREATE ShapeDesc(Radius, Radius, Radius);
			ShapeDesc.mRenderer		= CreateBoxRenderer(ShapeDesc.mExtents);

			PINT_OBJECT_CREATE ObjectDesc;
			ObjectDesc.mShapes			= &ShapeDesc;
			ObjectDesc.mMass			= 0.0f;
			ObjectDesc.mPosition.Zero();
			ObjectDesc.mRotation = Quat(1.1f, 2.2f, 3.3f, 4.4f);
			ObjectDesc.mRotation.Normalize();
			CreatePintObject(pint, ObjectDesc);
	*/

		const Point Dir(0.0f, -1.0f, 0.0f);
		const float CapsuleRadius = 1.4f;
		const float HalfHeight = 1.8f;

		UnregisterAllCapsuleSweeps();
		const Point P0(HalfHeight, 10.0f, HalfHeight);
		const Point P1(-HalfHeight, 10.0f, -HalfHeight);
		RegisterCapsuleSweep(LSS(Segment(P0, P1), CapsuleRadius), Dir, gSQMaxDist);
		return true;
	}

	virtual void	CommonUpdate(float dt)
	{
		TestBase::CommonUpdate(dt);

		const float Offset = sinf(mCurrentTime)*50.0f;

		const float Altitude = 100.0f;
		const float HalfHeight = 1.8f;
		udword Nb = GetNbRegisteredCapsuleSweeps();
		PintCapsuleSweepData* Data = GetRegisteredCapsuleSweeps();
		while(Nb--)
		{
			const Point P0(1.3f + HalfHeight, Altitude+Offset, HalfHeight);
			const Point P1(1.3f + -HalfHeight, Altitude+Offset, -HalfHeight);

			Data->mCapsule.mP0 = P0;
			Data->mCapsule.mP1 = P1;
			Data++;
		}
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchCapsuleSweeps(*this, pint);
	}

END_TEST(SweepAccuracy2)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneLongSweepVsSeaOfStatics = "(Configurable test) - Long diagonal shape-sweep against a sea of static boxes.";
class SceneLongSweepVsSeaOfStatics : public TestBase
{
			PintShape		mQueryShapeType;
			IceComboBox*	mComboBox_QueryShapeType;
	public:
							SceneLongSweepVsSeaOfStatics() :
								mQueryShapeType			(PINT_SHAPE_UNDEFINED),
								mComboBox_QueryShapeType(null)	{}
	virtual					~SceneLongSweepVsSeaOfStatics()		{									}
	virtual	const char*		GetName()			const	{ return "SceneLongSweepVsSeaOfStatics";	}
	virtual	const char*		GetDescription()	const	{ return gDesc_SceneLongSweepVsSeaOfStatics;}
	virtual	TestCategory	GetCategory()		const	{ return CATEGORY_SWEEP;					}
	virtual	bool			ProfileUpdate()				{ return true;								}

	virtual void			GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(40.53f, 3.74f, 42.89f), Point(0.15f, -0.89f, -0.43f));
		desc.mCamera[1] = CameraPose(Point(54.50f, 10.80f, 72.83f), Point(-0.59f, -0.11f, -0.80f));
	}

	virtual	void			InitUI(PintGUIHelper& helper)
	{
		WindowDesc WD;
		WD.mParent	= null;
		WD.mX		= 50;
		WD.mY		= 50;
		WD.mWidth	= 300;
		WD.mHeight	= 100;
		WD.mLabel	= "SceneLongSweepVsSeaOfStatics";
		WD.mType	= WINDOW_DIALOG;
		IceWindow* UI = ICE_NEW(IceWindow)(WD);
		RegisterUIElement(UI);
		UI->SetVisible(true);

		Container* UIElems = GetUIElements();

		const sdword EditBoxWidth = 60;
		const sdword LabelWidth = 100;
		const sdword OffsetX = LabelWidth + 10;
		const sdword LabelOffsetY = 2;
		const sdword YStep = 20;
		sdword y = 0;
		{
			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Query shape:", UIElems);
			mComboBox_QueryShapeType = CreateShapeTypeComboBox(UI, 4+OffsetX, y, true, SSM_UNDEFINED|SSM_SPHERE|SSM_CAPSULE|SSM_BOX);
			RegisterUIElement(mComboBox_QueryShapeType);
			mComboBox_QueryShapeType->Select(PINT_SHAPE_BOX);
			y += YStep;
		}

		y += YStep;
		AddResetButton(UI, 4, y, 300-16);
	}

	virtual bool	CommonSetup()
	{
		mCreateDefaultEnvironment = false;
		TestBase::CommonSetup();

		if(mComboBox_QueryShapeType)
			mQueryShapeType = PintShape(mComboBox_QueryShapeType->GetSelectedIndex());

		if(mQueryShapeType==PINT_SHAPE_SPHERE)
		{
			Point Dir(1.f, 0.0f, 1.0f);
			Dir.Normalize();

			const Point Origin(50.0f, 0.0f, 50.0f);

			RegisterSphereSweep(Sphere(Origin, 1.0f), -Dir, 2000.0f);
			RegisterSphereSweep(Sphere(Origin + Point(0.0f, 10.0f, 0.0f), 1.0f), -Dir, 2000.0f);
		}
		else if(mQueryShapeType==PINT_SHAPE_CAPSULE)
		{
			Point Dir(1.f, 0.0f, 1.0f);
			Dir.Normalize();

			const Point Origin(50.0f, 0.0f, 50.0f);

			const Point Ext(2.0f, 0.0f, 0.0f);

			RegisterCapsuleSweep(LSS(Segment(Origin-Ext, Origin+Ext), 1.0f), -Dir, 2000.0f);

			const Point C = Origin + Point(0.0f, 10.0f, 0.0f);
			RegisterCapsuleSweep(LSS(Segment(C-Ext, C+Ext), 1.0f), -Dir, 2000.0f);
		}
		else if(mQueryShapeType==PINT_SHAPE_BOX)
		{
			Point Dir(1.f, 0.0f, 1.0f);
			Dir.Normalize();

			const Point Origin(50.0f, 0.0f, 50.0f);

			OBB Box;
			Box.mCenter		= Origin;
			Box.mExtents	= Point(1.0f, 1.0f, 1.0f);
			Box.mRot.Identity();

			RegisterBoxSweep(Box, -Dir, 2000.0f);

			Box.mCenter		= Origin + Point(0.0f, 10.0f, 0.0f);
			RegisterBoxSweep(Box, -Dir, 2000.0f);
		}
		return true;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(mQueryShapeType==PINT_SHAPE_SPHERE && !caps.mSupportSphereSweeps)
			return false;
		if(mQueryShapeType==PINT_SHAPE_CAPSULE && !caps.mSupportCapsuleSweeps)
			return false;
		if(mQueryShapeType==PINT_SHAPE_BOX && !caps.mSupportBoxSweeps)
			return false;

		return CreateSeaOfStaticBoxes(pint, 40.0f, 128, 128, 0.0f);
	}

	virtual void	CommonUpdate(float dt)
	{
		TestBase::CommonUpdate(dt);
		if(mQueryShapeType==PINT_SHAPE_CAPSULE)
			UpdateCapsuleSweeps(*this, mCurrentTime);
		else if(mQueryShapeType==PINT_SHAPE_BOX)
			UpdateBoxSweeps(*this, mCurrentTime);
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		if(mQueryShapeType==PINT_SHAPE_SPHERE)
			return DoBatchSphereSweeps(*this, pint);
		if(mQueryShapeType==PINT_SHAPE_CAPSULE)
			return DoBatchCapsuleSweeps(*this, pint);
		if(mQueryShapeType==PINT_SHAPE_BOX)
			return DoBatchBoxSweeps(*this, pint);
		return 0;
	}

}SceneLongSweepVsSeaOfStatics;

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneShapeSweepVsSingleTriangle = "(Configurable test) - one shape-sweep against a single triangle, various configurations.";
class ConfigurableSceneSweepVsSingleTriangle : public TestBase
{
	enum TestMode
	{
		TEST_VERTICAL_SWEEP,
		TEST_PARALLEL_SWEEP,
		TEST_INITIAL_OVERLAP,
	};

			MyConvex			mConvex;
			PintShapeRenderer*	mRenderer;

			Triangle			mTriangle;
			PintShape			mQueryShapeType;
			TestMode			mTestMode;
			IceComboBox*		mComboBox_QueryShapeType;
			IceComboBox*		mComboBox_TestMode;
			IceCheckBox*		mCheckBox_DoubleSided;
			bool				mDoubleSided;
	public:
							ConfigurableSceneSweepVsSingleTriangle() :
										mRenderer					(null),
										mQueryShapeType				(PINT_SHAPE_UNDEFINED),
										mTestMode					(TEST_VERTICAL_SWEEP),
										mComboBox_QueryShapeType	(null),
										mComboBox_TestMode			(null),
										mCheckBox_DoubleSided		(null),
										mDoubleSided				(false)
																		{								}
	virtual					~ConfigurableSceneSweepVsSingleTriangle()	{								}
	virtual	const char*		GetName()			const	{ return "SceneSweepVsSingleTriangle";			}
	virtual	const char*		GetDescription()	const	{ return gDesc_SceneShapeSweepVsSingleTriangle;	}
	virtual	TestCategory	GetCategory()		const	{ return CATEGORY_SWEEP;						}
	virtual	bool			ProfileUpdate()				{ return true;									}

	virtual void			GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(1.50f, 0.41f, 6.99f), Point(-0.16f, 0.15f, -0.98f));
	}

	virtual	void			InitUI(PintGUIHelper& helper)
	{
		WindowDesc WD;
		WD.mParent	= null;
		WD.mX		= 50;
		WD.mY		= 50;
		WD.mWidth	= 300;
		WD.mHeight	= 160;
		WD.mLabel	= "SceneSweepVsSingleTriangle";
		WD.mType	= WINDOW_DIALOG;
		IceWindow* UI = ICE_NEW(IceWindow)(WD);
		RegisterUIElement(UI);
		UI->SetVisible(true);

		Container* UIElems = GetUIElements();

		const sdword EditBoxWidth = 60;
		const sdword LabelWidth = 100;
		const sdword OffsetX = LabelWidth + 10;
		const sdword LabelOffsetY = 2;
		const sdword YStep = 20;
		sdword y = 10;
		{
			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Configuration:", UIElems);
			{
				ComboBoxDesc CBBD;
				CBBD.mID		= 0;
				CBBD.mParent	= UI;
				CBBD.mX			= 4+OffsetX;
				CBBD.mY			= y;
				CBBD.mWidth		= 150;
				CBBD.mHeight	= 20;
				CBBD.mLabel		= "Configuration";
				IceComboBox* CB = ICE_NEW(IceComboBox)(CBBD);
				CB->Add("Vertical sweep");
				CB->Add("Parallel sweep");
				CB->Add("Initial overlap");
				CB->Select(TEST_VERTICAL_SWEEP);
				CB->SetVisible(true);
				mComboBox_TestMode = CB;
			}
			RegisterUIElement(mComboBox_TestMode);
			y += YStep;

			helper.CreateLabel(UI, 4, y+LabelOffsetY, LabelWidth, 20, "Query shape:", UIElems);
			mComboBox_QueryShapeType = CreateShapeTypeComboBox(UI, 4+OffsetX, y, true, SSM_UNDEFINED|SSM_SPHERE|SSM_CAPSULE|SSM_BOX|SSM_CONVEX);
			RegisterUIElement(mComboBox_QueryShapeType);
			mComboBox_QueryShapeType->Select(PINT_SHAPE_CAPSULE);
			y += YStep;

			mCheckBox_DoubleSided = helper.CreateCheckBox(UI, 0, 4, y, 400, 20, "Flip triangle winding", UIElems, false, null, null);
			y += YStep;
		}

		y += YStep;
		AddResetButton(UI, 4, y, 300-16);
	}

	virtual void				CommonRelease()
	{
		mConvex.Release();
		TestBase::CommonRelease();
	}

	virtual bool			CommonSetup()
	{
		mDoubleSided = mCheckBox_DoubleSided ? mCheckBox_DoubleSided->IsChecked() : false;

		CreateSingleTriangleMesh(*this, 4.0f, &mTriangle, mDoubleSided);
		mCreateDefaultEnvironment = false;

		if(mComboBox_QueryShapeType)
			mQueryShapeType = PintShape(mComboBox_QueryShapeType->GetSelectedIndex());

		if(mComboBox_TestMode)
			mTestMode = TestMode(mComboBox_TestMode->GetSelectedIndex());

		if(mQueryShapeType==PINT_SHAPE_BOX)
		{
			Matrix3x3 Rot;
			Rot.Identity();
			Point Dir, P;
			if(mTestMode==TEST_VERTICAL_SWEEP)
			{
				Dir = Point(0.0f, -1.0f, 0.0f);
				P = Point(0.0f, 10.0f, 0.0f);
			}
			else if(mTestMode==TEST_PARALLEL_SWEEP)
			{
				Dir = Point(1.0f, 0.0f, 0.0f);
				P = Point(-10.0f, 0.1f, 0.0f);
			}
			else if(mTestMode==TEST_INITIAL_OVERLAP)
			{
				Dir = Point(0.0f, -1.0f, 0.0f);
				P = Point(0.0f, 0.1f, 0.0f);
			}
			RegisterBoxSweep(OBB(P, Point(0.4f, 2.0f, 0.4f), Rot), Dir, gSQMaxDist);
		}
		else if(mQueryShapeType==PINT_SHAPE_SPHERE)
		{
			if(0)
			{
				const Triangle& T = mTriangle;

				Point TriCenter;
				T.Center(TriCenter);

				const Point D0 = (T.mVerts[0] - TriCenter).Normalize();
				const Point D1 = (T.mVerts[1] - TriCenter).Normalize();
				const Point D2 = (T.mVerts[2] - TriCenter).Normalize();

				const Point E01 = (T.mVerts[0] + T.mVerts[1])*0.5f;
				const Point E12 = (T.mVerts[1] + T.mVerts[2])*0.5f;
				const Point E20 = (T.mVerts[2] + T.mVerts[0])*0.5f;

				const Point D01 = (E01 - TriCenter).Normalize();
				const Point D12 = (E12 - TriCenter).Normalize();
				const Point D20 = (E20 - TriCenter).Normalize();
			}

			const float Radius = 0.4f;
			if(mTestMode==TEST_VERTICAL_SWEEP)
			{
				const Point Dir(0.0f, -1.0f, 0.0f);
				const Point P(0.0f, 10.0f, 0.0f);
				RegisterSphereSweep(Sphere(P, Radius), Dir, gSQMaxDist);
			}
			else if(mTestMode==TEST_PARALLEL_SWEEP)
			{
				const Point Dir(1.0f, 0.0f, 0.0f);
				const Point P(-10.0f, 0.1f, 0.0f);
				RegisterSphereSweep(Sphere(P, Radius), Dir, gSQMaxDist);
			}
			else if(mTestMode==TEST_INITIAL_OVERLAP)
			{
		//		const Point Dir(0.0f, -1.0f, 0.0f);
				Point Dir(-1.0f, -1.0f, -1.0f);
				Dir.Normalize();

				const Point P(0.0f, 0.1f, 0.0f);
		//		const Point P(0.0f, -0.1f, 0.0f);
		//		const Point P(0.0f, -Radius+0.01f, 0.0f);
		//		const Point P(0.0f, -Radius-0.1f, 0.0f);
		//		const Point P = (T.mVerts[0] + D0*0.2f) + Point(0.0f, 0.1f, 0.0f);
		//		const Point P = (T.mVerts[1] + D1*0.2f) + Point(0.0f, 0.1f, 0.0f);
		//		const Point P = (E01 + D01*0.2f) + Point(0.0f, 0.1f, 0.0f);
				RegisterSphereSweep(Sphere(P, Radius), Dir, gSQMaxDist);

				BasicRandom Rnd(42);
				for(udword i=0;i<4095;i++)
				{
					UnitRandomPt(Dir, Rnd);
					Dir.y = -fabsf(Dir.y);
					RegisterSphereSweep(Sphere(P, Radius), Dir, gSQMaxDist);
				}
			}
		}
		else if(mQueryShapeType==PINT_SHAPE_CAPSULE)
		{
			Point Dir, P0, P1;
//			const float CapsuleRadius = 1.4f;
			const float CapsuleRadius = 0.4f;
			const float HalfHeight = 1.8f;
			const float CapsuleAltitude = 0.1f;
//			const float CapsuleAltitude = 1.5f;
			if(mTestMode==TEST_VERTICAL_SWEEP)
			{
				Dir = Point(0.0f, -1.0f, 0.0f);
				P0 = Point(HalfHeight, 10.0f, HalfHeight);
				P1 = Point(-HalfHeight, 10.0f, -HalfHeight);
			}
			else if(mTestMode==TEST_PARALLEL_SWEEP)
			{
				Dir = Point(1.0f, 0.0f, 0.0f);
				P0 = Point(HalfHeight-10.0f, CapsuleAltitude, HalfHeight);
				P1 = Point(-HalfHeight-10.0f, CapsuleAltitude, -HalfHeight);
			}
			else if(mTestMode==TEST_INITIAL_OVERLAP)
			{
				Dir = Point(0.0f, -1.0f, 0.0f);
				P0 = Point(HalfHeight, CapsuleAltitude, HalfHeight);
				P1 = Point(-HalfHeight, CapsuleAltitude, -HalfHeight);
			}
			RegisterCapsuleSweep(LSS(Segment(P0, P1), CapsuleRadius), Dir, gSQMaxDist);
		}
		else if(mQueryShapeType==PINT_SHAPE_CONVEX)
		{
			Matrix3x3 Rot;
			Rot.Identity();
			Point Dir, P;
			if(mTestMode==TEST_VERTICAL_SWEEP)
			{
				Dir = Point(0.0f, -1.0f, 0.0f);
				P = Point(0.0f, 10.0f, 0.0f);
			}
			else if(mTestMode==TEST_PARALLEL_SWEEP)
			{
				Dir = Point(1.0f, 0.0f, 0.0f);
				P = Point(-10.0f, 0.1f, 0.0f);
			}
			else if(mTestMode==TEST_INITIAL_OVERLAP)
			{
				Dir = Point(0.0f, -1.0f, 0.0f);
				P = Point(0.0f, 0.1f, 0.0f);
			}

		//	udword i=2;	// Small convexes
			udword i=4;	// 'Big' convexes
		//	udword i=7;
		//	udword i=13;
			mConvex.LoadFile(i);

			// There's a design issue for convex sweeps. The sweeps now need data from each Pint (the convex mesh), which
			// was not the case before. Creating things in CommonSetup() doesn't work since there's no Pint pointer. Creating
			// things in Setup() doesn't work since we UnregisterAllConvexSweeps() and the previous Pint-provided objects are
			// lost. We end up calling all Pint plugins with the convex data from the last Pint. Which happens to work just
			// fine between PhysX 3.3 and 3.4, which is mental (we send a PxConvexMesh created in the 3.3 plugin to the 3.4 SDK
			// and things just work. Madness!)
			//
			// The workaround is to create things in CommonSetup while using the future convex indices created later in Setup.
			// For example here we use "0", and we expect "pint.CreateConvexObject()" to later return 0 for each PINT plugin.
			mRenderer = CreateConvexRenderer(mConvex.mNbVerts, mConvex.mVerts);
			RegisterConvexSweep(0, mRenderer, PR(P, Rot), Dir, gSQMaxDist);
		}
		return TestBase::CommonSetup();
	}

	virtual bool			Setup(Pint& pint, const PintCaps& caps)
	{
		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		if(mQueryShapeType==PINT_SHAPE_BOX)
			return caps.mSupportBoxSweeps;

		if(mQueryShapeType==PINT_SHAPE_SPHERE)
			return caps.mSupportSphereSweeps;

		if(mQueryShapeType==PINT_SHAPE_CAPSULE)
			return caps.mSupportCapsuleSweeps;

		if(mQueryShapeType==PINT_SHAPE_CONVEX)
		{
			if(!caps.mSupportConvexSweeps)
				return false;

			PINT_CONVEX_DATA_CREATE Desc(mConvex.mNbVerts, mConvex.mVerts);
			Desc.mRenderer	= mRenderer;
			const udword h = pint.CreateConvexObject(Desc);
			ASSERT(h==0);
			return true;
		}

		return false;
	}

	virtual void			CommonUpdate(float dt)
	{
		mCurrentTime += 0.5f/60.0f;
//		mCurrentTime = 20.0f/60.0f;		// Enable this for the static version corresponding to the debug render code
		if(mQueryShapeType==PINT_SHAPE_BOX)
			UpdateBoxSweeps(*this, mCurrentTime);
//		else if(mQueryShapeType==PINT_SHAPE_SPHERE)
//			UpdateSphereSweeps(*this, mCurrentTime);
		else if(mQueryShapeType==PINT_SHAPE_CAPSULE)
			UpdateCapsuleSweeps(*this, mCurrentTime);
		else if(mQueryShapeType==PINT_SHAPE_CONVEX)
			UpdateConvexSweeps(*this, mCurrentTime);
	}

	virtual udword			Update(Pint& pint, float dt)
	{
		if(mQueryShapeType==PINT_SHAPE_BOX)
			return DoBatchBoxSweeps(*this, pint);
		if(mQueryShapeType==PINT_SHAPE_SPHERE)
			return DoBatchSphereSweeps(*this, pint);
		if(mQueryShapeType==PINT_SHAPE_CAPSULE)
			return DoBatchCapsuleSweeps(*this, pint);
		if(mQueryShapeType==PINT_SHAPE_CONVEX)
			return DoBatchConvexSweeps(*this, pint);
		return 0;
	}

	virtual	void			CommonRender(PintRender& renderer)
	{
		const Point Color(1.0f, 1.0f, 1.0f);
		renderer.DrawLine(mTriangle.mVerts[0], mTriangle.mVerts[1], Color);
		renderer.DrawLine(mTriangle.mVerts[1], mTriangle.mVerts[2], Color);
		renderer.DrawLine(mTriangle.mVerts[2], mTriangle.mVerts[0], Color);
		if(mDoubleSided)
//			renderer.DrawTriangle(mTriangle.mVerts[0], mTriangle.mVerts[1], mTriangle.mVerts[2], Color);
			renderer.DrawTriangle(mTriangle.mVerts[0], mTriangle.mVerts[2], mTriangle.mVerts[1], Color);

		if(0)	// Debug render code drawing extruded triangle for the capsule case / TEST_INITIAL_OVERLAP
		{
			const Point v0(-2.666666f, 0.0f, -1.333333f);
			const Point v1(1.333333f, 0.0f, 2.666666f);
			const Point v2(1.333333f, 0.0f, -1.333333f);

//			const Point ExtrusionDir(2.4054673f, 0.83290195f, 0.0f);

			PintCapsuleSweepData* Data = GetRegisteredCapsuleSweeps();
			const Point ExtrusionDir = (Data->mCapsule.mP0 - Data->mCapsule.mP1)*0.5f;
			const Point Center = (Data->mCapsule.mP0 + Data->mCapsule.mP1)*0.5f;
			renderer.DrawLine(Center, Center+Point(10.0f, 0.0f, 0.0f), Point(0.5f, 0.5f, 0.5f));
			renderer.DrawLine(Center, Center+Point(0.0f, 10.0f, 0.0f), Point(0.5f, 0.5f, 0.5f));
			renderer.DrawLine(Center, Center+Point(0.0f, 0.0f, 10.0f), Point(0.5f, 0.5f, 0.5f));

			const Point p0 = v0 - ExtrusionDir;
			const Point p1 = v1 - ExtrusionDir;
			const Point p2 = v2 - ExtrusionDir;

			const Point p0b = v0 + ExtrusionDir;
			const Point p1b = v1 + ExtrusionDir;
			const Point p2b = v2 + ExtrusionDir;

			renderer.DrawTriangle(v0, v1, v2, Point(0.0f, 1.0f, 0.0f));
			renderer.DrawTriangle(p0, p1, p2, Point(1.0f, 0.0f, 0.0f));
			renderer.DrawTriangle(p0b, p1b, p2b, Point(0.0f, 0.0f, 1.0f));

			renderer.DrawTriangle(p1, p1b, p2b, Point(0.0f, 1.0f, 1.0f));
			renderer.DrawTriangle(p1, p2b, p2, Point(0.0f, 1.0f, 1.0f));

			renderer.DrawTriangle(p0, p2, p2b, Point(1.0f, 1.0f, 0.0f));
			renderer.DrawTriangle(p0, p2b, p0b, Point(1.0f, 1.0f, 0.0f));

			renderer.DrawTriangle(p0b, p1b, p1, Point(1.0f, 0.0f, 1.0f));
			renderer.DrawTriangle(p0b, p1, p0, Point(1.0f, 0.0f, 1.0f));
		}

		if(0)	// Debug render code drawing extruded triangle for the capsule case / TEST_PARALLEL_SWEEP
		{
			const Point v0(-2.666666f, 0.0f, -1.333333f);
			const Point v1(1.333333f, 0.0f, 2.666666f);
			const Point v2(1.333333f, 0.0f, -1.333333f);

//			const Point ExtrusionDir(2.4054673f, 0.83290195f, 0.0f);

			PintCapsuleSweepData* Data = GetRegisteredCapsuleSweeps();
			const Point ExtrusionDir = (Data->mCapsule.mP0 - Data->mCapsule.mP1)*0.5f;
			const Point Center = (Data->mCapsule.mP0 + Data->mCapsule.mP1)*0.5f;
			renderer.DrawLine(Center, Center+Point(10.0f, 0.0f, 0.0f), Point(0.5f, 0.5f, 0.5f));
			renderer.DrawLine(Center, Center+Point(0.0f, 10.0f, 0.0f), Point(0.5f, 0.5f, 0.5f));
			renderer.DrawLine(Center, Center+Point(0.0f, 0.0f, 10.0f), Point(0.5f, 0.5f, 0.5f));

			const Point p0 = v0 - ExtrusionDir;
			const Point p1 = v1 - ExtrusionDir;
			const Point p2 = v2 - ExtrusionDir;

			const Point p0b = v0 + ExtrusionDir;
			const Point p1b = v1 + ExtrusionDir;
			const Point p2b = v2 + ExtrusionDir;

			renderer.DrawTriangle(v0, v1, v2, Point(0.0f, 1.0f, 0.0f));
			renderer.DrawTriangle(p0, p1, p2, Point(1.0f, 0.0f, 0.0f));
			renderer.DrawTriangle(p0b, p1b, p2b, Point(0.0f, 0.0f, 1.0f));

			renderer.DrawTriangle(p1, p1b, p2b, Point(0.0f, 1.0f, 1.0f));
			renderer.DrawTriangle(p1, p2b, p2, Point(0.0f, 1.0f, 1.0f));

			renderer.DrawTriangle(p0, p2, p2b, Point(1.0f, 1.0f, 0.0f));
			renderer.DrawTriangle(p0, p2b, p0b, Point(1.0f, 1.0f, 0.0f));

			renderer.DrawTriangle(p0b, p1b, p1, Point(1.0f, 0.0f, 1.0f));
			renderer.DrawTriangle(p0b, p1, p0, Point(1.0f, 0.0f, 1.0f));
		}
	}

}ConfigurableSceneSweepVsSingleTriangle;

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneBoxSweepVsStaticMeshes_Archipelago = "32*32 box-sweeps against the Archipelago mesh level.";

START_SQ_TEST(SceneBoxSweepVsStaticMeshes_Archipelago, CATEGORY_SWEEP, gDesc_SceneBoxSweepVsStaticMeshes_Archipelago)

	virtual bool	CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Archipelago.bin");

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportBoxSweeps)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		Point Offset, Extents;
		GetGlobalBounds(Offset, Extents);

		const udword NbX = 32;
		const udword NbY = 32;
		const float ScaleX = Extents.x - 1.0f;
		const float ScaleY = Extents.z - 1.0f;
		const float Altitude = 30.0f;
		const Point Dir(0.0f, -1.0f, 0.0f);
		const Point BoxExtents(1.2f, 0.5f, 0.5f);
		RegisterArrayOfBoxSweeps(*this, NbX, NbY, Altitude, ScaleX, ScaleY, Dir, BoxExtents, Offset, gSQMaxDist);
		return true;
	}

	virtual void	CommonUpdate(float dt)
	{
		TestBase::CommonUpdate(dt);
		UpdateBoxSweeps(*this, mCurrentTime);
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(SceneBoxSweepVsStaticMeshes_Archipelago)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneBoxSweepVsStaticMeshes_KP = "32*32 box-sweeps against the Konoko Payne mesh level.";

START_SQ_TEST(SceneBoxSweepVsStaticMeshes_KP, CATEGORY_SWEEP, gDesc_SceneBoxSweepVsStaticMeshes_KP)

	virtual bool	CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "kp.bin");

		mCreateDefaultEnvironment = false;

		const Point Extents(1.2f, 0.5f, 0.5f);

		UnregisterAllBoxSweeps();

		BasicRandom Rnd(42);
		for(udword i=0;i<1024;i++)
		{
			Point Dir;
			UnitRandomPt(Dir, Rnd);

			OBB Box;
			Box.mCenter		= Point(50.0f, 50.0f, 50.0f);
			Box.mExtents	= Extents;
			Box.mRot.Identity();

			RegisterBoxSweep(Box, Dir, gSQMaxDist);
		}
		return true;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportBoxSweeps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual void	CommonUpdate(float dt)
	{
		TestBase::CommonUpdate(dt);
		UpdateBoxSweeps(*this, mCurrentTime);
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(SceneBoxSweepVsStaticMeshes_KP)

///////////////////////////////////////////////////////////////////////////////

static bool CommonSetup_SceneBoxSweepVsStaticMeshes_TessBunny(TestBase& test, const Point& extents, udword nb_sweeps, const Matrix3x3& rot)
{
	LoadMeshesFromFile_(test, "Bunny.bin", null, false, 3);

	BasicRandom Rnd(42);

	Point Center, Extents;
	test.GetGlobalBounds(Center, Extents);

	for(udword i=0;i<nb_sweeps;i++)
	{
		Point Dir;
		UnitRandomPt(Dir, Rnd);

		const Point Origin = Center + Dir * 20.0f;
		const OBB Box(Origin, extents, rot);
		test.RegisterBoxSweep(Box, -Dir, 200.0f);
//		if(i>=12 && i<15)
//		if(i==13)
//		test.RegisterBoxSweep(Box, -Dir, 20.0f);	//###MEGADEBUG
	}

	test.mCreateDefaultEnvironment = false;
	return true;
}

static bool Setup_SceneBoxSweepVsStaticMeshes_TessBunny(TestBase& test, Pint& pint, const PintCaps& caps/*, const Point& extents, udword nb_sweeps, const Matrix3x3& rot*/)
{
	if(!caps.mSupportBoxSweeps)
		return false;

	return CreateMeshesFromRegisteredSurfaces(pint, caps, test);
}

static const char* gDesc_SceneBoxSweepVsStaticMeshes_TessBunny_Test1 = "64 radial box-sweeps against the tessellated bunny. Each box is a (1, 1, 1) cube. Box orientation is identity.";

START_SQ_TEST(SceneBoxSweepVsStaticMeshes_TessBunny_Test1, CATEGORY_SWEEP, gDesc_SceneBoxSweepVsStaticMeshes_TessBunny_Test1)

	virtual bool	CommonSetup()
	{
		Matrix3x3 Rot;
		Rot.Identity();
		return CommonSetup_SceneBoxSweepVsStaticMeshes_TessBunny(*this, Point(1.0f, 1.0f, 1.0f), 64, Rot);
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		return Setup_SceneBoxSweepVsStaticMeshes_TessBunny(*this, pint, caps);
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(SceneBoxSweepVsStaticMeshes_TessBunny_Test1)

static const char* gDesc_SceneBoxSweepVsStaticMeshes_TessBunny_Test1b = "4096 radial box-sweeps against the tessellated bunny. Each box is a (1, 1, 1) cube. Box orientation is identity.";

START_SQ_TEST(SceneBoxSweepVsStaticMeshes_TessBunny_Test1b, CATEGORY_SWEEP, gDesc_SceneBoxSweepVsStaticMeshes_TessBunny_Test1b)

	virtual bool	CommonSetup()
	{
		Matrix3x3 Rot;
		Rot.Identity();
		return CommonSetup_SceneBoxSweepVsStaticMeshes_TessBunny(*this, Point(1.0f, 1.0f, 1.0f), 4096, Rot);
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		return Setup_SceneBoxSweepVsStaticMeshes_TessBunny(*this, pint, caps);
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(SceneBoxSweepVsStaticMeshes_TessBunny_Test1b)

static const char* gDesc_SceneBoxSweepVsStaticMeshes_TessBunny_Test2 = "256 radial box-sweeps against the tessellated bunny. Each box is a (1, 1, 1) cube. Box orientation is non-identity.";

START_SQ_TEST(SceneBoxSweepVsStaticMeshes_TessBunny_Test2, CATEGORY_SWEEP, gDesc_SceneBoxSweepVsStaticMeshes_TessBunny_Test2)

	virtual bool	CommonSetup()
	{
		Matrix3x3 RotX;
		RotX.RotX(degToRad(45.0f));

		Matrix3x3 RotY;
		RotY.RotY(degToRad(45.0f));

		RotX *= RotY;

		return CommonSetup_SceneBoxSweepVsStaticMeshes_TessBunny(*this, Point(1.0f, 1.0f, 1.0f), 256, RotX);
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		return Setup_SceneBoxSweepVsStaticMeshes_TessBunny(*this, pint, caps);
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(SceneBoxSweepVsStaticMeshes_TessBunny_Test2)

static const char* gDesc_SceneBoxSweepVsStaticMeshes_TessBunny_Test3 = "A single radial box-sweep against the tessellated bunny. Box is a (1, 1, 1) cube. Box orientation is non-identity.";

START_SQ_TEST(SceneBoxSweepVsStaticMeshes_TessBunny_Test3, CATEGORY_SWEEP, gDesc_SceneBoxSweepVsStaticMeshes_TessBunny_Test3)

	virtual bool	CommonSetup()
	{
		Matrix3x3 RotX;
		RotX.RotX(degToRad(45.0f));

		Matrix3x3 RotY;
		RotY.RotY(degToRad(45.0f));

		RotX *= RotY;

		return CommonSetup_SceneBoxSweepVsStaticMeshes_TessBunny(*this, Point(1.0f, 1.0f, 1.0f), 1, RotX);
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		return Setup_SceneBoxSweepVsStaticMeshes_TessBunny(*this, pint, caps);
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(SceneBoxSweepVsStaticMeshes_TessBunny_Test3)

static const char* gDesc_SceneBoxSweepVsStaticMeshes_TessBunny2_Test4 = "256 radial box-sweeps against the tessellated bunny. Each box is a (0.1, 0.1, 0.1) cube. Box orientation is identity.";

START_SQ_TEST(SceneBoxSweepVsStaticMeshes_TessBunny_Test4, CATEGORY_SWEEP, gDesc_SceneBoxSweepVsStaticMeshes_TessBunny2_Test4)

	virtual bool	CommonSetup()
	{
		Matrix3x3 Rot;
		Rot.Identity();
		return CommonSetup_SceneBoxSweepVsStaticMeshes_TessBunny(*this, Point(0.1f, 0.1f, 0.1f), 256, Rot);
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		return Setup_SceneBoxSweepVsStaticMeshes_TessBunny(*this, pint, caps);
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(SceneBoxSweepVsStaticMeshes_TessBunny_Test4)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneSphereSweepVsStaticMeshes_Archipelago = "32*32 sphere-sweeps against the Archipelago mesh level.";

START_SQ_TEST(SceneSphereSweepVsStaticMeshes_Archipelago, CATEGORY_SWEEP, gDesc_SceneSphereSweepVsStaticMeshes_Archipelago)

	virtual bool	CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Archipelago.bin");

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphereSweeps)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		Point Offset, Extents;
		GetGlobalBounds(Offset, Extents);

		const udword NbX = 32;
		const udword NbY = 32;
		const float ScaleX = Extents.x - 1.0f;
		const float ScaleY = Extents.z - 1.0f;
		const float Altitude = 30.0f;
		const Point Dir(0.0f, -1.0f, 0.0f);
		const float Radius = 0.75f;
		RegisterArrayOfSphereSweeps(*this, NbX, NbY, Altitude, ScaleX, ScaleY, Dir, Radius, Offset, gSQMaxDist);
		return true;
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchSphereSweeps(*this, pint);
	}

END_TEST(SceneSphereSweepVsStaticMeshes_Archipelago)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneSphereSweepVsStaticMeshes_KP = "32*32 sphere-sweeps against the Konoko Payne mesh level.";

START_SQ_TEST(SceneSphereSweepVsStaticMeshes_KP, CATEGORY_SWEEP, gDesc_SceneSphereSweepVsStaticMeshes_KP)

	virtual bool	CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "kp.bin");

		mCreateDefaultEnvironment = false;

		const float Radius = 0.75f;

		UnregisterAllSphereSweeps();

		BasicRandom Rnd(42);
		for(udword i=0;i<1024;i++)
		{
			Point Dir;
			UnitRandomPt(Dir, Rnd);

			RegisterSphereSweep(Sphere(Point(50.0f, 50.0f, 50.0f), Radius), Dir, gSQMaxDist);
		}
		return true;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphereSweeps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchSphereSweeps(*this, pint);
	}

END_TEST(SceneSphereSweepVsStaticMeshes_KP)

///////////////////////////////////////////////////////////////////////////////

static bool CommonSetup_SceneSphereSweepVsStaticMeshes_TessBunny(TestBase& test, float radius)
{
	LoadMeshesFromFile_(test, "Bunny.bin", null, false, 3);

	BasicRandom Rnd(42);

	Point Center, Extents;
	test.GetGlobalBounds(Center, Extents);

	for(udword i=0;i<64;i++)
	{
		Point Dir;
		UnitRandomPt(Dir, Rnd);

		const Point Origin = Center + Dir * 20.0f;

		test.RegisterSphereSweep(Sphere(Origin, radius), -Dir, 20.0f);
	}

	test.mCreateDefaultEnvironment = false;
	return true;
}

static bool Setup_SceneSphereSweepVsStaticMeshes_TessBunny(TestBase& test, Pint& pint, const PintCaps& caps)
{
	if(!caps.mSupportSphereSweeps)
		return false;

	return CreateMeshesFromRegisteredSurfaces(pint, caps, test);
}

static const char* gDesc_SceneSphereSweepVsStaticMeshes_TessBunny = "64 radial sphere-sweeps against the tessellated bunny. Radius of the spheres is 1.0.";

START_SQ_TEST(SceneSphereSweepVsStaticMeshes_TessBunny, CATEGORY_SWEEP, gDesc_SceneSphereSweepVsStaticMeshes_TessBunny)

	virtual bool	CommonSetup()
	{
		return CommonSetup_SceneSphereSweepVsStaticMeshes_TessBunny(*this, 1.0f);
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		return Setup_SceneSphereSweepVsStaticMeshes_TessBunny(*this, pint, caps);
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchSphereSweeps(*this, pint);
	}

END_TEST(SceneSphereSweepVsStaticMeshes_TessBunny)

static const char* gDesc_SceneSphereSweepVsStaticMeshes_TessBunny2 = "64 radial sphere-sweeps against the tessellated bunny. Radius of the spheres is 0.1.";

START_SQ_TEST(SceneSphereSweepVsStaticMeshes_TessBunny2, CATEGORY_SWEEP, gDesc_SceneSphereSweepVsStaticMeshes_TessBunny2)

	virtual bool	CommonSetup()
	{
		return CommonSetup_SceneSphereSweepVsStaticMeshes_TessBunny(*this, 0.1f);
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		return Setup_SceneSphereSweepVsStaticMeshes_TessBunny(*this, pint, caps);
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchSphereSweeps(*this, pint);
	}

END_TEST(SceneSphereSweepVsStaticMeshes_TessBunny2)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneCapsuleSweepVsStaticMeshes_Archipelago = "32*32 capsule-sweeps against the Archipelago mesh level.";

START_SQ_TEST(SceneCapsuleSweepVsStaticMeshes_Archipelago, CATEGORY_SWEEP, gDesc_SceneCapsuleSweepVsStaticMeshes_Archipelago)

	virtual void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(71.21f, 4.04f, 83.04f), Point(0.55f, -0.46f, 0.70f));
		desc.mCamera[1] = CameraPose(Point(414.28f, 2.60f, 228.18f), Point(0.46f, -0.12f, 0.88f));
	}

	virtual bool	CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Archipelago.bin");

		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportCapsuleSweeps)
			return false;

		if(!CreateMeshesFromRegisteredSurfaces(pint, caps, *this))
			return false;

		Point Offset, Extents;
		GetGlobalBounds(Offset, Extents);

		const udword NbX = 32;
		const udword NbY = 32;
		const float ScaleX = Extents.x - 1.0f;
		const float ScaleY = Extents.z - 1.0f;
		const float Altitude = 30.0f;
		const Point Dir(0.0f, -1.0f, 0.0f);
		const float CapsuleRadius = 0.4f;
		const float HalfHeight = 1.8f;
		RegisterArrayOfCapsuleSweeps(*this, NbX, NbY, Altitude, ScaleX, ScaleY, Dir, CapsuleRadius, HalfHeight, Offset, gSQMaxDist);
		return true;
	}

	virtual void	CommonUpdate(float dt)
	{
		TestBase::CommonUpdate(dt);
		UpdateCapsuleSweeps(*this, mCurrentTime);
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchCapsuleSweeps(*this, pint);
	}

END_TEST(SceneCapsuleSweepVsStaticMeshes_Archipelago)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneCapsuleSweepVsStaticMeshes_KP = "32*32 capsule-sweeps against the Konoko Payne mesh level.";

START_SQ_TEST(SceneCapsuleSweepVsStaticMeshes_KP, CATEGORY_SWEEP, gDesc_SceneCapsuleSweepVsStaticMeshes_KP)

	virtual bool	CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "kp.bin");

		mCreateDefaultEnvironment = false;

		const float CapsuleRadius = 0.4f;
		const float HalfHeight = 1.8f;

		UnregisterAllCapsuleSweeps();

		BasicRandom Rnd(42);
		for(udword i=0;i<1024;i++)
		{
			Point Dir;
			UnitRandomPt(Dir, Rnd);

			LSS Capsule;
			Capsule.mP0 = Point(50.0f-HalfHeight, 50.0f, 50.0f);
			Capsule.mP1 = Point(50.0f+HalfHeight, 50.0f, 50.0f);
			Capsule.mRadius	= CapsuleRadius;

			RegisterCapsuleSweep(Capsule, Dir, gSQMaxDist);
		}
		return true;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportCapsuleSweeps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual void	CommonUpdate(float dt)
	{
		TestBase::CommonUpdate(dt);
		UpdateCapsuleSweeps(*this, mCurrentTime);
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchCapsuleSweeps(*this, pint);
	}

END_TEST(SceneCapsuleSweepVsStaticMeshes_KP)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SceneCapsuleSweepVsStaticMeshes_TessBunny = "64 radial capsule-sweeps against the tessellated bunny.";

START_SQ_TEST(SceneCapsuleSweepVsStaticMeshes_TessBunny, CATEGORY_SWEEP, gDesc_SceneCapsuleSweepVsStaticMeshes_TessBunny)

	virtual bool	CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "Bunny.bin", null, false, 3);

		mCreateDefaultEnvironment = false;

		UnregisterAllCapsuleSweeps();
		BasicRandom Rnd(42);

		Point Center, Extents;
		GetGlobalBounds(Center, Extents);

		for(udword i=0;i<64;i++)
		{
			Point Dir;
			UnitRandomPt(Dir, Rnd);

			const Point Origin = Center + Dir * 20.0f;

			LSS Capsule;
			Capsule.mP0		= Origin + Point(1.0f, 1.0f, 1.0f);
			Capsule.mP1		= Origin - Point(1.0f, 1.0f, 1.0f);
			Capsule.mRadius	= 0.1f;

			RegisterCapsuleSweep(Capsule, -Dir, 20.0f);
		}
/*
//		for(udword i=0;i<1024;i++)
		for(udword i=0;i<16;i++)
		{
//			UnitRandomPt(mRays[i].mDir, Rnd);
//			mRays[i].mOrig = mRays[i].mDir * 5000.0f;
//			mRays[i].mDir = -mRays[i].mDir;

			Point Dir;
			UnitRandomPt(Dir, Rnd);

			const Point Origin = Dir * 5000.0f;

			LSS Capsule;
			Capsule.mP0		= Origin + Point(0.5f, 0.0f, 0.0f);
			Capsule.mP1		= Origin - Point(0.5f, 0.0f, 0.0f);
			Capsule.mRadius	= 1.0f;

//			if(i==0)	// Very interesting one, gives very different hit results for each engine
//			if(i==1)	// Clear issue with bv4, impact is way too early
//			if(i==2)	// Gives a hit in bv4 (probably wrong), no hit for other engines... clear bug => now fixed
			RegisterCapsuleSweep(Capsule, -Dir, 5000.0f);
		}*/

		return true;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportCapsuleSweeps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchCapsuleSweeps(*this, pint);
	}

END_TEST(SceneCapsuleSweepVsStaticMeshes_TessBunny)

///////////////////////////////////////////////////////////////////////////////

static PintShapeRenderer* CommonSetup_SceneConvexSweepVsStaticMeshes_TessBunny(MyConvex& convex, TestBase& test)
{
//	LoadMeshesFromFile_(test, "Venus.bin", null, false, 0);
	LoadMeshesFromFile_(test, "Bunny.bin", null, false, 3);

	BasicRandom Rnd(42);

	Point Center, Extents;
	test.GetGlobalBounds(Center, Extents);

	udword i=2;	// Small convexes
//	udword i=4;	// 'Big' convexes
//	udword i=7;
//	udword i=13;
	convex.LoadFile(i);

	PintShapeRenderer* renderer = CreateConvexRenderer(convex.mNbVerts, convex.mVerts);

	const Point P(0.0f, 10.0f, 0.0f);
	Matrix3x3 Rot;
	Rot.Identity();

	for(udword i=0;i<64;i++)
	{
		Point Dir;
		UnitRandomPt(Dir, Rnd);

		const Point Origin = Center + Dir * 20.0f;

		// Beware: we rely on the indices returned by pint.CreateConvexObject() here,
		// even though the functions haven't been called yet.
		test.RegisterConvexSweep(0, renderer, PR(Origin, Rot), -Dir, 20.0f);
	}

	test.mCreateDefaultEnvironment = false;
	return renderer;
}

static bool Setup_SceneConvexSweepVsStaticMeshes_TessBunny(PintShapeRenderer* renderer, MyConvex& convex, TestBase& test, Pint& pint, const PintCaps& caps)
{
	if(!caps.mSupportMeshes || !caps.mSupportConvexSweeps)
		return false;

	PINT_CONVEX_DATA_CREATE Desc(convex.mNbVerts, convex.mVerts);
	Desc.mRenderer	= renderer;
	const udword h = pint.CreateConvexObject(Desc);
	ASSERT(h==0);

	return CreateMeshesFromRegisteredSurfaces(pint, caps, test);
}

static const char* gDesc_SceneConvexSweepVsStaticMeshes_TessBunny = "64 radial convex-sweeps against the tessellated bunny.";

START_SQ_TEST(SceneConvexSweepVsStaticMeshes_TessBunny, CATEGORY_SWEEP, gDesc_SceneConvexSweepVsStaticMeshes_TessBunny)

	MyConvex			mConvex;
	PintShapeRenderer*	mRenderer;

	virtual void	CommonRelease()
	{
		mConvex.Release();
		TestBase::CommonRelease();
	}

	virtual bool	CommonSetup()
	{
		TestBase::CommonSetup();
		mRenderer = CommonSetup_SceneConvexSweepVsStaticMeshes_TessBunny(mConvex, *this);
		return mRenderer!=null;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		return Setup_SceneConvexSweepVsStaticMeshes_TessBunny(mRenderer, mConvex, *this, pint, caps);
	}

	virtual void	CommonUpdate(float dt)
	{
		TestBase::CommonUpdate(dt);
		UpdateConvexSweeps(*this, mCurrentTime);
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchConvexSweeps(*this, pint);
	}

END_TEST(SceneConvexSweepVsStaticMeshes_TessBunny)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_BoxSweep_TestZone = "TestZone. Box sweep.";

START_SQ_TEST(BoxSweep_TestZone, CATEGORY_SWEEP, gDesc_BoxSweep_TestZone)

	virtual bool	CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "testzone.bin");

//		return Setup_PotPourri_Raycasts(*this, 4096, 10.0f);
		mCreateDefaultEnvironment = false;
		UnregisterAllBoxSweeps();

		if(1)
		{
			BasicRandom Rnd(42);
/*			for(udword i=0;i<1024;i++)
			{
				const float x = Rnd.randomFloat()*100.0f;
				const float z = Rnd.randomFloat()*100.0f;

				Point D(x, -100.0f, z);
				D.Normalize();

				Quat Q;
				//UnitRandomQuat(Q, Rnd);
				Q.Identity();
				RegisterBoxSweep(OBB(Point(x, 50.0f, z), Point(1.0f, 1.0f, 1.0f), Q), D, 1000.0f);
//				RegisterBoxSweep(OBB(Point(x, 50.0f, z), Point(1.0f, 1.0f, 1.0f), Q), Point(0.0f, -1.0f, 0.0f), 1000.0f);
			}*/

			Point Center, Extents;
			GetGlobalBounds(Center, Extents);
//desc.mCamera[0] = CameraPose(Point(8.36f, -4.22f, -28.18f), Point(0.14f, 0.21f, -0.97f));
			const udword Nb = 2048;
			for(udword i=0;i<Nb;i++)
			{
				const float Coeff = TWOPI*float(i)/float(Nb);
				const float x = sinf(Coeff);
				const float z = cosf(Coeff);

				Point D(x, 0.0f, z);
				D.Normalize();

				Quat Q;
//				UnitRandomQuat(Q, Rnd);
				Q.Identity();
//				RegisterBoxSweep(OBB(Point(Center.x, 2.0f, Center.z), Point(0.1f, 0.1f, 0.1f), Q), D, 1000.0f);
//				RegisterBoxSweep(OBB(Point(8.36f, -4.22f, -28.18f), Point(0.1f, 0.1f, 0.1f), Q), D, 1000.0f);
				RegisterBoxSweep(OBB(Point(8.36f, -4.22f, -28.18f), Point(0.5f, 0.5f, 0.5f), Q), D, 1000.0f);
			}

		}
		else
		{
			IndexedSurface* IS = GetFirstSurface();
			const Point* V = IS->GetVerts();
			const float Length = 0.1f;
			BasicRandom Rnd(42);
			for(udword i=0;i<IS->GetNbFaces();i++)
			{
				const IndexedTriangle* T = IS->GetFace(i);
				Point Center, Normal;
				T->Center(V, Center);
				T->Normal(V, Normal);

				Quat Q;
				UnitRandomQuat(Q, Rnd);
				Q.Identity();

				RegisterBoxSweep(OBB(Center+Normal*2.0f, Point(1.0f, 1.0f, 1.0f), Q), -Normal, 10000.0f);
	//			RegisterBoxSweep(OBB(Center+Normal*2.0f, Point(0.1f, 0.1f, 0.1f), Q), -Normal, 10.0f);
			}
		}
		return true;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportBoxSweeps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(BoxSweep_TestZone)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_BoxSweep_TestZone2 = "TestZone. Box sweep 2.";

START_SQ_TEST(BoxSweep2_TestZone, CATEGORY_SWEEP, gDesc_BoxSweep_TestZone2)

	virtual bool	CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "testzone.bin");

//		return Setup_PotPourri_Raycasts(*this, 4096, 10.0f);
		mCreateDefaultEnvironment = false;
		UnregisterAllBoxSweeps();

		if(1)
		{
			BasicRandom Rnd(42);
			Point Center, Extents;
			GetGlobalBounds(Center, Extents);
			const udword Nb=2048;
			for(udword i=0;i<Nb;i++)
			{
				const float Coeff = TWOPI*float(i)/float(Nb);
				const float x = sinf(Coeff);
				const float z = cosf(Coeff);

				Point D(x, 0.0f, z);
				D.Normalize();

				Quat Q;
//				UnitRandomQuat(Q, Rnd);
				Q.Identity();
				RegisterBoxSweep(OBB(Point(-15.08f, 5.20f, -58.01f), Point(0.5f, 0.5f, 0.5f), Q), D, 1000.0f);
			}

		}
		return true;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportBoxSweeps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(BoxSweep2_TestZone)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_SphereSweep_TestZone = "TestZone. Sphere sweep.";

START_SQ_TEST(SphereSweep_TestZone, CATEGORY_SWEEP, gDesc_SphereSweep_TestZone)

	virtual bool	CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "testzone.bin");

//		return Setup_PotPourri_Raycasts(*this, 4096, 10.0f);
		mCreateDefaultEnvironment = false;
		UnregisterAllSphereSweeps();

		if(1)
		{
			BasicRandom Rnd(42);
			Point Center, Extents;
			GetGlobalBounds(Center, Extents);
			for(udword i=0;i<1024;i++)
			{
				const float Coeff = TWOPI*float(i)/1024.0f;
				const float x = sinf(Coeff);
				const float z = cosf(Coeff);

				Point D(x, 0.0f, z);
				D.Normalize();

				RegisterSphereSweep(Sphere(Point(-15.08f, 5.20f, -58.01f), 0.5f), D, 1000.0f);
			}
		}
		return true;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportSphereSweeps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchSphereSweeps(*this, pint);
	}

END_TEST(SphereSweep_TestZone)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_CapsuleSweep_TestZone = "TestZone. Capsule sweep.";

START_SQ_TEST(CapsuleSweep_TestZone, CATEGORY_SWEEP, gDesc_CapsuleSweep_TestZone)

	virtual bool	CommonSetup()
	{
		TestBase::CommonSetup();

		LoadMeshesFromFile_(*this, "testzone.bin");

//		return Setup_PotPourri_Raycasts(*this, 4096, 10.0f);
		mCreateDefaultEnvironment = false;
		UnregisterAllCapsuleSweeps();

		if(1)
		{
			BasicRandom Rnd(42);
			Point Center, Extents;
			GetGlobalBounds(Center, Extents);
//			for(udword i=0;i<1024;i++)
			for(udword i=0;i<32;i++)
			{
//				const float Coeff = TWOPI*float(i)/1024.0f;
				const float Coeff = TWOPI*float(i)/32.0f;
				const float x = sinf(Coeff);
				const float z = cosf(Coeff);

				Point D(x, 0.0f, z);
				D.Normalize();

				RegisterCapsuleSweep(LSS(Segment(Point(-15.08f, 5.20f, -58.01f), Point(-15.08f, 7.20f, -58.01f)), 0.5f), D, 1000.0f);
			}
		}
		return true;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportCapsuleSweeps)
			return false;

		return CreateMeshesFromRegisteredSurfaces(pint, caps, *this);
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchCapsuleSweeps(*this, pint);
	}

END_TEST(CapsuleSweep_TestZone)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_ValveRaysAndSweeps = "Valve level, raycasts & sweeps.";

START_SQ_TEST(ValveRaysAndSweeps, CATEGORY_SWEEP, gDesc_ValveRaysAndSweeps)

	virtual bool	IsPrivate()	const
	{
		return true;
	}

	virtual bool	CommonSetup()
	{
		TestBase::CommonSetup();
		LoadRaysFile(*this, "rays(lotsof boxes).bin", false);
		mRepX = CreateRepXContext("c5m4_quarter2_Statics.repx", gValveScale, true);
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportRaycasts || !caps.mSupportMeshes || !caps.mSupportBoxSweeps)
			return false;
		return AddToPint(pint, mRepX);
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		udword Nb1 = DoBatchRaycasts(*this, pint);
		udword Nb2 = DoBatchBoxSweeps(*this, pint);
		return Nb1+Nb2;
	}

END_TEST(ValveRaysAndSweeps)

///////////////////////////////////////////////////////////////////////////////

static const char* gDesc_PlanetsideBoxSweeps = "512*4 box sweeps against the Planetside level.";

START_SQ_TEST(PlanetsideBoxSweeps, CATEGORY_SWEEP, gDesc_PlanetsideBoxSweeps)

	virtual bool	IsPrivate()	const
	{
		return true;
	}

	virtual void	GetSceneParams(PINT_WORLD_CREATE& desc)
	{
		TestBase::GetSceneParams(desc);
		desc.mCamera[0] = CameraPose(Point(157.59f, 21.66f, 770.35f), Point(0.66f, -0.54f, 0.52f));
		desc.mCamera[1] = CameraPose(Point(248.93f, 12.60f, 676.81f), Point(-0.04f, -0.23f, 0.97f));
		desc.mCamera[2] = CameraPose(Point(275.19f, 32.54f, 703.23f), Point(-1.00f, -0.08f, 0.05f));
		desc.mCamera[3] = CameraPose(Point(295.76f, 4.08f, 845.31f), Point(-0.32f, 0.11f, -0.94f));

		SRand(42);
		const Point Orig(241.09f, 6.29f, 730.26f);
		const Point Orig2(164.74f, 1.67f, 783.85f);
		const Point Orig3(275.19f, 32.54f, 703.23f);
		const Point Orig4(295.76f, 4.08f, 845.31f);
		Matrix3x3 Rot;
		Rot.Identity();
		for(udword j=0;j<512;j++)
		{
			const float angle = (float(j)/512.0f)*3.14159f*2.0f;
			const float s = sinf(angle);
			const float c = cosf(angle);
			Point Dir(s, 0.1f*(UnitRandomFloat()-0.5f), c);
			Dir.Normalize();
			RegisterBoxSweep(OBB(Orig, Point(0.1f, 0.1f, 0.1f), Rot), Dir, 5000.0f);
			RegisterBoxSweep(OBB(Orig2, Point(0.1f, 0.1f, 0.1f), Rot), Dir, 5000.0f);
			RegisterBoxSweep(OBB(Orig3, Point(0.1f, 0.1f, 0.1f), Rot), Dir, 5000.0f);
			RegisterBoxSweep(OBB(Orig4, Point(0.1f, 0.1f, 0.1f), Rot), Dir, 5000.0f);
		}
	}

	virtual bool	CommonSetup()
	{
		TestBase::CommonSetup();
		mRepX = CreateRepXContext("Planetside_Statics.repx", 1.0f, false);
		mCreateDefaultEnvironment = false;
		return true;
	}

	virtual bool	Setup(Pint& pint, const PintCaps& caps)
	{
		if(!caps.mSupportMeshes || !caps.mSupportBoxSweeps)
			return false;
		return AddToPint(pint, mRepX);
	}

	virtual udword	Update(Pint& pint, float dt)
	{
		return DoBatchBoxSweeps(*this, pint);
	}

END_TEST(PlanetsideBoxSweeps)

