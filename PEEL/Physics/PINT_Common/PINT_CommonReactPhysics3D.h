///////////////////////////////////////////////////////////////////////////////
/*
 *	PEEL - Physics Engine Evaluation Lab
 *	Copyright (C) 2012 Pierre Terdiman
 *	Homepage: http://www.codercorner.com/blog.htm
 */
///////////////////////////////////////////////////////////////////////////////

#ifndef PINT_COMMON_REACTPHYSICS3D_H
#define PINT_COMMON_REACTPHYSICS3D_H

	inline_ Point				ToPoint(const rp3d::Vector3& p)		{ return Point(p.x, p.y, p.z);			}
	inline_ Quat				ToQuat(const rp3d::Quaternion& q)	{ return Quat(q.w, q.x, q.y, q.z);	}
	inline_ rp3d::Vector3		ToRP3DVector3(const Point& p)		{ return rp3d::Vector3(p.x, p.y, p.z);				}
	inline_ rp3d::Quaternion	ToRP3DQuaternion(const Quat& q)	{ return rp3d::Quaternion(q.p.x, q.p.y, q.p.z, q.w);}
	inline_ PR				ToPR(const rp3d::Transform& pose)
	{
		const rp3d::Vector3 p = pose.getPosition();
		const rp3d::Quaternion q = pose.getOrientation();
		return PR(ToPoint(p), ToQuat(q));
	}

#endif