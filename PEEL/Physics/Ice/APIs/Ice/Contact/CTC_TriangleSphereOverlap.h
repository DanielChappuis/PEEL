///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains code for triangle-sphere intersection
 *	\file		CTC_TriangleSphereOverlap.h
 *	\author		Pierre Terdiman
 *	\date		January, 13, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef CTCTRIANGLESPHEREOVERLAP_H
#define CTCTRIANGLESPHEREOVERLAP_H

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 *	Computes a triangle-sphere intersection.
	 *	\param		tri			[in] the indexed triangle
	 *	\param		verts		[in] the vertices
	 *	\param		center		[in] the sphere's center
	 *	\param		radius		[in] the sphere's radius
	 *	\return		true on overlap
	 */
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	inline_ bool TriSphere(const Point& p0, const Point& p1, const Point& p2, const Point& center, float radius)
	{
		return PointTriangleSqrDist(center, p0, p1, p2) < radius * radius;
	}

	inline_ bool TriSphere(const IndexedTriangle& triangle, const Point* verts, const Point& center, float radius)
	{
		return Ctc::TriSphere(verts[triangle.mRef[0]], verts[triangle.mRef[1]], verts[triangle.mRef[2]], center, radius);
	}

	// Version using SAT
	CONTACT_API bool SphereTriangleOverlap(const Point& p0, const Point& p1, const Point& p2, const Point& sphere_center, float sphere_radius);

	CONTACT_API bool SphereTriangleOverlap_Christer(const Triangle& tri, const Sphere& sphere);

#endif // CTCTRIANGLESPHEREOVERLAP_H

