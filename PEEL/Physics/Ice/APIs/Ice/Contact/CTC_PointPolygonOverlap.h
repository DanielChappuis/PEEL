///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains code for Point-Polygon intersection
 *	\file		CTC_PointPolygonOverlap.h
 *	\author		Pierre Terdiman
 *	\date		January, 15, 2003
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef CTCPOINTPOLYGONOVERLAP_H
#define CTCPOINTPOLYGONOVERLAP_H

	CONTACT_API	BOOL	PointInPolygon2D		(const Point* vertices2D, int nb_verts, const float x, const float y);
	CONTACT_API	BOOL	PointInConvexPolygon2D	(const Point* vertices2D, int nb_verts, const float x, const float y);

#endif // CTCPOINTPOLYGONOVERLAP_H

