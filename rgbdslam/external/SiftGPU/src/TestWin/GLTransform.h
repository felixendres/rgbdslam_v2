////////////////////////////////////////////////////////////////////////////
//	File:		GLTransform.h
//	Author:		Changchang Wu
//	Description : GLTransform tookit for opengl display
//
//
//
//	Copyright (c) 2007 University of North Carolina at Chapel Hill
//	All Rights Reserved
//
//	Permission to use, copy, modify and distribute this software and its
//	documentation for educational, research and non-profit purposes, without
//	fee, and without a written agreement is hereby granted, provided that the
//	above copyright notice and the following paragraph appear in all copies.
//	
//	The University of North Carolina at Chapel Hill make no representations
//	about the suitability of this software for any purpose. It is provided
//	'as is' without express or implied warranty. 
//
//	Please send BUG REPORTS to ccwu@cs.unc.edu
//
////////////////////////////////////////////////////////////////////////////
#if !defined(GL_TRANSFORM_H)
#define GL_TRANSFORM_H

#include <math.h>
class GlTransform
{
public:
	double		cx, cy;
	double		q[4];
	double		t[3];
	double		sc, ds;
	GlTransform()
	{
		q[0]	=	1.0;
		q[1]	=	q[2]	=	q[3]	=0;
		t[0]	=	t[1]	=	t[2]	=0;
		sc		=	1.0;
		cx		=	cy		= 0;
	}
	void reset()
	{
		q[0]	=	1.0;
		q[1]	=	q[2]	=	q[3]	=0;
		t[0]	=	t[1]	=	t[2]	=0;
		sc		=	1.0;
	}
	 void operator=(GlTransform& v)
	 {
		q[0]	=	v.q[0];
		q[1]	=	v.q[1];
		q[2]	=	v.q[2];
		q[3]	=	v.q[3];
		t[0]	=	v.t[0];
		t[1]	=	v.t[1];
		t[2]	=	v.t[2];
		sc		=	v.sc;	
	 }
	 void operator *=(double scale)
	 {
		sc	*=	scale;
		t[0]*=	scale;
		t[1]*=	scale;
		t[2]*=	scale;
	 }
	 void scaleset(double scale)
	 {
		double ds = scale/sc;
		t[0]*=	ds;
		t[1]*=	ds;
		t[2]*=	ds;	
		sc  = scale;
	 }
	 void scaleup()
	 {
		double scale;
		if(sc < 6) scale = float(int(sc))+1;
		else scale = sc * 2.0;
		scaleset(scale);
	 }
	 void scaledown()
	 {
		double scale;
		if(sc >1.0 &&sc < 2.0) scale = 1.0;
		else scale = sc*0.5;
		scaleset(scale);
	 }
	 void translate(int dx, int dy,	int dz =0)
	 {
		t[0]	+=	dx;
		t[1]	+=	dy;
		t[2]	+=	dz;
	 }
	 void setcenter(double x, double y)
	 {
		cx = x;
		cy = y;
		t[0] = t[1] = t[2] = 0;
	 }

	 void transform(double es = 1.0)
	 {
	    double s = sc* es;
		glTranslated(cx*es, cy*es, 0.0);
		glTranslated(t[0] ,t[1] ,t[2]);
		glScaled(s,s,s);
		glTranslated(-cx, - cy, 0);
	 }
};

#endif

