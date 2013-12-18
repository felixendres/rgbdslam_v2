////////////////////////////////////////////////////////////////////////////
//	File:		BasicTestWin.h
//	Author:		Changchang Wu
//	Description :
//		BasicTestWin:	basic viewer of SiftGPU
//						both TestWinGlut and GLTestWndare derived from this
//		SiftDriver:		A simple driver of SiftGPU
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


#if !defined(BASIC_TEST_WIN_H)
#define BASIC_TEST_WIN_H

#if _WIN32 && _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


#include "GLTransform.h"

class SiftParam;
class SiftGPUEX;

//////////////////////////////////////////////////////////////////////////
//class TestDriver
//description:	simple SiftGPU driver
/////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////
//class BasicTestWin
//description:	basic SiftGPU viewer
//				two implementations are GLTestWnd and TestWinGlut
///////////////////////////////////////////////////////////////////////////


class BasicTestWin
{
	GlTransform _transform;
	int  _looping;
	int  _motion;
	int  _motion_x, _motion_y;
	char  _title[512];
	int   _view;
	int	  _sub_view;
	int   _win_w, _win_h;

protected:
	float	_displayScale;
	int	_imgWidth, _imgHeight;
	int _win_x, _win_y;
	int	_current;
private:
	//
	float _stat_tstart;
	int  _stat_frames;
protected:
	SiftGPUEX*			_sift;
public:
	void SetVerbose();
	void FitWindow();
	void OnIdle();
	void EndMotion();
	void StartMotion(int x, int y);
	void SetView();
	void ReShape(int w, int h);
	void MoveMouse(int x, int y);
	void KeyInput(int key);
	void Display();
	virtual void UpdateDisplay()=0;
	BasicTestWin();
	virtual ~BasicTestWin();
	void ParseSiftParam(int argc, char** argv);
	void RunSiftGPU();
	static float myclock();
protected:
	virtual void SetWindowTitle(char * title)=0;
	virtual void SetDisplaySize(int w, int h)=0;
};

#endif // !defined(BASIC_TEST_WIN_H)

