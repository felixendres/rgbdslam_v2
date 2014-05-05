////////////////////////////////////////////////////////////////////////////
//	File:		GLTestWnd.h
//	Author:		Changchang Wu
//	Description : interface for the GLTestWnd class.
//				  Win32-based SiftGPU viewer
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


#if !defined(GL_TEST_WND_H)
#define GL_TEST_WND_H

#if _WIN32 && _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define WM_MY_IDLE WM_USER+1




class BasicTestWin;
class GLTestWnd  : public BasicTestWin
{
	HGLRC	        _hglrc;
	HWND	        _hWndMain;
private:
	static 	LRESULT CALLBACK	___WndProc(HWND, UINT, WPARAM, LPARAM);
	inline	LRESULT				_WndProc(UINT, WPARAM, LPARAM);
    void    CreateWindowGL();
    static  void RegisterWindowClass();
public:
	void UpdateDisplay();
	void SetWindowTitle(char *title);
	void SetDisplaySize(int w, int h);
	void ParseCommandLine(LPSTR cmd);
	void glPaint(HDC );
	void glResize(int w, int h);
	void glCreateRC(HDC hdc);
	GLTestWnd(LPSTR cmd);
    GLTestWnd(int argc, char**argv);
	virtual ~GLTestWnd();

};

#endif // !defined(GL_TEST_WND_H)
