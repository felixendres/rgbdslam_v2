////////////////////////////////////////////////////////////////////////////
//	File:		TestWinGlut.h
//	Author:		Changchang Wu
//	Description : interface for the TestWinGlut class.
//				  GLUT-based SiftGPU viewer
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

#if !defined(TEST_WIN_GLUT_H)
#define TEST_WIN_GLUT_H

#if _WIN32 && _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class BasicTestWin;

class TestWinGlut : public BasicTestWin
{
	enum
	{
		MAX_TEST_WIN_GLUT  = 100
	};
	static void button(int button, int state,int x, int y);
	static void display();
	static void keyboard(unsigned char key, int x, int y);
	static void idle();
	void CreateGLUT();
	static void reshape(int w, int h);

	//may also use std::vector
	static TestWinGlut * _win[MAX_TEST_WIN_GLUT];

public:
	void SetDisplaySize(int w, int h);
	void UpdateDisplay();
	void SetWindowTitle(char *title);
	static void motion(int x, int y);
	static void Run();
	TestWinGlut(int argc, char**argv);
	virtual ~TestWinGlut();


};

#endif // !defined(TEST_WIN_GLUT_H)

