////////////////////////////////////////////////////////////////////////////
//	File:		BasicTestWin.cpp
//	Author:		Changchang Wu
//	Description : implementation of the BasicTestWin class.
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
#ifdef _WIN32
	#define WIN32_LEAN_AND_MEAN
	#include <windows.h>
	#define SIFTGPU_DLL
	#include <time.h>
#else
	#include <sys/time.h>
#endif

#include "stdlib.h"
#include <iostream>
using std::iostream;

#ifdef __APPLE__
	#include "OpenGL/OpenGL.h"
#else
	#include "GL/gl.h"
#endif

#include "../SiftGPU/SiftGPU.h"
#include "BasicTestWin.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

BasicTestWin::BasicTestWin()
{
	_view = 0;
	_sub_view = 0;
	_motion = 0;
	_looping = 0;
	_current = 0;


	//
	_win_w = _win_h = 0;
	_imgWidth = _imgHeight = 0;
	_displayScale = 1.0f;
	_sift = new SiftGPUEX();
}

BasicTestWin::~BasicTestWin()
{
	_motion = 0;
	_looping = 0;
}



void BasicTestWin::Display()
{
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glClear(GL_COLOR_BUFFER_BIT);
	_transform.transform(_displayScale);
	if(_sift)	_sift->DisplaySIFT();
	glFlush();
	glFinish();

}

void BasicTestWin::OnIdle()
{
	if(_looping && ! _motion)
	{
		KeyInput('r');
		UpdateDisplay();
	}

}
void BasicTestWin::KeyInput(int  key)
{
	switch(key)
	{
	case '+':
	case '=':
		_transform.scaleup();
		break;
	case '-':
		_transform.scaledown();
		break;
	case '\r':
		_view++;
		_sub_view =0;
		SetView();
		break;
	case '\b':
		_view--;
		_sub_view = 0;
		SetView();
		break;
	case ' ':
	case '.':
		_sub_view++;
		SetView();
		break;
	case ',':
		_sub_view--;
		SetView();
		break;
	case 'o':
	case 'O':
		_transform.reset();
		break;
	case 'd':
	case 'D':
		if(_sift) _sift->ToggleDisplayDebug();
		break;
	case 'r':
	case 'R':
		if(_sift)
		{
			_sift->RunSIFT(++_current);
			_stat_frames++;
			FitWindow();
		}
		break;
	case 'c':
	case 'C':
		if(_sift) _sift->RandomizeColor();
		break;
	case 'q':
	case 'Q':
		if(_sift) _sift->SetVerbose(-1);
		break;
	case 'v':
		if(_sift) _sift->SetVerbose(4);
		break;
	case 'x':
	case 'X':
	case 27:
		exit(0);
		break;
	case 'l':
	case 'L':
		_looping = ! _looping;
		if(_looping)
		{
			_stat_tstart = myclock();
			_stat_frames = 0;
		}else
		{
			float t   = (myclock() - _stat_tstart);
			float fps = _stat_frames/t; 
			std::cout<<"************************************\n"
				     <<fps << " Hz : " << _stat_frames << " frames in " << t << " sec \n"
				     <<"************************************\n";
		}
		break;
	}
}



void BasicTestWin::MoveMouse(int x, int y)
{
	if(_motion==0)return;
	_transform.translate(x-_motion_x, y-_motion_y);
	_motion_x = x;
	_motion_y = y;
	UpdateDisplay();
}

void BasicTestWin::ReShape(int w, int h)
{
	glViewport(0, 0, w, h);            
    glMatrixMode(GL_PROJECTION);    
    glLoadIdentity();               
	glOrtho(0, w, h, 0,0,1);
    glMatrixMode(GL_MODELVIEW);     
    glLoadIdentity();  

	_win_w = w;
	_win_h = h;
}

void BasicTestWin::RunSiftGPU()
{
	if(_sift->RunSIFT())
	{
		_sift->SetVerbose(2);
		FitWindow();
	}else
	{
		exit(0);
	}
}

void BasicTestWin::ParseSiftParam(int argc, char** argv)
{
	_sift->ParseParam(argc, argv);
	_sift->SetVerbose(5);
	_win_x = _win_y = -1;
	_sift->GetInitWindowPotition(_win_x, _win_y);
}




void BasicTestWin::FitWindow()
{
	int w, h , dw, dh;
	_sift->GetImageDimension(w, h);
	

	if(w <=0 || h <=0 ) return;


	if( w == _imgWidth || h == _imgHeight)
	{
		ReShape(_win_w, _win_h);
		return;
	}
	
	
	_transform.setcenter(w*0.5, h*0.5);

	///

	dw =_imgWidth = w;
	dh =_imgHeight = h;

	_displayScale = 1.0;

	while(dw>1024 || dh >1024)
	{
		dw>>=1;
		dh>>=1;
		_displayScale *= 0.5;
	}

	while(dw < 512 && dh < 512)
	{
		dw <<= 1;
		dh <<= 1;
		_displayScale*= 2.0;
	}

	if ( dw > _win_w || dh > _win_h)
	{
		_win_w = dw;
		_win_h = dh;
		SetDisplaySize(dw, dh);
	}else
	{
		ReShape(_win_w, _win_h);
	}
}




void BasicTestWin::SetView()
{
	if(_sift)
	{
		_sift->SetView(_view, _sub_view, _title);
		SetWindowTitle(_title);
	}
}

void BasicTestWin::StartMotion(int x, int y)
{
	_motion = 1;
	_motion_x = x;
	_motion_y = y;
}

void BasicTestWin::EndMotion()
{
	_motion = 0;
}


void BasicTestWin::SetVerbose()
{
	_sift->SetVerbose();
}

float BasicTestWin::myclock()
{
#if defined(_WIN32)
	return clock() * 1.0 / CLOCKS_PER_SEC;
#else
	static int    started = 0;
	static struct timeval tstart;
	if(started == 0) 
	{
		gettimeofday(&tstart, NULL);
		started = 1;
		return 0;
	}else
	{	
		struct timeval now;
		gettimeofday(&now, NULL) ;
		return (now.tv_usec - tstart.tv_usec + (now.tv_sec - tstart.tv_sec) * 1000000)/1000000.f;
	}
#endif
}
