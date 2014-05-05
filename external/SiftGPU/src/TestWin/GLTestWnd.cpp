////////////////////////////////////////////////////////////////////////////
//	File:		GLTestWnd.cpp
//	Author:		Changchang Wu
//	Description : implementation of the GLTestWnd class.
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


#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <windowsx.h>
#include "GL/gl.h"
#include <stdlib.h>

/////////////////////////
#include "BasicTestWin.h"
#include "GLTestWnd.h"

#ifdef _WINDOWS
int APIENTRY WinMain(HINSTANCE hInstance,   HINSTANCE hPrevInstance,
                     LPSTR     lpCmdLine,   int       nCmdShow)
{
    //////////////////////////////////////////
	//create a window
	GLTestWnd win(lpCmdLine);
#else
int main(int argc, char** argv)
{
    GLTestWnd win(argc, argv);
#endif

    ///////////////////////////////////////    
    MSG msg;    //
	while (GetMessage(&msg, NULL, 0, 0)) 
	{
		TranslateMessage(&msg);
		DispatchMessage(&msg);	
        //on idle
		if(PeekMessage(&msg, 0, 0, 0, PM_NOREMOVE)==0)SendMessage(GetActiveWindow(), WM_MY_IDLE, 0, 0);
	}
	return (int) msg.wParam;
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


GLTestWnd::GLTestWnd(LPSTR cmd)
{
	///////////////////////
	//Parse Command Line
	if(cmd) ParseCommandLine(cmd);

	//////////////////////
	//create the window
    CreateWindowGL();
}

GLTestWnd::GLTestWnd(int argc, char**argv)
{
	BasicTestWin::ParseSiftParam(argc, argv);
    CreateWindowGL();
}

GLTestWnd::~GLTestWnd()
{

}

void GLTestWnd::CreateWindowGL()
{
	_hWndMain = NULL;
    RegisterWindowClass();
	HWND hWnd = CreateWindow("SIFT_GPU_WND", "SIFT_GPU", 
		WS_OVERLAPPEDWINDOW|WS_CLIPCHILDREN,
		CW_USEDEFAULT,  CW_USEDEFAULT, 
		600, 450, 	NULL, NULL,  NULL, 	this);
	ShowWindow(hWnd,  SW_SHOW);
	UpdateWindow(hWnd);

}
LRESULT CALLBACK GLTestWnd::___WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	static int nWnd=0;
	GLTestWnd *pWnd;
	if(message==WM_CREATE)
	{
        LPCREATESTRUCT cs = (LPCREATESTRUCT)lParam;
		pWnd = (GLTestWnd*)cs->lpCreateParams;
		pWnd->_hWndMain = hWnd;
		SetWindowLong(hWnd,0,(long)(pWnd));
		nWnd++;

	}else if(message== WM_DESTROY)
	{
		//pWnd=(GLTestWnd*)GetWindowLong(hWnd,0);
		//delete pWnd;
		SetWindowLong(hWnd,0,0);
		nWnd--;
		if(nWnd==0)	PostQuitMessage(0);
		pWnd = NULL;
	}else
	{
		pWnd=(GLTestWnd*)GetWindowLong(hWnd,0);
	}
	if(pWnd)
		return pWnd->_WndProc(message,wParam,lParam);
	else
		return DefWindowProc(hWnd, message, wParam, lParam);
	
}

void GLTestWnd::RegisterWindowClass()
{
    static int registered = 0;
    if(registered) return;
	WNDCLASSEX wcex;
	wcex.cbSize			= sizeof(WNDCLASSEX); 
	wcex.style			= CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
	wcex.lpfnWndProc	= (WNDPROC)___WndProc;
	wcex.cbClsExtra		= 0;
	wcex.cbWndExtra		= 4;
	wcex.hInstance		= 0;
	wcex.hIcon			= NULL;
	wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground	= (HBRUSH)(COLOR_WINDOW+1);
	wcex.lpszMenuName	= NULL;;
	wcex.lpszClassName	= "SIFT_GPU_WND";
	wcex.hIconSm		= NULL;
	RegisterClassEx(&wcex);
	registered = 1;
}


LRESULT GLTestWnd::_WndProc(UINT message, WPARAM wParam, LPARAM lParam)
{


	switch (message) 
	{
		case WM_CREATE:
			{
                if(_win_x !=-1)
                {
                    LPCREATESTRUCT cs = (LPCREATESTRUCT)lParam;
                    MoveWindow(_hWndMain, _win_x, _win_y, cs->cx, cs->cy, 0);
                }
				SetDisplaySize(600, 450);
				HDC hdc	=	GetDC(_hWndMain);
				glCreateRC(hdc);
				if(_hglrc == NULL) exit(0);
				ReleaseDC(_hWndMain,hdc);

			}
			break;
		case WM_SIZE:
			{
				glResize(LOWORD(lParam),HIWORD(lParam));
			}
			break;
		case WM_PAINT:
			{
				PAINTSTRUCT ps;
				HDC hdc = BeginPaint(_hWndMain, &ps);
				///
				glPaint(hdc);
				///
				EndPaint(_hWndMain, &ps);
			}
			break;
		case WM_CHAR:
			KeyInput((int) wParam);
			InvalidateRect(_hWndMain, NULL, FALSE);
			break;
		case WM_LBUTTONDOWN:
			{
				int xPos = GET_X_LPARAM(lParam); 
				int yPos = GET_Y_LPARAM(lParam); 
				StartMotion(xPos, yPos);
			}
			break;
		case WM_LBUTTONUP:
			EndMotion();
			break;
		case WM_MOUSEMOVE:
			if( wParam & MK_LBUTTON)
			{
				int xPos = GET_X_LPARAM(lParam); 
				int yPos = GET_Y_LPARAM(lParam); 
				MoveMouse(xPos, yPos);
			}
		case WM_ERASEBKGND:
			return TRUE;
		case WM_MY_IDLE:
			OnIdle();
			return TRUE;
		case WM_DESTROY:
			PostQuitMessage(0);
			break;
		default:
			return DefWindowProc(_hWndMain, message, wParam, lParam);
	}
	return 0;
}
void GLTestWnd::glCreateRC(HDC hdc)
{
    int pixelformat;
	PIXELFORMATDESCRIPTOR pfd = 
	{
        sizeof(PIXELFORMATDESCRIPTOR),1,                             
        PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL|PFD_DOUBLEBUFFER,		
        PFD_TYPE_RGBA,24,0, 0, 0, 0, 0, 0,0,0,0,0, 0, 0, 0,16,0,0,                          
        PFD_MAIN_PLANE,0,0, 0, 0                     
    };
    if ((pixelformat = ChoosePixelFormat(hdc, &pfd)) ==0)return ;
    if (SetPixelFormat(hdc, pixelformat, &pfd) == FALSE)return ;
	pixelformat =::GetPixelFormat(hdc);
	::DescribePixelFormat(hdc, pixelformat, sizeof(pfd), &pfd);

	//
    _hglrc = wglCreateContext(hdc);
	wglMakeCurrent(hdc, _hglrc);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	RunSiftGPU();

	wglMakeCurrent(NULL, NULL);
 
}

void GLTestWnd::glResize(int w, int h)
{
	ReShape(w, h);
}

void GLTestWnd::glPaint(HDC hdc)
{
	wglMakeCurrent(hdc,HGLRC(_hglrc));
	Display();
	SwapBuffers(hdc);
}

void GLTestWnd::ParseCommandLine(LPSTR cmd)
{
	int argc=0;
	char**argv = new char*[256];
	if(*cmd == 0) return;
	do
	{
		while(*cmd ==' ') cmd++;
		if(*cmd)
		{
			argv[argc++] = cmd;
		}
		while(*cmd && *cmd != ' ') cmd++;
		if(*cmd==' ')	*cmd++ = 0;

	}while(*cmd && argc <256);
	BasicTestWin::ParseSiftParam(argc, argv);
}

void GLTestWnd::SetDisplaySize(int w, int h)
{
	RECT rc;	int dw,  dh;
	GetClientRect(_hWndMain, &rc);

	dw = w - rc.right;
	dh = h - rc.bottom;
	GetWindowRect(_hWndMain, &rc);
	
	MoveWindow(_hWndMain, rc.left, rc.top, rc.right - rc.left + dw,
		rc.bottom - rc.top + dh, TRUE);

}

void GLTestWnd::SetWindowTitle(char *title)
{
	SetWindowText(_hWndMain, title);
}

void GLTestWnd::UpdateDisplay()
{
	InvalidateRect(_hWndMain, NULL,0);
}

