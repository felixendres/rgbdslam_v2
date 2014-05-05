////////////////////////////////////////////////////////////////////////////
//	File:		Speed.cpp
//	Author:		Changchang Wu
//	Description : Evaluate the speed of SiftGPU 
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


#include <stdlib.h>
#include <vector>
#include <iostream>
using std::cout;

#ifdef _WIN32
	//dll import definition for win32
	#define SIFTGPU_DLL
#endif

#if defined(_WIN32)
//for windows, the default timing uses timeGetTime, you can define TIMING_BY_CLOCK to use clock()
	#if defined(TIMING_BY_CLOCK)
		#include <time.h>
	#else
		#define WIN32_LEAN_AND_MEAN
		#include <windows.h>
		#include <mmsystem.h>
		#pragma comment(lib, "winmm.lib")
	#endif
#else
	#include <sys/time.h>
#endif


#include "../SiftGPU/SiftGPU.h"

//define the marcro bellow to reload image file everytime
//#define INCLUDE_DISK_READING_TIME

//define the macro below to test speed of multi-process mode
//#define TEST_MULTI_PROCESS_SIFTGPU



#define SIFTGPU_REPEAT 30

// you should specify the input image and the sift parameter to speed.exe
// for example: speed.exe -i 600.pgm  -fo -1
// to test CUDA, you first need to recompile SiftGPU with CUDA capability

int GetMilliSecond(); 

int main(int argc, char * argv[])
{
#ifndef TEST_MULTI_PROCESS_SIFTGPU
	SiftGPU sift;
#else
	ComboSiftGPU* combo = CreateRemoteSiftGPU();
	SiftGPU& sift = (*combo);
#endif
	int num;
	float timing[10], time_all =0, time_start;

	//Parse parameters
	sift.ParseParam(argc - 1, argv + 1);
	sift.SetVerbose(0); 

	std::cout<<"Initialize and warm up...\n";
	//create an OpenGL context for computation
	if(sift.CreateContextGL() ==0) return 0;
	
	if(sift.RunSIFT()==0)	return 0;

	//image is loaded for only once for this experiment
#ifndef TEST_MULTI_PROCESS_SIFTGPU
	std::cout<<"Loading image: " << sift._timing[0]*1000 << "ms, "
	         <<"Tex initialization: "<<sift._timing[1]*1000 << "ms\n\n"
	         <<"Start 2x"<<SIFTGPU_REPEAT<<" repetitions...\n";
#ifdef INCLUDE_DISK_READING_TIME
    char imagepath[MAX_PATH];
    strcpy(imagepath, sift.GetCurrentImagePath()); 
#endif
#endif

	//run one more time to get all texture allocated

	sift.RunSIFT();
	num = sift.GetFeatureNum();


	//use no message output mode to get maximum speed.
	time_start = (float) GetMilliSecond();
	for(int i = 0; i < SIFTGPU_REPEAT; i++)
	{
#ifdef INCLUDE_DISK_READING_TIME
        sift.RunSIFT(imagepath);
#else
		sift.RunSIFT();
#endif
		if(sift.GetFeatureNum()==num) std::cout <<"+";
		else std::cout << "e";
	}
	time_all = ((float)GetMilliSecond() - time_start)/1000/SIFTGPU_REPEAT;
	std::cout<<"\n"; 

	//change the timing setting, so that we can get more accurate timing for each steps
	//in this mode, the overal speed will be decreased.
	sift.SetVerbose(-2); //little trick to disable all output but keep the timing
	std::fill(timing, timing + 10, 0.0f);
	for(int k = 0; k < SIFTGPU_REPEAT; k++)
	{
#ifdef INCLUDE_DISK_READING_TIME
        sift.RunSIFT(imagepath);
#else
		sift.RunSIFT();
#endif
		for(int j = 0; j < 10; j++)		timing[j] += sift._timing[j];
		if(sift.GetFeatureNum()==num) std::cout <<"#";
		else std::cout << "e";
	}
	for(int j = 0; j < 10; j++) 	timing[j] /= SIFTGPU_REPEAT;

	std::cout
		<<"\n\n****************************************\n"
		<<"[Feature Count]:\t" << num << "\n"
		<<"[Average Time]:\t\t" << time_all * 1000.0f << "ms\n"
		<<"[Average Speed]:\t" << 1.0 / time_all << "hz\n"
#ifndef TEST_MULTI_PROCESS_SIFTGPU
#ifdef INCLUDE_DISK_READING_TIME
		<<"[Load Image File]:\t" << timing[0] * 1000.0f << "ms\n"
#endif
		<<"[Build Pyramid]:\t" << timing[2] * 1000.0f << "ms\n"
		<<"[Detection]:\t\t" << timing[3] * 1000.0f << "ms\n"
		<<"[Feature List]:\t\t" << timing[4] * 1000.0f << "ms\n"
		<<"[Orientation]:\t\t" << timing[5] * 1000.0f << "ms\n"
		<<"[MO Feature List]:\t" << timing[6] * 1000.0f << "ms\n"
		<<"[Download Keys]:\t" << timing[7] * 1000.0f << "ms\n"
		<<"[Descriptor]:\t\t" << timing[8] * 1000.0f << "ms\n"
#endif
		<<"****************************************\n";

#ifdef TEST_MULTI_PROCESS_SIFTGPU
	delete combo;
#endif

	return 0;
}



int GetMilliSecond()
{
#if defined(_WIN32)
	#if defined(TIMING_BY_CLOCK)
		return clock() * 1000 / CLOCKS_PER_SEC;
	#else
		static int  started = 0;
		static int	tstart;
		//the resolution of teimGetTime will be changed to 1ms inside SiftGPU
		if(started == 0)
		{
			tstart = timeGetTime();
			started = 1;
			return 0;
		}else
		{
			return timeGetTime() - tstart;
		}
	#endif
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
		return (now.tv_usec - tstart.tv_usec + (now.tv_sec - tstart.tv_sec) * 1000000)/1000;
	}
#endif
}
