////////////////////////////////////////////////////////////////////////////
//    File:        MultiThreadSIFT.cpp
//    Author:      Changchang Wu
//    Description : An example to show how to use SiftGPU in multi-threading
//                  with each thread using different GPU device.
//                  The same idea also applies to SiftMatchGPU.
//
//
//    Copyright (c) 2007 University of North Carolina at Chapel Hill
//    All Rights Reserved
//
//    Permission to use, copy, modify and distribute this software and its
//    documentation for educational, research and non-profit purposes, without
//    fee, and without a written agreement is hereby granted, provided that the
//    above copyright notice and the following paragraph appear in all copies.
//    
//    The University of North Carolina at Chapel Hill make no representations
//    about the suitability of this software for any purpose. It is provided
//    'as is' without express or implied warranty. 
//
//    Please send BUG REPORTS to ccwu@cs.unc.edu
//
////////////////////////////////////////////////////////////////////////////


#include <stdlib.h>
#include <vector>
#include <iostream>
using std::vector;
using std::iostream;
#include <time.h>


#ifdef _WIN32
    #include <windows.h>
    //define this to get dll import definition for win32
    #define SIFTGPU_DLL
    #ifdef _DEBUG 
        #pragma comment(lib, "../../lib/siftgpu_d.lib")
    #else
        #pragma comment(lib, "../../lib/siftgpu.lib")
    #endif
    #define thread_t HANDLE
#else
    #include <stdio.h>
    #include <pthread.h>
    #define thread_t pthread_t
    pthread_mutex_t global_mutex = PTHREAD_MUTEX_INITIALIZER;
#endif


#include "../SiftGPU/SiftGPU.h"

class ScopedMutex
{
#ifdef _WIN32
private:
	HANDLE hmutex;
public:
	ScopedMutex(const char* name) {
		hmutex = CreateMutex(NULL, FALSE, name);
		WaitForSingleObject(hmutex, INFINITE);
	}
	~ScopedMutex()
	{
		ReleaseMutex(hmutex);
		CloseHandle(hmutex);
	}
#else
public:
	ScopedMutex(const char* name) {
        pthread_mutex_lock(&global_mutex);
	}
	~ScopedMutex()
	{
		pthread_mutex_unlock(&global_mutex);
	}
#endif
};



class MultiThreadSIFT
{
protected:
    SiftGPU*    _sift;
    const void *  _thread_param;
    int         _device_id;
private:
    void Initialize(int device_id)
    {
        ScopedMutex mutex("siftgpu_initialize");
        printf("#%d: Initialize MultiThreadSIFT...", device_id);
        InitializeSIFT();
        printf("done\n");

        //The initialization part should be protected by a mutex in
        //single-process-multi-thread mode. For now many parameters 
        //are still global static variables. 
    }
public:
    MultiThreadSIFT(int device_id = 0, const void* thread_param = NULL)
    {
        _thread_param = thread_param;
        _device_id = device_id;
    }
    virtual ~MultiThreadSIFT()
    {
        if(_sift) delete _sift;
    }

    thread_t RunThread()
    {
#ifdef _WIN32
        return CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)RunMultiThreadSIFT, this, 0, 0);
#else
        thread_t threadid;
        pthread_create(&threadid, NULL, RunMultiThreadSIFT, this);
        return threadid;
#endif
    }
#ifdef _WIN32
    static DWORD
#else
    static void*
#endif
    RunMultiThreadSIFT(void* mts)
    {
        MultiThreadSIFT* mtsift = (MultiThreadSIFT*) mts;
        mtsift->Initialize(mtsift->_device_id); 
        mtsift->RunTask();
        return 0;
    }
public:
    //Two Functions to overload for specific task
    virtual void RunTask()
    {

    }
    //set parameters and initizlze SIFT
    virtual void InitializeSIFT()
    {
        ////////////////////////////
   	    char device[2] = {'0' + _device_id, '\0'}; //works for 0-9...use sprintf if you have more than 10
        char * argv[] = {"-fo", "-1",  "-v", "0", "-cuda", device};
        //-nogl was previously required, but now default 
        int argc = sizeof(argv)/sizeof(char*);

        /////////////////////////////////////
        _sift = new SiftGPU;
        _sift->ParseParam(argc, argv);

        //create an OpenGL context for computation, and SiftGPU will be initialized automatically 
        if(_sift->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)		exit(0);
    }
};


//Multi-threading demo. 
//Each thread will load a file, and repeat the processing 100 times.

class MultiThreadDemo: public MultiThreadSIFT
{
public:
    MultiThreadDemo(int deviceid, const char* filename):  MultiThreadSIFT(deviceid, filename)
    {
    }
    virtual void RunTask()
    {
        printf("#%d, running task\n", _device_id);
        time_t t1, t2;
        t1 = time(NULL);
        for(int i = 0; i < 100; ++i) _sift->RunSIFT();
        t2 = time(NULL); 
        printf("#%d: %dhz\n", _device_id, int(100/(t2-t1)));
    }
    virtual void InitializeSIFT()
    {
        MultiThreadSIFT::InitializeSIFT();
        const char* filename = (const char*) _thread_param;
        _sift->RunSIFT(filename);

        //////////////////////////////////////////////////////////////////////
        //WARNING: the image loader (DeviL) used by SiftGPU is not thread-safe. 
        //This demo will put the file loading part in InitializeSIFT, which
        //is protected by a mutex. In your multi-thread application, you should
        //load the image data outside SiftGPU, and specify the memory data to 
        //SiftGPU directly. 
        /////////////////////////////////////////////////////////////////////
    }
};

//Multi-process demo
//Each process will repeat the processing of a file for 100 times.
class MultiProcessDemo: public MultiThreadSIFT
{
public:
    MultiProcessDemo(int deviceid, const char* filename):  MultiThreadSIFT(deviceid, filename)
    {
    }	
    virtual void RunTask()
    {
        char* filename = (char*) _thread_param;
        time_t t1, t2;
        t1 = time(NULL);
        for(int i = 0; i < 100; ++i) _sift->RunSIFT(filename);
        t2 = time(NULL); 
        printf("Speed on device %d : %dhz\n", _device_id, int(100/(t2-t1)));
    }
    virtual void InitializeSIFT()
    {
		//Although the multi-process demo here uses CUDA, 
		//if multiple GPUs are mapped to multiple monitors/displayes
		//it is possible to use OpenGL (not CUDA)for this. 
        //Also, the mutex protection is not necessary
   	    char device[2] = {'0' + _device_id, '\0'}; //works for 0-9...use sprintf if you have more than 10
        char * argv[] = {"-fo", "-1",  "-v", "0", "-cuda", device};
        int argc = sizeof(argv)/sizeof(char*);

        /////////////////////////////////////
		//create two server with differ socket ports
        _sift = CreateRemoteSiftGPU(7777 + _device_id, NULL);
        _sift->ParseParam(argc, argv);

        //create an OpenGL context for computation, and SiftGPU will be initialized automatically 
        if(_sift->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)		exit(0);
    }
};


int main()
{
    //NOTE that SiftGPU must be compiled with CUDA for this demo
    MultiThreadDemo thread1(0, "../data/640-1.jpg");
    MultiThreadDemo thread2(1, "../data/800-1.jpg");


	//Use MultiProcessDemo for multi-process mode
    //MultiProcessDemo thread1(0, "../data/640-1.jpg");
    //MultiProcessDemo thread2(1, "../data/800-1.jpg");

    printf("Starting two threads...\n");
    thread_t t1 = thread1.RunThread();
    thread_t t2 = thread2.RunThread();

#ifdef _WIN32
    HANDLE handles[2] = {t1, t2};
    WaitForMultipleObjects(2, handles, TRUE, INFINITE); 
    ////////////////////////////////////////////////////////////////
    CloseHandle(t1); 
    CloseHandle(t2);
#else   
    pthread_join(t1, NULL);
    pthread_join(t2, NULL);
#endif
    return 1;
}

