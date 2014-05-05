////////////////////////////////////////////////////////////////////////////
//	File:		server.cpp
//	Author:		Changchang Wu
//	Description :	driver of a SiftGPU Server.
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
#include <string.h>
#include <iostream>
#include <stdio.h>
using std::cout;

#include "../SiftGPU/SiftGPU.h"

//if you included ServerSiftGPU.h here. 
//CreateRemoteSiftGPU can be replaced by new ServerSiftGPU();

int main(int argc, char** argv)
{

	if(argc >= 2 && strcmp(argv[1], "-server") == 0)
	{	
		//run siftgpu server
		std::cout << "Starting server process...\n";
		int port = 7777;
		if(argc >= 3) sscanf(argv[2], "%d", &port);
		//////////////////////////////////////
		RunServerLoop(port, argc - 3, argv + 3);
	}else if(argc >=2 && strcmp(argv[1], "-test") == 0)
	{
		//start server on same computer and connect...
        //note we are using a SiftGPU pointer..
		SiftGPU* siftgpu = CreateRemoteSiftGPU();
		siftgpu->ParseParam(argc - 2, argv + 2);
        if(siftgpu->VerifyContextGL()) 
        {
			//files on the matchines where the server runs
		    siftgpu->RunSIFT("../data/800-1.jpg");
		    siftgpu->RunSIFT("../data/800-2.jpg");
        }
        delete siftgpu; //this will terminate the server
	}else if(argc >=4 && strcmp(argv[1], "-test_remote") == 0)
	{
		////test client that connects to remote server...
        char server_name[1024];  int port = 7777;	
        strcpy(server_name, argv[2]);
        sscanf(argv[3], "%d", &port);
		SiftGPU* siftgpu = CreateRemoteSiftGPU(port, server_name);//new ServerSiftGPU(port, server_name);
		siftgpu->ParseParam(argc - 4, argv + 4);
        if(siftgpu->VerifyContextGL()) 
        {
			//files on the matchines where the server runs
		    siftgpu->RunSIFT("../data/800-1.jpg");
		    siftgpu->RunSIFT("../data/800-2.jpg");
        }
        delete siftgpu;
	}else if(argc >=2 && strcmp(argv[1], "-test2") == 0)
	{
		//test using two siftgpu servers...usage:
        //server_siftgpu -test2 [server1_params] [-server2 [server2_params]]

        ////////////////////////////////////////////
        //they need to use different ports.
        SiftGPU* siftgpu1 = CreateRemoteSiftGPU(7777);//new ServerSiftGPU(7777);
        SiftGPU* siftgpu2 = CreateRemoteSiftGPU(8888);//new ServerSiftGPU(8888);

        //split the parameters for the two servers
        int argc1 = 0, argc2 = 0;
        char** argv1 = argv + 2, **argv2 = 0;
        while(argc1 + 2 < argc && strcmp(argv[argc1 + 2], "-server2"))
        {
            argc1++;
        }
        if(argc1 + 3 < argc && strcmp(argv[argc1 + 2], "-server2") == 0)
        {
            argv2 = argv + argc1 + 3;
            argc2 = argc - argc1 - 3;
        }

        //initialize the two servers with their siftgpu parameters
        siftgpu1->ParseParam(argc1, argv1);
		siftgpu2->ParseParam(argc2, argv2);
        if(siftgpu1->VerifyContextGL() && siftgpu2->VerifyContextGL()) 
        {
		    siftgpu1->RunSIFT("../data/800-1.jpg");
            siftgpu2->RunSIFT("../data/800-1.jpg");
		    siftgpu1->RunSIFT("../data/800-2.jpg");
            siftgpu2->RunSIFT("../data/800-2.jpg");
        }
        delete siftgpu1;
        delete siftgpu2;
	}else
	{
        std::cout
        <<"try -test [siftgpu_param] to create a local server and a client\n"
        <<"try -test2 [siftgpu_param1] [-server2 [siftgpu_param2]]\n"
        <<"            to create two servers and two clients\n"
        <<"try -test_remote remote_sever_name remote_server_port\n"
        <<"            to create a client and connect to remote server\n"
        <<"use -server port [siftgpu_param] to start a server\n"
        <<"Note [siftgpu_param] allows you to select GPU from multi-GPUs\n"
        <<"\n";
	}
	return 1;
}

