////////////////////////////////////////////////////////////////////////////
//	File:		ServerSiftGPU.cpp
//	Author:		Changchang Wu
//	Description :	implementation for the ServerSiftGPU class.
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

#include <iostream>
#include <vector>

using std::cout;
using std::vector;


#if defined(SERVER_SIFTGPU_ENABLED)

#include "GL/glew.h"

#ifdef _WIN32
	#include <winsock2.h>
	#include <process.h>
	#define socklen_t int
    #pragma comment(lib,  "ws2_32.lib")
#else
    #include <string.h>
    #include <stdio.h>
    #include <stdlib.h>
	#include <sys/types.h>
	#include <sys/socket.h>
	#include <unistd.h>
	#include <pthread.h>
	#include <spawn.h>
	#include <netdb.h>
	#include <netinet/in.h>
	//conversion from Win32
	typedef int SOCKET;
	#define INVALID_SOCKET -1
	#define closesocket close
#endif

#include "../SiftGPU/SiftGPU.h"
#include "ServerSiftGPU.h"






class SocketUtil
{
public:
	static int readline(SOCKET s, char *buf, int nread)
	{
		char c;
		int  num,n=0;
		for(n=1; n<nread; n++)
		{
			if((num=recv(s,&c,1,0))==1)
			{
				if(c== '\n')	break;	
				if(c==0) *buf++=' ';
				else *buf++=c;
			}else if( num==0)
			{
				if(n==1)	return 0;
				else		break;
			}else 
			{
				return -1;
			}		
		}
		*buf=0;	
		return n;
	}
	static int readint(SOCKET s, int* data, int count = 1)
	{
		int size = (sizeof(int) * count);
		data[0] = 0;
		return recv(s, (char*)data, size, 0) == size;
	}
	static int writeint(SOCKET s, int data)
	{
		int size = sizeof(int);
		return send(s, (char*) (&data), size, 0) == size;
	}
	static int writeint(SOCKET s, unsigned int data)
	{
		int size = sizeof(unsigned int);
		return send(s, (char*) (&data), size, 0) == size;
	}
	static int writedata(SOCKET s, const void* data, int count)
	{
		return send(s, (const char*)data, count, 0) == count;
	}
	static int readdata(SOCKET s, void* data, int count)
	{
		int count_read_sum = 0, count_read;
		char * p = (char*) data;
		do
		{
			count_read =  recv(s, p, count - count_read_sum, 0);
			p+= count_read;
			count_read_sum += count_read;
		}while(count_read > 0 && count_read_sum < count);
		return  count_read_sum == count;
	}
	static int init()
	{
 
#ifdef _WIN32
		WSADATA			WsaData; 
		if(WSAStartup(0x0202, &WsaData))
		{
			std::cout << "server: can't startup socket\n";
			return 0;
		}else
        {
            return 1;
        }
#else
		return 1;
#endif

	}
};


ServerSiftGPU::ServerSiftGPU(int port, char* remote_server)
{
	_port = port;
	_socketfd = INVALID_SOCKET;
    _connected = 0;
	strcpy(_server_name, remote_server? remote_server : "\0");
}

int ServerSiftGPU::InitSocket()
{
    return SocketUtil::init();
}


int ServerSiftGPU::StartServerProcess(int argc, char** argv)
{
	vector<char*> args(argc + 4);   char ports[16];

    args[0] = "server_siftgpu";	   
    args[1] = "-server";
	sprintf(ports, "%d", _port);    
    args[2] = ports;
    ///////////////////////////////////////////////
	for(int i = 0; i < argc; ++i) 	args[i + 3] = argv[i];
	args[argc + 3] = 0;
	///////////////////////////////////////////////////////
	//make a new process
#ifdef _WIN32
    int result = (int) _spawnv(_P_NOWAIT, "server_siftgpu.exe", &args[0]);
    if(result == -1) std::cerr<< "spawn returns -1 with error = " << errno << "\n";
	return  result != -1;
#else
	#ifdef __APPLE__
	#define LIBPATH_NAME "DYLD_LIBRARY_PATH"
	#else
	#define LIBPATH_NAME "LD_LIBRARY_PATH"
	#endif
	int result; 	pid_t pid; 
    char* oldpath = getenv(LIBPATH_NAME);
    if(oldpath == NULL)
    {
	    result =  posix_spawn(&pid, "server_siftgpu", NULL, NULL, &args[0], NULL);
    }else
    {
        char newpath[1024]= LIBPATH_NAME "=";
        strcat(newpath, oldpath);
        char* envp [] = {newpath, 0};
        result = posix_spawn(&pid, "server_siftgpu", NULL, NULL, &args[0], envp);
    }
	if(result) std::cerr << "failed to use poxis_spawn to create the server.\n";
	return  result == 0;
#endif
}

int ServerSiftGPU::ConnectServer(const char* server_name, int port)
{
	struct hostent* hh;

	if((hh = gethostbyname(server_name)) == NULL)return 0;
	
	////////////////////////////////////////
	struct sockaddr_in servaddr;	
	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(port);
	servaddr.sin_addr.s_addr=((struct in_addr*)(hh->h_addr))->s_addr;

	((SOCKET&)_socketfd) = socket(AF_INET, SOCK_STREAM, 0);
	if(_socketfd==INVALID_SOCKET) return 0;

	//if can not connect to server, start again
	if(connect(_socketfd, (struct sockaddr *)&servaddr, sizeof(servaddr)))
	{
		closesocket(_socketfd);
		_socketfd = INVALID_SOCKET;
		return 0;
	}else
	{
		std::cout<<"Connected to server " << server_name << "\n";
		return 1;
	}
}

void ServerSiftGPU::Disconnect()
{
	SocketUtil::writeint(_socketfd, _server_name[0]? COMMAND_DISCONNECT : COMMAND_EXIT);
	closesocket(_socketfd);
	_socketfd = INVALID_SOCKET;
	_connected = 0;
}

ServerSiftGPU::~ServerSiftGPU()
{
	if(_connected) Disconnect();
}


inline void ServerSiftGPU::ServerLoop(int port, int argc, char** argv)
{
	SOCKET sockfd, newsockfd;	
	struct	sockaddr_in	serv_addr, cli_addr;
	socklen_t addr_len = sizeof(sockaddr_in);
	//////////////////////////////////////////////////
	memset((char*)&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family	= AF_INET;
	serv_addr.sin_port	= htons(port);
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	/////////////////////////////////////////////

    if(ServerSiftGPU::InitSocket() == 0)
	{
		return;
	}
	//////////////////////////////////////////////////////////////
	sockfd=socket(AF_INET,SOCK_STREAM,0);
	if(sockfd==INVALID_SOCKET) 
	{
		std::cout << "server: can't open stream socket\n";
		return;
	}else if(bind(sockfd,(struct sockaddr*)&serv_addr, sizeof(serv_addr)))
    {
	    std::cout << "server: can't bind to port " <<  port <<"\n";
	    return;
    }else if(listen(sockfd, 1))
	{
		std::cout << "server: failed to listen\n";
		return;
	}else
	{
		std::cout << "server: listent to port "<< port << "\n";
	}
		
	newsockfd = accept(sockfd, (struct sockaddr*) &cli_addr, &addr_len);
	if(newsockfd == INVALID_SOCKET)
	{
		std::cout << "error: accept failed\n";
        closesocket(sockfd);
		return;
	}
	////////////////////////////////////////////////////////////////
	char buf[1024];
	int command, result;
	int sift_feature_count = 0;;
	vector<SiftGPU::SiftKeypoint> keys;
	vector<float> descriptors;
	vector<char> databuf;

	/////////////////////////////////////////////////////////////////
	SiftGPU siftgpu;
	SiftMatchGPU matcher;

	if(argc > 0) siftgpu.ParseParam(argc, argv);
    
	/////////////////////
	siftgpu.SetVerbose(0);
	/////////////////////////////////////////////////////////////////

    do
    {
	    while(SocketUtil::readint(newsockfd, &command) && command != COMMAND_DISCONNECT)
	    {
		    switch(command)
		    {
			case COMMAND_INITIALIZE:
				{
				    result = (siftgpu.CreateContextGL() == SiftGPU::SIFTGPU_FULL_SUPPORTED);
				    SocketUtil::writeint(newsockfd, result);
				    if(result)	break;
				}
            case COMMAND_EXIT:
                closesocket(newsockfd);
                closesocket(sockfd);
                return;
		    case COMMAND_ALLOCATE_PYRAMID:
			    {
				    int size[2];
				    SocketUtil::readint(newsockfd, size, 2);
				    if(size[0] > 0 && size[1] > 0) siftgpu.AllocatePyramid(size[0], size[1]);
				    break;
			    }
		    case COMMAND_GET_KEY_VECTOR:
			    {
				    int size = sift_feature_count * sizeof(SiftGPU::SiftKeypoint);
				    SocketUtil::writedata(newsockfd, &keys[0], size);
				    break;
			    }
		    case COMMAND_GET_DES_VECTOR:
			    {
				    int size = sift_feature_count * sizeof(float) * 128;
				    SocketUtil::writedata(newsockfd, &descriptors[0], size);
				    break;
			    }
		    case COMMAND_RUNSIFT:
			    {
				    result = siftgpu.RunSIFT();
				    if((sift_feature_count = siftgpu.GetFeatureNum()) > 0)
				    {
					    keys.resize(sift_feature_count);
					    descriptors.resize(sift_feature_count * 128);
					    siftgpu.GetFeatureVector(&keys[0], &descriptors[0]);
						std::cout << "RunSIFT: [-] [" << sift_feature_count << "]\n";
				    }
				    SocketUtil::writeint(newsockfd, result);
				    break;
			    }
		    case COMMAND_RUNSIFT_FILE:
			    {
				    SocketUtil::readline(newsockfd, buf, 1024);

				    result = siftgpu.RunSIFT(buf);
				    if((sift_feature_count = siftgpu.GetFeatureNum()) > 0)
				    {
					    keys.resize(sift_feature_count);
					    descriptors.resize(sift_feature_count * 128);
					    siftgpu.GetFeatureVector(&keys[0], &descriptors[0]);
				    }
					std::cout << "RunSIFT: "<< buf <<" " << sift_feature_count << "\n" ;
				    SocketUtil::writeint(newsockfd, result);
				    break;
			    }
		    case COMMAND_SET_KEYPOINT:
			    {
				    int keys_have_orientation;
				    SocketUtil::readint(newsockfd, &sift_feature_count);
				    SocketUtil::readint(newsockfd, &keys_have_orientation);
				    if(sift_feature_count > 0)
				    {
					    keys.resize(sift_feature_count);
					    descriptors.resize(sift_feature_count * 128);
						SocketUtil::readdata(newsockfd, &keys[0], int(keys.size() * sizeof(SiftGPU::SiftKeypoint)));
					    siftgpu.SetKeypointList(sift_feature_count, &keys[0], keys_have_orientation);
				    }
				    break;
			    }
		    case COMMAND_RUNSIFT_KEY:
			    {
				    int keys_have_orientation;
				    SocketUtil::readint(newsockfd, &sift_feature_count);
				    SocketUtil::readint(newsockfd, &keys_have_orientation);
				    if(sift_feature_count > 0)
				    {
						std::cout << "RunSIFT: "<< sift_feature_count << " KEYPOINTS\n" ;
					    int key_data_size = sift_feature_count * sizeof(SiftGPU::SiftKeypoint);
					    keys.resize(sift_feature_count);
					    descriptors.resize(sift_feature_count * 128);
					    SocketUtil::readdata(newsockfd, &keys[0], key_data_size);
					    result = siftgpu.RunSIFT(sift_feature_count, &keys[0], keys_have_orientation);
					    siftgpu.GetFeatureVector(NULL, &descriptors[0]);
				    }else
				    {
					    result = 0;
				    }
				    SocketUtil::writeint(newsockfd, result);
				    break;
			    }
		    case COMMAND_RUNSIFT_DATA:
			    {
				    int data_des[4], size = 0;	
				    SocketUtil::readint(newsockfd, data_des, 4);
					SocketUtil::readint(newsockfd, &size, 1);
					std::cout << "RunSIFT: [" << data_des[0] << "x" << data_des[1] << "]";

					databuf.resize(size);
					void* data_ptr = &databuf[0];
					SocketUtil::readdata(newsockfd, data_ptr, size);


				    result = siftgpu.RunSIFT(data_des[0], data_des[1], data_ptr, data_des[2], data_des[3]);
				    if((sift_feature_count = siftgpu.GetFeatureNum()) > 0)
				    {
					    keys.resize(sift_feature_count);
					    descriptors.resize(sift_feature_count * 128);
					    siftgpu.GetFeatureVector(&keys[0], &descriptors[0]);
				    }
					std::cout << "[" << sift_feature_count << "]\n";
				    SocketUtil::writeint(newsockfd, result);
				    break;
			    }
		    case COMMAND_SAVE_SIFT:
			    {
				    SocketUtil::readline(newsockfd, buf, 1024);
				    siftgpu.SaveSIFT(buf);
				    break;
			    }
		    case COMMAND_SET_MAX_DIMENSION:
			    {
				    int maxd;
				    if(SocketUtil::readint(newsockfd, &maxd) && maxd > 0) siftgpu.SetMaxDimension(maxd);
				    break;
			    }
		    case COMMAND_SET_TIGHTPYRAMID:
			    {
				    int tight;
				    if(SocketUtil::readint(newsockfd, &tight))  siftgpu.SetTightPyramid(tight);
				    break;
			    }
		    case COMMAND_GET_FEATURE_COUNT:
			    {
				    SocketUtil::writeint(newsockfd, sift_feature_count);
				    break;
			    }
			case COMMAND_PARSE_PARAM:
				{
				    SocketUtil::readline(newsockfd, buf, 1024);
					std::cout << "ParseParam [" << buf << "]\n";
					vector<char*> params;
					char* p = buf;
					while(*p)
					{
						while(*p == ' ' || *p == '\t')*p++ = 0;
						params.push_back(p);
						while(*p && *p != ' ' && *p != '\t') p++;
					}
					siftgpu.ParseParam(params.size(), &params[0]);
					break;
				}
			case COMMAND_MATCH_INITIALIZE:
				{
					result = matcher.CreateContextGL();
					SocketUtil::writeint(newsockfd, result);
					break;
				}
			case COMMAND_MATCH_SET_LANGUAGE:
				{
					int language;
					if(SocketUtil::readint(newsockfd, &language)) matcher.SetLanguage(language);
					break;
				}
			case COMMAND_MATCH_SET_DES_FLOAT:
				{
					int command[3] = {0, 0, 0};
					if(SocketUtil::readdata(newsockfd, command, sizeof(command)))
					{
						databuf.resize(sizeof(float) * 128 * command[1]);
						if(SocketUtil::readdata(newsockfd, &databuf[0], databuf.size()))
						{
							matcher.SetDescriptors(command[0], command[1], (float*) (&databuf[0]), command[2]);	
						}
					}
					break;
				}
			case COMMAND_MATCH_SET_DES_BYTE:
				{
					int command[3] = {0, 0, 0};
					if(SocketUtil::readdata(newsockfd, command, sizeof(command)))
					{
						databuf.resize(sizeof(unsigned char) * 128 * command[1]);
						if(SocketUtil::readdata(newsockfd, &databuf[0], databuf.size()))
						{
							matcher.SetDescriptors(command[0], command[1], (unsigned char*) (&databuf[0]), command[2]);	
						}
					}
					break;
				}
			case COMMAND_MATCH_GET_MATCH:
				{
					int command[2]; float fcommand[2];
					result = 0;
					if( SocketUtil::readdata(newsockfd, command, sizeof(command)) &&
						SocketUtil::readdata(newsockfd, fcommand, sizeof(fcommand)))
					{
						int max_match  = command[0], mbm = command[1];
						float distmax = fcommand[0], ratiomax = fcommand[1];
						databuf.resize(max_match * 2 * sizeof(int));
						result = matcher.GetSiftMatch(max_match, ( int(*)[2]) (&databuf[0]), distmax, ratiomax, mbm);

					}
					SocketUtil::writeint(newsockfd, result);
					if(result > 0) SocketUtil::writedata(newsockfd, &databuf[0], sizeof(int) * 2 * result);
					std::cout << "SiftMatch: " <<  result << "\n"; 
					break;
				}
			case COMMAND_MATCH_SET_MAXSIFT:
				{
					int max_sift;
					if(SocketUtil::readint(newsockfd, &max_sift)) matcher.SetMaxSift(max_sift);
					break;
				}
				break;
		    default:
			    std::cout << "unrecognized command: " << command << "\n";
				break;
		    }
	    }

        //client disconneted
        closesocket(newsockfd);
        //wait for the next client.
        std::cout << "wait for new client...";
	    newsockfd = accept(sockfd, (struct sockaddr*) &cli_addr, &addr_len);
	    if(newsockfd == INVALID_SOCKET)
	    {
		    std::cout << "error: accept failed";
            closesocket(sockfd);
		    return;
	    }else
        {
            std::cout << "connected\n\n";
        }
   }while(1);
}

void ServerSiftGPU::SetParamSiftGPU(int argc, char** argv)
{
	if(!_connected || argc <= 0) return;

	char buf[1025], *p = buf, cret = '\n';
	for(int i = 0; i < argc; ++i)
	{
		if(argv[i])	
		{
			strcpy(p, argv[i]);
			p += strlen(argv[i]);
		}
		*p++= ((i +1 < argc)? ' ' : '\0');
	}
	SocketUtil::writeint(_socketfd, COMMAND_PARSE_PARAM);
	SocketUtil::writedata(_socketfd, buf, (p - buf));
	SocketUtil::writedata(_socketfd, &cret, 1);

}

int ServerSiftGPU:: InitializeConnection(int argc, char** argv)
{
	char server_name[1024]; 
	if(!InitSocket()) return 0 ;
    if(_server_name[0] == 0)
    {
	    if(StartServerProcess(argc, argv) == 0) 
	    {
		    std::cout<<"Unable to start local siftgpu server\n";
            return 0 ;
	    }
        strcpy(server_name, "127.0.0.1");
    }else
    {
        strcpy(server_name, _server_name);
    }
	
    /////////////////////////////////////////////// 
	if(ConnectServer(server_name, _port) == 0)
	{
        //wait for one second
#ifdef _WIN32
	    Sleep(1000);
#else
	    sleep(1);
#endif
		if(ConnectServer(server_name, _port) == 0)
		{
			std::cout<<"Unable to connect siftgpu sever\n";
			return 0 ;
		}
	}
	return 1;
}

void ServerSiftGPU::ParseParam(int argc, char **argv)
{
    if(_connected == 1) 
	{
		SetParamSiftGPU(argc, argv); 
	}else	
	{
		_connected = InitializeConnection(argc, argv);
		if(_server_name[0] && argc > 0) SetParamSiftGPU(argc, argv);
	}
}

int ServerSiftGPU::VerifyContextGL()
{
		////////////////////////////////////////////////////
	if(!_connected) return 0;

	int result = 0;

	if(SocketUtil::writeint(_socketfd, COMMAND_INITIALIZE) && 
		SocketUtil::readint(_socketfd, &result) && result)
	{
		return SiftGPU::SIFTGPU_FULL_SUPPORTED;
	}else
	{
		std::cout<<"SifGPU failed to initialize\n";
		Disconnect();
		return 0;
	}
}

int	ServerSiftGPU::GetFeatureNum()
{
	if(!_connected) return 0;
	int result = 0;
	SocketUtil::writeint(_socketfd, COMMAND_GET_FEATURE_COUNT);
	SocketUtil::readint(_socketfd, &result);
	return result;
}

int ServerSiftGPU::AllocatePyramid(int width, int height)
{
	if(!_connected) return 0;
	int command[3] = {COMMAND_ALLOCATE_PYRAMID, width, height};
	return SocketUtil::writedata(_socketfd, command, sizeof(int) * 3);
}

////////////////////////////////////////////////////////////
void ServerSiftGPU::SaveSIFT(const char * szFileName)
{
	if(!_connected) return;

	char cret = '\n';
	SocketUtil::writeint(_socketfd, COMMAND_SAVE_SIFT);
	SocketUtil::writedata(_socketfd, szFileName, (int)strlen(szFileName));
	SocketUtil::writedata(_socketfd, &cret, 1);

}

void ServerSiftGPU::GetFeatureVector(SiftGPU::SiftKeypoint * keys, float * descriptors)
{
	if(!_connected) return;

	int num = GetFeatureNum(), result = 1;
	if(keys && num > 0)
	{
		result&= SocketUtil::writeint(_socketfd, COMMAND_GET_KEY_VECTOR);
		result &= SocketUtil::readdata(_socketfd, keys, num * sizeof(SiftGPU::SiftKeypoint));
	}
	if(descriptors && num > 0)
	{
		result&= SocketUtil::writeint(_socketfd, COMMAND_GET_DES_VECTOR);
		result&= SocketUtil::readdata(_socketfd, descriptors, num * 128 * sizeof(float));
	}
}

void ServerSiftGPU::SetKeypointList(int num, const SiftGPU::SiftKeypoint * keys, int keys_have_orientation)
{
	if(!_connected) return;

	SocketUtil::writeint(_socketfd, COMMAND_SET_KEYPOINT);
	SocketUtil::writeint(_socketfd, num);
	SocketUtil::writeint(_socketfd, keys_have_orientation);
	SocketUtil::writedata(_socketfd, keys, sizeof(SiftGPU::SiftKeypoint) * num);	

}

void ServerSiftGPU::SetTightPyramid(int tight)
{
	if(!_connected) return ;
	SocketUtil::writeint(_socketfd, COMMAND_SET_TIGHTPYRAMID);
	SocketUtil::writeint(_socketfd, tight);

}

void ServerSiftGPU::SetMaxDimension(int sz)
{
	if(!_connected) return ;
	SocketUtil::writeint(_socketfd, COMMAND_SET_MAX_DIMENSION);
	SocketUtil::writeint(_socketfd, sz);

}

int ServerSiftGPU::GetPixelSizeGL(unsigned int gl_format, unsigned int gl_type)
{
    int num_channel_byte = 0;
    int num_channels  = 0;
    switch(gl_type)
    {
    case GL_BITMAP:
    case GL_UNSIGNED_BYTE:
    case GL_BYTE:
        num_channel_byte = 1;
        break;
    case GL_UNSIGNED_SHORT:
    case GL_SHORT:
        num_channel_byte = 2;
        break;
    case GL_UNSIGNED_INT:
    case GL_INT: 
    case GL_FLOAT:
        num_channel_byte = 4;
        break;
    default:
        num_channel_byte = 0;
        break;
    }

    switch(gl_format)
    {
    case GL_RED:
    case GL_GREEN:
    case GL_BLUE:
    case GL_ALPHA:
    case GL_LUMINANCE:
        num_channels = 1;
        break;
    case GL_RGB:
    case GL_BGR_EXT:
        num_channels = 3;
        break;
    case GL_RGBA:
    case GL_BGRA_EXT:
#ifdef GL_ARGB_I3D
    case GL_ARGB_I3D:
#endif
        num_channels = 4;
        break;
    case GL_LUMINANCE_ALPHA:
#ifdef GL_422_EXT
    case GL_422_EXT:
    case GL_422_REV_EXT: 
    case GL_422_AVERAGE_EXT:
    case GL_422_REV_AVERAGE_EXT:
#endif
        num_channels = 2;
    default:
        num_channels = 0;
        break;
    }
    return num_channels * num_channel_byte;
}

int ServerSiftGPU::RunSIFT(int width, int height, const void * data, unsigned int gl_format, unsigned int gl_type)
{
	if(width <=0 || height <= 0 || data == NULL || !_connected) return 0;
    int num_bytes = GetPixelSizeGL(gl_format , gl_type) * width * height;
    if(num_bytes == 0) return 0;
	SocketUtil::writeint(_socketfd, COMMAND_RUNSIFT_DATA);
	unsigned int data_des[5] = {width, height, gl_format, gl_type, num_bytes};
	SocketUtil::writedata(_socketfd, data_des, 5 * sizeof(unsigned int));
	SocketUtil::writedata(_socketfd, data, num_bytes);
	int result = 0;	
	return SocketUtil::readint(_socketfd, &result) && result;
}

int ServerSiftGPU::RunSIFT(int num, const SiftGPU::SiftKeypoint * keys, int keys_have_orientation)
{
	if(num <= 0 || keys == NULL) return 0;
	if(!_connected) return 0;
	SocketUtil::writeint(_socketfd, COMMAND_RUNSIFT_KEY);
	SocketUtil::writeint(_socketfd, num);
	SocketUtil::writeint(_socketfd, keys_have_orientation);
	SocketUtil::writedata(_socketfd, keys, sizeof(SiftGPU::SiftKeypoint) * num);
	int result = 0;	
	return SocketUtil::readint(_socketfd, &result) && result;
}

int ServerSiftGPU::RunSIFT()
{
	if(!_connected) return 0;
    int result = 0;
    SocketUtil::writeint(_socketfd, COMMAND_RUNSIFT);
	return SocketUtil::readint(_socketfd, &result) && result;
}

int ServerSiftGPU::RunSIFT(const char *imgpath)
{
	if(!_connected) return 0;
	int result = 0;	char cret = '\n';
	SocketUtil::writeint(_socketfd, COMMAND_RUNSIFT_FILE);
	SocketUtil::writedata(_socketfd, imgpath, (int)strlen(imgpath));
	SocketUtil::writedata(_socketfd, &cret, 1);
	return SocketUtil::readint(_socketfd, &result) && result;
}


////////////////////////////////////////////////
int ServerSiftGPU::_VerifyContextGL()
{
	if(!_connected) return 0;
	int result;
	if( SocketUtil::writeint(_socketfd, COMMAND_MATCH_INITIALIZE) &&
		SocketUtil::readint(_socketfd, &result) && result)
	{
		return 1;
	}else
	{
		Disconnect();
		return 0;
	}
}

void ServerSiftGPU::SetLanguage(int gpu_language)
{
	if(!_connected) return ;
	SocketUtil::writeint(_socketfd, COMMAND_MATCH_SET_LANGUAGE);
	SocketUtil::writeint(_socketfd, gpu_language);
}

void ServerSiftGPU::SetDeviceParam(int argc, char**argv)
{
    ServerSiftGPU::ParseParam(argc, argv);
}


void ServerSiftGPU::SetMaxSift(int max_sift)
{
	if(!_connected) return;
	SocketUtil::writeint(_socketfd,	COMMAND_MATCH_SET_MAXSIFT);
	SocketUtil::writeint(_socketfd, max_sift);
}

void ServerSiftGPU::SetDescriptors(int index, int num, const float* descriptors, int id)
{
	if(!_connected) return ;
	int command[4] = {COMMAND_MATCH_SET_DES_FLOAT, index, num, id};
	SocketUtil::writedata(_socketfd, command, sizeof(command));
	SocketUtil::writedata(_socketfd, descriptors, sizeof(float) * 128 * num);
}


void ServerSiftGPU::SetDescriptors(int index, int num, const unsigned char * descriptors, int id)
{
	if(!_connected) return ;
	int command[4] = {COMMAND_MATCH_SET_DES_BYTE, index, num, id};
	SocketUtil::writedata(_socketfd, command, sizeof(command));
	SocketUtil::writedata(_socketfd, descriptors, sizeof(unsigned char) * 128 * num);
}

int  ServerSiftGPU::GetSiftMatch(int max_match,	int match_buffer[][2], 
	float distmax, float ratiomax,	int mutual_best_match)
{
	if(!_connected) return 0;
	int command[3] = {COMMAND_MATCH_GET_MATCH, max_match, mutual_best_match};
	float fcommand[2] = {distmax, ratiomax};
	SocketUtil::writedata(_socketfd, command, sizeof(command));
	SocketUtil::writedata(_socketfd, fcommand, sizeof(fcommand));
	int nm;	SocketUtil::readint(_socketfd, &nm);
	if(nm > 0) SocketUtil::readint(_socketfd, match_buffer[0], 2 * nm);
	return nm;
}

void RunServerLoop(int port, int argc, char** argv)
{
    ServerSiftGPU::ServerLoop(port, argc, argv);
}


ComboSiftGPU* CreateRemoteSiftGPU(int port, char* remote_server)
{
	return new ServerSiftGPU(port, remote_server);
}


#else

#include "../SiftGPU/LiteWindow.h"
#include "../SiftGPU/SiftGPU.h"

ComboSiftGPU* CreateRemoteSiftGPU(int port, char* remote_server)
{
    std::cout << "ServerSiftGPU need marcro SERVER_SIFTGPU_EANBLED.\n"
              << "Use local SiftGPU/SiftMatchGPU instead. \n";
	return new ComboSiftGPU;
}

void RunServerLoop(int port, int argc, char** argv)
{
    std::cout << "ServerSiftGPU need marcro SERVER_SIFTGPU_EANBLED.\n"
 }

#endif

