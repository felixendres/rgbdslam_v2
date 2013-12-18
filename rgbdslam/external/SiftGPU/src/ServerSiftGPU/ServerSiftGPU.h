////////////////////////////////////////////////////////////////////////////
//	File:		ServerSiftGPu.h
//	Author:		Changchang Wu
//	Description :	interface for the ServerSiftGPU class.
//					It starts a SiftGPU server in a new process
//					or connects to an existing server.
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

#ifndef GPU_SIFT_SERVER_H
#define GPU_SIFT_SERVER_H


class ComboSiftGPU;
class LiteWindow;

/////////////////////////////////////////////////////////////////////////////
//ServerSiftGPU::ServerSiftGPU(int port, char* remote_server)
//Run SiftGPU/SiftMatchGPU computation on a remote computer/process
//if( remote_server == NULL) 
//			a local server is created in a different process and connects to it
//			multiple-GPU can be used by creating multiple instances
//			GPU selection done through ParseParam function
//otherwise, 
//			Assumes the existenc of a remote server and connects to it
//			GPU selection is done on the server-end when create the server by running
//			server_siftgpu -server port [siftgpu_param]
/////////////////////////////////////////////////////////////////////////////


class ServerSiftGPU: public ComboSiftGPU
{
	enum
	{
		COMMAND_NONE= 0,
		COMMAND_EXIT= 1,
        COMMAND_DISCONNECT,
		///////////////////////////////
		COMMAND_INITIALIZE,
		COMMAND_ALLOCATE_PYRAMID,
		COMMAND_RUNSIFT,
		COMMAND_RUNSIFT_FILE,
		COMMAND_RUNSIFT_KEY,
		COMMAND_RUNSIFT_DATA,
		COMMAND_SAVE_SIFT,
		COMMAND_SET_MAX_DIMENSION,
		COMMAND_SET_KEYPOINT,
		COMMAND_GET_FEATURE_COUNT,
		COMMAND_SET_TIGHTPYRAMID,
		COMMAND_GET_KEY_VECTOR,
		COMMAND_GET_DES_VECTOR,
		COMMAND_PARSE_PARAM,

		/////////////////////////////////
		COMMAND_MATCH_INITIALIZE,
		COMMAND_MATCH_SET_LANGUAGE,
		COMMAND_MATCH_SET_DES_FLOAT,
		COMMAND_MATCH_SET_DES_BYTE,
		COMMAND_MATCH_SET_MAXSIFT,
		COMMAND_MATCH_GET_MATCH,
		///////////////////////////////
		DEFAULT_PORT = 7777
	};
private:
#ifdef _WIN64
	unsigned __int64 _socketfd;
#else
	int			 _socketfd;
#endif
	int			 _port;
    int          _connected;
    char		 _server_name[1024];
private:
	void		SetParamSiftGPU(int argc, char** argv);
	int			InitializeConnection(int argc, char** argv);
	int			StartServerProcess(int argc, char** argv);
	int			ConnectServer(const char* server_name, int port);
	void		Disconnect();
    static int  InitSocket();
    static int	GetPixelSizeGL(unsigned int gl_format, unsigned int gl_type);
	static void ServerLoop(int port, int argc, char** argv);
public:
	//two options : multi-threading or multi-processing
	SIFTGPU_EXPORT ServerSiftGPU(int port = DEFAULT_PORT, char* remote_server = NULL);
	virtual ~ServerSiftGPU();
	////////////////////////////////////////
	virtual void ParseParam(int argc, char **argv);
    virtual int  VerifyContextGL();
	////////////////////////////////////////////////////////////////////////
	//available SiftGPU functions are the following:
	virtual int RunSIFT(const char * imgpath);
    virtual int RunSIFT();
	//note the difference with class SiftGPU for the function below, 
	//you have to provide the number of bytes of the data.
	virtual int RunSIFT(int width, int height, const void * data, unsigned int gl_format, unsigned int gl_type);
	virtual int RunSIFT(int num, const SiftGPU::SiftKeypoint * keys, int keys_have_orientation = 1);
	virtual int AllocatePyramid(int width, int height);
	/////////////////////////////////////////////////////////////////////
	virtual int	GetFeatureNum();
	virtual void SetTightPyramid(int tight = 1);
	virtual void SetMaxDimension(int sz);
	virtual void GetFeatureVector(SiftGPU::SiftKeypoint * keys, float * descriptors);
	virtual void SetKeypointList(int num, const SiftGPU::SiftKeypoint * keys, int keys_have_orientation = 1);
	/////////////////////////////////////////////////////////////////////////////
	//the following functions do not block in multi-process mode
	//for example, SaveSIFT will return before the file is written
	virtual void SaveSIFT(const char * szFileName);

	//simplified functions
    int  GetImageCount(){return 1;}
    int  CreateContextGL(){return VerifyContextGL();}
	int  IsFullSupported(){return VerifyContextGL() == SiftGPU::SIFTGPU_FULL_SUPPORTED;}
    int  RunSIFT(int index) {return RunSIFT();}

////////////////////////
public:
	virtual int  _CreateContextGL() {return _VerifyContextGL();}
	virtual int  _VerifyContextGL();
	virtual void SetLanguage(int gpu_language);
    virtual void SetDeviceParam(int argc, char**argv);
	virtual void SetMaxSift(int max_sift);
	virtual void SetDescriptors(int index, int num, const float* descriptors, int id  = -1);
	virtual void SetDescriptors(int index, int num, const unsigned char * descriptors, int id = -1);
	virtual int  GetSiftMatch(int max_match,	int match_buffer[][2], //buffeture indices
				float distmax, float ratiomax,	int mutual_best_match); //
public:
    //////////////////////////////////////////////////////
    //Some SiftGPU functions  are not supported
    void SetImageList(int nimage, const char** filelist) {}
    void SetVerbose(){}

	///Guided matching is not supported here, not hard to implement yourself
	virtual void SetFeautreLocation(int index, const float* locations, int gap) {return ;}
	virtual int  GetGuidedSiftMatch(int max_match, int match_buffer[][2], float H[3][3],float F[3][3],
		float distmax,	float ratiomax,  float hdistmax, float fdistmax, int mutual_best_match) {return 0; }

    friend void RunServerLoop(int port, int argc, char** argv);

};



#endif


