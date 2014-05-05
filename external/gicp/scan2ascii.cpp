/*************************************************************
  Generalized-ICP Copyright (c) 2009 Aleksandr Segal.
  All rights reserved.

  Redistribution and use in source and binary forms, with 
  or without modification, are permitted provided that the 
  following conditions are met:

* Redistributions of source code must retain the above 
  copyright notice, this list of conditions and the 
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the 
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
*************************************************************/



#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

// program options
#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/tokenizer.hpp> 

#include "scan.h"
#include "transform.h"

using namespace std;
namespace po = boost::program_options;

static string filename_in;
static string filename_out;
static string filename_tfm_out;

void print_usage(const char* program_name, po::options_description const& desc) {
  cout << program_name << " scan_in [scan_out]" << endl;
  cout << desc << endl;  
}

bool parse_options(int argc, char** argv) {
  bool error = false;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "print help message")
    ("scan_in", po::value<string>(), "input scan filename")
    ("scan_out", po::value<string>(), "output scan filename");

  po::positional_options_description p;
  p.add("scan_in", 1);
  p.add("scan_out", 1);

  po::variables_map vm;
  try {    
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    po::notify(vm);
  }
  catch(const std::exception &e) {
    cout << "Error: " << e.what() << endl;
    print_usage(argv[0], desc);
    error = true;
    return error;
  }

  if(vm.count("help")) {
    print_usage(argv[0], desc);
    error = true;
    return error;
  }


  if(!vm.count("scan_in")) {
    cout << "Error: scan_in filename not specified!" << endl;
    error = true;
  }
  else {
    filename_in = vm["scan_in"].as<string>();
  }

  if(!vm.count("scan_out")) {
    filename_out = filename_in;
    
    int dot_pos = filename_out.find_last_of(".");
    if(dot_pos != -1) {
      filename_out = filename_out.substr(0, dot_pos);
    }
    filename_out += ".ascii";
  }
  else {
    filename_out = vm["scan_out"].as<string>();
  }

  filename_tfm_out = filename_out;

  int dot_pos = filename_tfm_out.find_last_of(".");
  if(dot_pos != -1) {
    filename_tfm_out = filename_tfm_out.substr(0, dot_pos);   
  }
  filename_tfm_out += ".tfm";
  
  return error;
}


int main(int argc, char** argv) {
  bool error;

  error = parse_options(argc, argv);
  
  if(error) {
    return 1;
  }
  dgc_scan_t scan;
  
  
  scan.load(filename_in.c_str());

  ofstream fout(filename_out.c_str());

  if(!fout) {
    cout << "Error: could not open output file " << filename_out << endl;
    return 1;
  }
  
  for(int i = 0; i < scan.points.size(); i++) {
    fout << scan.points[i].vec[0] << "\t";
    fout << scan.points[i].vec[1] << "\t";
    fout << scan.points[i].vec[2] << endl;;
  }

  dgc_transform_t pose_tfm;
  dgc_transform_identity(pose_tfm);
  dgc_transform_rotate_x(pose_tfm, scan.pose.roll);
  dgc_transform_rotate_y(pose_tfm, scan.pose.pitch);
  dgc_transform_rotate_z(pose_tfm, scan.pose.yaw);
  dgc_transform_translate(pose_tfm, scan.pose.x, scan.pose.y, scan.pose.z);

  dgc_transform_write(pose_tfm, filename_tfm_out.c_str());

  fout.close();
  return 0;
}

