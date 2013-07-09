/* This file is part of RGBDSLAM.
 * 
 * RGBDSLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * RGBDSLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with RGBDSLAM.  If not, see <http://www.gnu.org/licenses/>.
 */


 /*
 * gicp.cpp
 *
 *  Created on: Jan 23, 2011
 *      Author: engelhar
 */

#include "gicp-fallback.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <pcl/filters/voxel_grid.h>
using namespace std;

void saveCloud(const char* filename, const pointcloud_type& pc, const int max_cnt, const bool color){

    ofstream of;
    of.open(filename);
    assert(of.is_open());

    
    int write_step = 1;
    if (max_cnt>0 && (int)pc.points.size()>max_cnt)
        write_step = floor(pc.points.size()*1.0/max_cnt);
    
    int cnt = 0;
    assert(write_step > 0);

    
    // only write every write_step.th points
    for (unsigned int i=0; i<pc.points.size(); i += write_step)
    {
        point_type p = pc.points[i];

        bool invalid = (isnan(p.x) || isnan(p.y) || isnan(p.z));
        if (invalid)
        	continue;

        
        of << p.x << "\t" << p.y << "\t" << p.z;
        /*
        if (color) {
        	
        	int color = *reinterpret_cast<const int*>(&p.rgb); 
        	int r = (0xff0000 & color) >> 16;
        	int g = (0x00ff00 & color) >> 8;
        	int b = 0x0000ff & color; 
        	of << "\t \t" << r << "\t" << g << "\t" << b << "\t" << endl;
        }	
        else
        */
        	of << endl;
       // cout << p.x << "\t" << p.y << "\t" << p.z << endl;
                       
        cnt++;
    }
    

   // ROS_INFO("gicp.cpp:  saved %i pts (of %i) to %s", cnt,(int) pc.points.size(), filename);
   // printf("gicp.cpp:  saved %i pts (of %i) to %s \n", cnt,(int) pc.points.size(), filename);

    of.close();

}


void downSample(const pointcloud_type& src, pointcloud_type& to){
    pcl::VoxelGrid<point_type> down_sampler;
    down_sampler.setLeafSize (0.01, 0.01, 0.01);
    pcl::PCLBase<point_type>::PointCloudConstPtr const_cloud_ptr = boost::make_shared<pointcloud_type> (src);
    down_sampler.setInputCloud (const_cloud_ptr);
    down_sampler.filter(to);
    ROS_INFO("gicp.cpp: Downsampling from %i to %i", (int) src.points.size(), (int) to.points.size());
}



bool gicpfallback(const pointcloud_type& from, const pointcloud_type& to, Eigen::Matrix4f& transform){

	// std::clock_t starttime_gicp = std::clock();
	
    FILE *fp;
    char f1[200];
    char f2[200];

    char line[130];
    char cmd[200];

    sprintf(f1, "pc1.txt");
    sprintf(f2, "pc2.txt");

    // default values for algo work well on this data
    sprintf(cmd, "/home/endres/Phd/rospacks/rgbdslam/external/gicp/test_gicp %s %s --d_max 0.1  --debug 0",f1,f2); 

    int N = 10000;
    
    saveCloud(f1,from,N);
    saveCloud(f2,to,N);

    // cout << "time for writing: " << ((std::clock()-starttime_gicp*1.0) / (double)CLOCKS_PER_SEC) << endl;
    // std::clock_t starttime_gicp2 = std::clock();
    
    /*
     ICP is calculated by external program. It writes some intermediate results
     and the final homography on stdout, which is parsed here.
     Not very beautiful, but works :)
     */
    fp = popen(cmd, "r");

    std::vector<string> lines;

    // collect all output
    while ( fgets( line, sizeof line, fp))
    {
        lines.push_back(line);
        // ROS_INFO("gicp.cpp: %s", line);
    }
    int retval = pclose(fp);
   // cout << "time for binary: " << ((std::clock()-starttime_gicp2*1.0) / (double)CLOCKS_PER_SEC) << endl;

   // std::clock_t starttime_gicp3 = std::clock();
        
    if(retval != 0){
        ROS_ERROR_ONCE("Non-zero return value from %s: %i. Identity transformation is returned instead of GICP result.\nThis error will be reported only once.", cmd, retval);
        transform = Eigen::Matrix<float, 4, 4>::Identity();
        return false;
    }

    int pos = 0;
    // last lines contain the transformation:
    for (unsigned int i=lines.size()-5; i<lines.size()-1; i++){
    	
        stringstream ss(lines.at(i));
        for (int j=0; j<4; j++)
        {
            ss >> line;
            transform(pos,j) = atof(line);
        }
        // cout << endl;
        pos++;
    }
    
    // read the number of iterations:
    stringstream ss(lines.at(lines.size()-1));
    ss >> line; // Converged in
    ss >> line;
    ss >> line;
        
    int iter_cnt;
    iter_cnt = atoi(line);
    
    return iter_cnt < 200; // check test_gicp for maximal allowed number of iterations
    
    // ROS_INFO_STREAM("Parsed: Converged in " << iter_cnt << "iterations");
    
    
    // ROS_DEBUG_STREAM("Matrix read from ICP process: " << transform);
    // ROS_INFO_STREAM("Paper: time for icp1 (internal): " << ((std::clock()-starttime_gicp*1.0) / (double)CLOCKS_PER_SEC));
}
