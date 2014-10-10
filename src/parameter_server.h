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
#ifndef PARAMETER_SERVER_H_
#define PARAMETER_SERVER_H_
#include <string>
#include <ros/ros.h>
#include <boost/any.hpp>

#ifdef HEMACLOUDS
#define PCL_NO_PRECOMPILE
#endif

//this is a global definition of the points to be used
//changes to omit color would need adaptations in 
//the visualization too
//#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "point_types.h"
#ifdef RGB_IS_4TH_DIM
typedef pcl::PointXYZ point_type;
typedef pcl::PointXYZNormal point_normal_type;
#elif defined(HEMACLOUDS)
typedef hema::PointXYZRGBCamSL point_type;
#else
typedef pcl::PointXYZRGB point_type;
typedef pcl::PointXYZRGBNormal point_normal_type;
#endif
typedef pcl::PointCloud<point_type> pointcloud_type;
typedef pcl::PointCloud<point_normal_type> pointcloud_normal_type;
//#define CONCURRENT_EDGE_COMPUTATION
//Compile out DEBUG Statements. Hardly benefitial though
#define ROSCONSOLE_SEVERITY_INFO 1

/*!
 * \brief Getting values from parameter server.
 * This class is used for getting the parameters from
 * the parameter server
 */
class ParameterServer {
public:
    /*!
     * Returns the singleton instance
     */
    static ParameterServer* instance();

    /*!
     * The method sets a value in the local cache and on the Parameter Server.
     * You can use bool, int, double and std::string for T
     *
     * \param param the name of the parameter
     * \value the new parameter value
     */
    template<typename T>
    void set(const std::string param, T value) {
#ifdef RGBDSLAM_DEBUG
        if(config.count(param)==0){
          ROS_ERROR("ParameterServer: Ignoring invalid parameter: \"%s\"", param.c_str());
          return;
        }
#endif
        config[param] = value;
        setOnParameterServer(pre+param, value);
    }

    /*!
     * The method returns a value from the local cache.
     * You can use bool, int, double and std::string for T
     *
     * \param param the name of the parameter
     * \return the parameter value
     */
    template<typename T>
    T get(const std::string param) {
#ifdef RGBDSLAM_DEBUG
        if(config.count(param)==0){
          ROS_FATAL("ParameterServer object queried for invalid parameter \"%s\"", param.c_str());
          assert(config.count(param)==0);
        }
#endif
        boost::any value = config[param];
        try{
          return boost::any_cast<T>(value);
        } catch( boost::bad_any_cast bac){
          std::cerr << "Bad cast: Requested data type <" << typeid(T).name() << "> for parameter '" << param << "'";
          throw; //Programmer needs to fix this. Rethrow.
        }
    }

    /*!
     * Returns the description text for the named option
     */
    std::string getDescription(std::string param_name);

    /*!
     * Provides access to the raw config data
     */
    std::map<std::string, boost::any>& getConfigData(){
      return config;
    }

    /*!
     * Receives all values from the parameter server and store them
     * in the map 'config'.
     * Will be called in the constructor
     */
    void getValues();
private:
    void addOption(std::string name, boost::any value, std::string description);
    std::map<std::string, boost::any> config;
    std::map<std::string, std::string> descriptions;

    static ParameterServer* _instance;
    std::string pre;
    ros::NodeHandle handle;

    /*!
     * Default constructor
     * private, because of singleton
     */
    ParameterServer();

    /*!
     * Loads the default configuration
     */
    void defaultConfig();

    /*!
     * Checks, whether the parameters are ok
     */
    void checkValues();

    /*!
     * Returns a value from the parameter server
     * Will only be used by getValue()
     *
     * \param param name of the parameter
     * \param def default value (get through defaultConfig())
     *
     * \return the parameter value
     */
    template<typename T>
    T getFromParameterServer(const std::string param, T def) {
        T result;
        handle.param(param, result, def);
        return result;
    }

    template<typename T>
    void setOnParameterServer(const std::string param, T new_val) {
        handle.setParam(param, new_val);
    }
};

#endif /* PARAMETER_SERVER_H_ */
