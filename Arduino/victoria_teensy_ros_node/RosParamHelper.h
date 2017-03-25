#ifndef VICTORIA_TEENSY_ROS_NODE_ROS_PARAM_HELPER_H_
#define VICTORIA_TEENSY_ROS_NODE_ROS_PARAM_HELPER_H_

#include <ros.h>

/**
 * \class RosParamHelper
 * \brief Class to help simplify access to ros parameters.
 */
class RosParamHelper {

public:
  RosParamHelper(ros::NodeHandle nh);
  
  /**
   * \brief Retrieves int parameter from ros and sets it
   * into the param.
   * \param name The name of the parameter.
   * \param[out] param Pointer to where to store the
   * parameter value if found in ros.
   * \return True if parameter was found in ros.
   */
  bool getParam(const char* name, int* param);

  /**
   * \brief Retrieves int parameter from ros and sets it
   * into the param. If the parameter is not found, then
   * the defaultValue is set into the param.
   * \param name The name of the parameter.
   * \param defaultValue The value to return if parameter
   * not found in ros.
   * \param[out] param Pointer to where to store the value.
   * \return True if parameter was found in ros.
   */
  bool getParam(const char* name, int* param, int defaultValue);

  /**
   * \brief Retrieves int parameter from ros and returns it.
   * If the parameter is not found, then the defaultValue
   * is returned.
   * \param name The name of the parameter.
   * \param defaultValue The value to return if parameter
   * not found in ros.
   * \return Either the value found in ros or the default value
   * if parameter not found.
   */
  int getParam(const char* name, int defaultValue);
  
  /**
   * \brief Retrieves float parameter from ros and sets it
   * into the param.
   * \param name The name of the parameter.
   * \param[out] param Pointer to where to store the
   * parameter value if found in ros.
   * \return True if parameter was found in ros.
   */
  bool getParam(const char* name, float* param);

  /**
   * \brief Retrieves float parameter from ros and sets it
   * into the param. If the parameter is not found, then
   * the default is set into the param.
   * \param name The name of the parameter.
   * \param defaultValue The value to return if parameter
   * not found in ros.
   * \param[out] param Pointer to where to store the value.
   * \return True if parameter was found in ros.
   */
  bool getParam(const char* name, float* param, float defaultValue);

  /**
   * \brief Retrieves floag parameter from ros and returns it.
   * If the parameter is not found, then the defaultValue is
   * returned.
   * \param name The name of the parameter.
   * \param defaultValue The value to return if parameter
   * not found in ros.
   * \return Either the value found in ros or the default value
   * if parameter not found.
   */
  float getParam(const char* name, float defaultValue);
  
  /**
   * \brief Retrieves char* parameter from ros and sets it
   * into the param.
   * \param name The name of the parameter.
   * \param[out] param Pointer to where to store the
   * parameter value if found in ros.
   * \return True if parameter was found in ros.
   */
  bool getParam(const char* name, char** param);

  /**
   * \brief Retrieves char* parameter from ros and sets it
   * into the param. If the parameter is not found, then
   * the default is set into the param.
   * \param name The name of the parameter.
   * \param defaultValue The value to return if parameter
   * not found in ros.
   * \param[out] param Pointer to where to store the value.
   * \return True if parameter was found in ros.
   */
  bool getParam(const char* name, char** param, char* defaultValue);

private:
  ros::NodeHandle nh_;
};

#endif
