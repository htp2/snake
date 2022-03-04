#ifndef mtsTaskTest_H
#define mtsTaskTest_H

#include <ros/ros.h>
#include <cisst_ros_bridge/mtsROSBridge.h>
#include <fbgInterrogator/interrogator.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsGenericObjectProxy.h>


class mtsTaskTest{
    
    // CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);
    
    public:
    mtsTaskTest(const std::string& name, double period){};
    ~mtsTaskTest(void){};
    void Startup(void){};
    void Run(void){};
    void Cleanup(void){};




};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTaskTest);

#endif // mtsTaskTest_H
