
#include <snake/mtsTaskTest.h>


//TODO: make both position and velocity modes possible

int main (int argc, char *argv[]){
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);

    mtsTaskTest * test = new mtsTaskTest("test", 0.1);


    std::cout << "WOO" << std::endl;

}