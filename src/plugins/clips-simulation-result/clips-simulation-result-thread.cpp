#include "clips-simulation-result-thread.h"

// #include <clipsmm.h>
#include <iostream>
#include <fstream>

using namespace fawkes;

/** Constructor. */
ClipsSimulationResultThread::ClipsSimulationResultThread()
: Thread("ClipsSimulationResultThread", Thread::OPMODE_WAITFORWAKEUP),
  CLIPSFeature("simulation-result"),
  CLIPSFeatureAspect(this)
{
}

/** Destructor. */
ClipsSimulationResultThread::~ClipsSimulationResultThread()
{
}

void
ClipsSimulationResultThread::init()
{   
}

void
ClipsSimulationResultThread::finalize()
{
    envs_.clear();
}

void
ClipsSimulationResultThread::clips_context_init(const std::string           &env_name,
                                                LockPtr<CLIPS::Environment> &clips)
{
    envs_[env_name] = clips;
    logger->log_info(name(), "rabbit!!Called to initialize environment %s", env_name.c_str());
    clips.lock();
    // clips->add_function(
    //     "simulation-result-into-csv",
    //     sigc::slot<void>(sigc::bind<0>(
    //         sigc::mem_fun(*this, &ClipsSimulationResultThread::clips_simulation_result_into_csv), env_name)));

    clips->add_function(
        "simulation-result-into-csv",
        sigc::slot<void, CLIPS::Values>(sigc::bind<0>(
            sigc::mem_fun(*this, &ClipsSimulationResultThread::clips_simulation_result_into_csv), env_name)));
    clips.unlock();

}

void
ClipsSimulationResultThread::clips_context_destroyed(const std::string &env_name)
{
    envs_.erase(env_name);
	logger->log_info(name(), "Removing environment %s", env_name.c_str());
}

// void 
// ClipsSimulationResultThread::clips_simulation_result_into_csv(const std::string &env_name) 
// {
//     // get all printout data, through function parameters?
//     // write into a csv file
//     logger->log_info(name(), "Successfully invoke self-defined function inside environment %s", env_name.c_str());
// }

void 
ClipsSimulationResultThread::clips_simulation_result_into_csv(const std::string &env_name, 
                                                             CLIPS::Values      values) 
{
    // get all printout data through function parameters
    // size_t values_length = values.size();
    // for (size_t i = 0; i < values_length; i++) {
    //     std::cout << "Parameters coming: " << values[i] << std::endl;
    // }
    logger->log_info(name(), "Successfully lalala invoke self-defined function inside environment %s", env_name.c_str());

    // write into a csv file
    std::ofstream myfileliu;
    myfileliu.open ("example_liu.csv", std::ios_base::app);
    if (!myfileliu.is_open()) {
        std::cerr << "Failed to open file: " << std::endl;
        logger->log_info(name(), "wrong-------------wrong %s", env_name.c_str());
        return;
    }
    for (const auto& value : values) {
        if (value.type() == CLIPS::TYPE_SYMBOL || value.type() == CLIPS::TYPE_STRING){
            std::string val = value.as_string();
            myfileliu << val << ","; // Writing each value followed by a comma
            logger->log_info(name(), "test-------------string or symbol %s", env_name.c_str());
        }
        else if (value.type() == CLIPS::TYPE_INTEGER){
            long long int val = value.as_integer();
            myfileliu << val << ","; // Writing each value followed by a comma
            logger->log_info(name(), "test-------------integer %s", env_name.c_str());
        }
        else if (value.type() == CLIPS::TYPE_FLOAT) {
            double val = value.as_float();
            myfileliu << val << ","; // Writing each value followed by a comma
            logger->log_info(name(), "test-------------float %s", env_name.c_str());
        }
        else {
            logger->log_info(name(), "type-------------other type %s", env_name.c_str());
        }
    }
    myfileliu << std::endl;
    logger->log_info(name(), "test-------------end %s", env_name.c_str());
    myfileliu.close();
}

void
ClipsSimulationResultThread::loop()
{
}