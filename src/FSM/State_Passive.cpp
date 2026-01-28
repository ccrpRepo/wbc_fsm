#include <iostream>
#include <fstream>
#include "FSM/State_Passive.h"

using json = nlohmann::json;

State_Passive::State_Passive(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::PASSIVE, "passive"){

    std::string config_path = std::string(PROJECT_ROOT_DIR) + "/config/passive.json";
    std::ifstream config_file(config_path);
    if (!config_file.is_open())
    {
        std::cerr << "[ERROR] Failed to open config file: " << config_path << std::endl;
        throw std::runtime_error("Cannot open config file");
    }

    try
    {
        json config = json::parse(config_file);
        _Kds = config["passive_kds"].get<double>();
    }
    catch (const std::exception &e)
    {
        std::cerr << "[ERROR] Failed to parse config file: " << e.what() << std::endl;
        throw;
    }
    config_file.close();
}

void State_Passive::enter(){
    for(int i=0; i<NUM_DOF; i++){
        _lowCmd->motorCmd[i].q = 0;
        _lowCmd->motorCmd[i].dq = 0;
        _lowCmd->motorCmd[i].Kp = 0;
        _lowCmd->motorCmd[i].Kd = _Kds;
        _lowCmd->motorCmd[i].tau = 0;
    }
    
}

void State_Passive::run(){
}

void State_Passive::exit(){

}

FSMStateName State_Passive::checkChange(){
    if(_lowState->userCmd == UserCommand::START){
        return FSMStateName::FIXEDSTAND;
    }
    else if(_lowState->userCmd == UserCommand::SELECT){
        throw std::runtime_error("exit..");
        return FSMStateName::PASSIVE;
    }
    else{
        return FSMStateName::PASSIVE;
    }
}