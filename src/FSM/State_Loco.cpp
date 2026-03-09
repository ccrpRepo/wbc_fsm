
#include <iostream>
#include "FSM/State_Loco.h"

#include <fstream>
#include <algorithm>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

State_Loco::State_Loco(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::LOCO, "rl"){
        std::string config_path = std::string(PROJECT_ROOT_DIR) + "/config/loco.json";
        std::ifstream config_file(config_path);
        if (!config_file.is_open()) {
            std::cerr << "[ERROR] Failed to open config file: " << config_path << std::endl;
            throw std::runtime_error("Cannot open config file");
        }
        
        try {
            json config = json::parse(config_file);
            std::string base_path = std::string(PROJECT_ROOT_DIR) + "/";
            _model_path = base_path + config["model_path"].get<std::string>();
            _anchor_terminate_thresh = config["safe_projgravity_threshold"].get<float>();
            
            this->_vxLim << config["vx_limit_min"].get<float>(), config["vx_limit_max"].get<float>();
            this->_vyLim << config["vy_limit_min"].get<float>(), config["vy_limit_max"].get<float>();
            this->_wyawLim << config["wyaw_limit_min"].get<float>(), config["wyaw_limit_max"].get<float>();
            this->_cmdSmoothes = config["cmd_smoothes"].get<float>();
            std::cout << "[Config Loco] Model path: " << _model_path << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Failed to parse config file: " << e.what() << std::endl;
            throw;
        }
        config_file.close();

        this->_dYawCmdPast = 0.0;
        this->_vCmdBodyPast.setZero();

        _loadPolicy(); 
}

void State_Loco::_init_buffers()
{
    this->_dYawCmdPast=0.0;
    this->_vCmdBodyPast.setZero();
}

void State_Loco::_loadPolicy()  
{
    _session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    _session = std::make_unique<Ort::Session>(_env, _model_path.c_str(), _session_options);

    Ort::TypeInfo input_type = _session->GetInputTypeInfo(0);
    auto input_shapes = input_type.GetTensorTypeAndShapeInfo().GetShape();
    Ort::TypeInfo output_type = _session->GetOutputTypeInfo(0);
    auto output_shapes = output_type.GetTensorTypeAndShapeInfo().GetShape();

    _obs_size_ = input_shapes[1];  
    _hidden_size_ = _session->GetInputTypeInfo(1).GetTensorTypeAndShapeInfo().GetShape()[2];  // [1, 1, 256] 中的256
    _action_size_ = output_shapes[1];
    _h_state_ = std::vector<float>(_hidden_size_, 0.0f);
    _c_state_ = std::vector<float>(_hidden_size_, 0.0f);
    _action = std::vector<float>(_action_size_, 0.0f);

}


void State_Loco::_observations_compute()
{
    std::vector<float> base_quat = std::vector<float>(4, 0.0f); 
    base_quat = {
        _lowState->imu.quaternion[0],  // w
        _lowState->imu.quaternion[1],  // x
        _lowState->imu.quaternion[2],  // y
        _lowState->imu.quaternion[3]}; // z
    std::vector<float> projected_gravity(3);  
    projected_gravity = QuatRotateInverse(base_quat, this->_gravity_vec);
    auto obs_projected_gravity = projected_gravity;
    float projected_gravity_error = std::abs(projected_gravity[2] - (-1.0f));
    if (projected_gravity_error > _anchor_terminate_thresh)
    {
        _terminate_flag = true;
        std::cout << "[Warning] Large projected gravity error: " << projected_gravity_error << std::endl;
    }
    
    _userValue = _lowState->userValue; 
    _getUserCmd();  
    auto obs_commands = std::vector<float>({static_cast<float>(_vCmdBody(0)), static_cast<float>(_vCmdBody(1)), static_cast<float>(_dYawCmd)});

    std::vector<float> dof_pos_vec;
    dof_pos_vec.reserve(NUM_DOF);
    for (int i = 0; i < NUM_DOF; ++i) {
        dof_pos_vec.push_back(_lowState->motorState[dof_mapping[i]].q - this->_default_dof_pos[dof_mapping[i]]);
    }
    
    std::vector<float> dof_vel_vec;
    dof_vel_vec.reserve(NUM_DOF);
    for (int i = 0; i < NUM_DOF; ++i) {
        dof_vel_vec.push_back(_lowState->motorState[dof_mapping[i]].dq);
    }

    auto body_ang_vel = std::vector<float>({
        static_cast<float>(_lowState->imu.gyroscope[0]),
        static_cast<float>(_lowState->imu.gyroscope[1]),
        static_cast<float>(_lowState->imu.gyroscope[2])
    });

    this->_observation = std::vector<float>();
    
    body_ang_vel[0] = body_ang_vel[0] * scale_lin_vel; 
    body_ang_vel[1] = body_ang_vel[1] * scale_lin_vel;  
    body_ang_vel[2] = body_ang_vel[2] * scale_ang_vel;  
    obs_commands[0] = obs_commands[0] * scale_lin_vel;  
    obs_commands[1] = obs_commands[1] * scale_lin_vel;  
    obs_commands[2] = obs_commands[2] * scale_ang_vel;  
    for(int i=0; i<NUM_DOF; i++)
    {
        dof_pos_vec[i] = dof_pos_vec[i] * scale_dof_pos;  
        dof_vel_vec[i] = dof_vel_vec[i] * scale_dof_vel; 
    }
    this->_observation.insert(this->_observation.end(), body_ang_vel.begin(), body_ang_vel.end());
    this->_observation.insert(this->_observation.end(), obs_projected_gravity.begin(), obs_projected_gravity.end());
    this->_observation.insert(this->_observation.end(), obs_commands.begin(), obs_commands.end());
    this->_observation.insert(this->_observation.end(), dof_pos_vec.begin(), dof_pos_vec.end());
    this->_observation.insert(this->_observation.end(), dof_vel_vec.begin(), dof_vel_vec.end());
    this->_observation.insert(this->_observation.end(), _action.begin(), _action.end());
    for(int i = 0; i < this->_observation.size(); i++)
    {
        this->_observation[i] = std::max(-clip_observations, std::min(this->_observation[i], clip_observations));
    }
}

void State_Loco::_getUserCmd(){
    if (_userValue.ly < -dead_zone)
        _vCmdBody(0) = _userValue.ly * (-_vxLim(0));
    
    else if(_userValue.ly > dead_zone)
        _vCmdBody(0) = _userValue.ly * _vxLim(1);
    else
        _vCmdBody(0) = 0;

    if (_userValue.lx < -dead_zone)
        _vCmdBody(1) = _userValue.lx * (-_vyLim(0));
    else if (_userValue.lx > dead_zone)
        _vCmdBody(1) = _userValue.lx * _vyLim(1);
    else
        _vCmdBody(1) = 0;

    _vCmdBody(2) = 0;
    _vCmdBody = _vCmdBodyPast * this->_cmdSmoothes + _vCmdBody * (1 - this->_cmdSmoothes);
    _dYawCmd =  invNormalize(_userValue.rx, _wyawLim(0), _wyawLim(1));
    _dYawCmd = _dYawCmd;
    _dYawCmdPast = _dYawCmd;
    _vCmdBodyPast = _vCmdBody;
}

void State_Loco::_action_compute()
{

    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    std::vector<Ort::Value> input_tensors;
    std::vector<int64_t> obs_shape = {1, 96};        
    std::vector<int64_t> state_shape = {1, 1, 256};  
    input_tensors.push_back(Ort::Value::CreateTensor<float>(memory_info, _observation.data(), _observation.size(), obs_shape.data(), obs_shape.size()));  // obs输入
    input_tensors.push_back(Ort::Value::CreateTensor<float>(memory_info, _h_state_.data(), _h_state_.size(), state_shape.data(), state_shape.size()));  // h_in输入
    input_tensors.push_back(Ort::Value::CreateTensor<float>(memory_info, _c_state_.data(), _c_state_.size(), state_shape.data(), state_shape.size()));  // c_in输入
   
    auto output_tensors = _session->Run(
    Ort::RunOptions{nullptr}, 
    _input_names.data(), input_tensors.data(), input_tensors.size(),
    _output_names.data(), _output_names.size());
   
    float* actions = output_tensors[0].GetTensorMutableData<float>();
    float* h_out = output_tensors[1].GetTensorMutableData<float>();
    float* c_out = output_tensors[2].GetTensorMutableData<float>();

    std::memcpy(_action.data(), actions, _action.size() * sizeof(float));

    std::memcpy(_h_state_.data(), h_out, _h_state_.size() * sizeof(float));
    std::memcpy(_c_state_.data(), c_out, _c_state_.size() * sizeof(float));
    
    std::vector<float> actions_scaled(_action.size());
    for(int i=0; i<NUM_DOF; i++)
    {
        _action[i] = std::max(-clip_actions, std::min(_action[i], clip_actions));
        actions_scaled[i] = _action[i] * action_scale; 
        this->_joint_q[dof_mapping[i]] = actions_scaled[i];
        this->_joint_q[dof_mapping[i]] += this->_default_dof_pos[dof_mapping[i]];
    }
}

void State_Loco::enter()
{
    _terminate_flag = false;
    for (int i = 0; i < NUM_DOF; i++)
    {
        _lowCmd->motorCmd[i].mode = 10;
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q; 
        _lowCmd->motorCmd[i].dq = 0;
        _lowCmd->motorCmd[i].tau = 0;
        _lowCmd->motorCmd[i].Kp = this->dof_Kps[i];
        _lowCmd->motorCmd[i].Kd = this->dof_Kds[i];
        this->_targetPos_rl[i] = this->_default_dof_pos[i]; 
        this->_last_targetPos_rl[i] = _lowState->motorState[i].q;
        this->_joint_q[i] = this->_default_dof_pos[i];
    }

    _init_buffers(); 
    for (int i = 0; i < this->_num_obs_history; ++i) {
        _observations_compute(); 
    }

}

void State_Loco::run()
{
    _observations_compute(); 
    _action_compute();  
    memcpy(this->_targetPos_rl, this->_joint_q, sizeof(this->_joint_q));
   
    for (int j = 0; j < NUM_DOF; j++) 
    {
        _lowCmd->motorCmd[j].mode = 10;
        _lowCmd->motorCmd[j].q = _targetPos_rl[j];
        _lowCmd->motorCmd[j].dq = 0;
        _lowCmd->motorCmd[j].tau = 0;
        _lowCmd->motorCmd[j].Kp = this->dof_Kps[j];
        _lowCmd->motorCmd[j].Kd = this->dof_Kds[j];
        this->_last_targetPos_rl[j] = _targetPos_rl[j];
    }
}

void State_Loco::exit()
{
    _h_state_ = std::vector<float>(_hidden_size_, 0.0f);
    _c_state_ = std::vector<float>(_hidden_size_, 0.0f);
}

FSMStateName State_Loco::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B) 
    {
        return FSMStateName::PASSIVE;
    }
    else if(_terminate_flag)
    {
        std::cout << "LocoMode terminate" << std::endl;
        return FSMStateName::PASSIVE;
    }
    else if (_lowState->userCmd == UserCommand::R1_UP)
    { 
        return FSMStateName::WBC;
    }
    else if(_lowState->userCmd == UserCommand::SELECT){
        throw std::runtime_error("exit..");
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::R2_A){
        return FSMStateName::AMP;
    }
    else{  
        return FSMStateName::LOCO;
    }
}
