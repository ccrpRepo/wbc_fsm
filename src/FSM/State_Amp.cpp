#include <iostream>
#include "FSM/State_Amp.h"
#include "common/read_traj.h"
#include <fstream>
#include <algorithm>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

State_AMP::State_AMP(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::AMP, "amp"){

    std::string config_path = std::string(PROJECT_ROOT_DIR) + "/config/amp.json";
    std::ifstream config_file(config_path);
    if (!config_file.is_open()) {
        std::cerr << "[ERROR] Failed to open config file: " << config_path << std::endl;
        throw std::runtime_error("Cannot open config file");
    }

    try
    {
        json config = json::parse(config_file);
        std::string base_path = std::string(PROJECT_ROOT_DIR) + "/";
        _model_path = base_path + config["model_path"].get<std::string>();
        _anchor_terminate_thresh = config["safe_projgravity_threshold"].get<float>();

        this->_vxLim = {config["vx_limit_min"].get<float>(), config["vx_limit_max"].get<float>()};
        this->_vxLim_slow = {config["vx_limit_min_slow"].get<float>(), config["vx_limit_max_slow"].get<float>()};
        this->_vyLim = {config["vy_limit_min"].get<float>(), config["vy_limit_max"].get<float>()};
        this->_wyawLim = {config["wyaw_limit_min"].get<float>(), config["wyaw_limit_max"].get<float>()};
        this->_cmdSmoothes = config["cmd_smoothes"].get<float>();
        std::cout << "[Config AMP] Model path: " << _model_path << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[ERROR] Failed to parse config file: " << e.what() << std::endl;
        throw;
    }
    config_file.close();

    _loadPolicy();
}

void State_AMP::_getUserCmd(){
    if (_userValue.ly < -dead_zone)
        if (_high_speed_mode)
            _vCmdBody[0] = _userValue.ly * (-_vxLim[0]);
        else
            _vCmdBody[0] = _userValue.ly * (-_vxLim_slow[0]);

        else if (_userValue.ly > dead_zone)
            if (_high_speed_mode)
                _vCmdBody[0] = _userValue.ly * _vxLim[1];
            else
                _vCmdBody[0] = _userValue.ly * _vxLim_slow[1];
        else
            _vCmdBody[0] = 0;

    if (_userValue.lx < -dead_zone)
        _vCmdBody[1] = _userValue.lx * (-_vyLim[0]);
    else if (_userValue.lx > dead_zone)
        _vCmdBody[1] = _userValue.lx * _vyLim[1];
    else
        _vCmdBody[1] = 0;

    if (_userValue.rx < -dead_zone)
        _vCmdBody[2] = _userValue.rx * (-_wyawLim[0]);
    else if (_userValue.rx > dead_zone)
        _vCmdBody[2] = _userValue.rx * _wyawLim[1];
    else
        _vCmdBody[2] = 0;
    
    for(int i=0; i<3; i++)
    {
        _vCmdBody[i] = _vCmdBodyPast[i] * this->_cmdSmoothes + _vCmdBody[i] * (1 - this->_cmdSmoothes);
    }
    // std::cout << "User Command - velx: " << _vCmdBody[0] << ", vely: " << _vCmdBody[1] << ", yaw: " << _vCmdBody[2] << std::endl;
    _vCmdBodyPast = _vCmdBody;
}

void State_AMP::_init_buffers()
{
    for (int i = 0; i < this->_actor_state_history_length; ++i)
    {
        _observations_compute(); 
    }
}

void State_AMP::_loadPolicy() 
{
    auto available_providers = Ort::GetAvailableProviders();
    _session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    _session = std::make_unique<Ort::Session>(_env, _model_path.c_str(), _session_options);

    Ort::TypeInfo input_type = _session->GetInputTypeInfo(0);
    auto input_shapes = input_type.GetTensorTypeAndShapeInfo().GetShape();
    Ort::TypeInfo output_type = _session->GetOutputTypeInfo(0);
    auto output_shapes = output_type.GetTensorTypeAndShapeInfo().GetShape();

    _obs_size_ = input_shapes[1];
    _action_size_ = output_shapes[1];
    _action = std::vector<float>(_action_size_, 0.0f);

}

void State_AMP::_observations_compute()
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
    std::vector<float> waist_yrp(3);
    for(int i=0; i<3; i++)
    {
        waist_yrp[i] = _lowState->motorState[_waist_yrp_idx[i]].q;
    }
    _userValue = _lowState->userValue;
    _getUserCmd();

    Eigen::Matrix3f R_b2w = rotz(waist_yrp[0]) * rotx(waist_yrp[1]) * roty(waist_yrp[2]);
    Eigen::Matrix3f R_base = matrix_from_quat(base_quat);
    Eigen::Matrix3f R_waist = R_base * R_b2w;
    std::vector<float> waist_quat = quat_from_matrix(R_waist);

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

    body_ang_vel[0] = body_ang_vel[0] * scale_lin_vel; 
    body_ang_vel[1] = body_ang_vel[1] * scale_lin_vel;  
    body_ang_vel[2] = body_ang_vel[2] * scale_ang_vel;  
    for(int i=0; i<NUM_DOF; i++)
    {
        dof_pos_vec[i] = dof_pos_vec[i] * scale_dof_pos;  
        dof_vel_vec[i] = dof_vel_vec[i] * scale_dof_vel;  
    }
    std::vector<float> current_robot_state;
    current_robot_state.reserve(3 + 3 + 3 + dof_pos_vec.size() + dof_vel_vec.size() + _action.size());
    current_robot_state.insert(current_robot_state.end(), body_ang_vel.begin(), body_ang_vel.end());
    current_robot_state.insert(current_robot_state.end(), obs_projected_gravity.begin(), obs_projected_gravity.end());
    current_robot_state.insert(current_robot_state.end(), _vCmdBody.begin(), _vCmdBody.end());
    current_robot_state.insert(current_robot_state.end(), dof_pos_vec.begin(), dof_pos_vec.end());
    current_robot_state.insert(current_robot_state.end(), dof_vel_vec.begin(), dof_vel_vec.end());
    current_robot_state.insert(current_robot_state.end(), _action.begin(), _action.end());
    
    _robot_state_obs_buf.erase(_robot_state_obs_buf.begin(),
                                _robot_state_obs_buf.begin() + _robot_state_dim);
    _robot_state_obs_buf.insert(_robot_state_obs_buf.end(),
                                current_robot_state.begin(),
                                current_robot_state.end());


    float anchor_proj_gravity_error = std::abs(projected_gravity[2] - (-1.0f));
    if (anchor_proj_gravity_error > _anchor_terminate_thresh)
    {
        _terminate_flag = true;
        std::cout << "current _anchor_terminate_thresh: " << _anchor_terminate_thresh << std::endl;
        std::cout << "[Warning] Large anchor projected gravity error: " << anchor_proj_gravity_error << std::endl;
    }

    this->_observation.clear();
    this->_observation = std::vector<float>();
    this->_observation.reserve(_robot_state_obs_buf.size());
    this->_observation.insert(this->_observation.end(), _robot_state_obs_buf.begin(), _robot_state_obs_buf.end());
    
    for(int i = 0; i < this->_observation.size(); i++)
    {
        this->_observation[i] = std::max(-clip_observations, std::min(this->_observation[i], clip_observations));
    }

}

void State_AMP::_action_compute()
{
    try
    {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU);

        std::vector<Ort::Value> input_tensors;
        std::vector<int64_t> obs_shape = {1,
            _robot_state_dim * _actor_state_history_length
        }; 

        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info,
            _observation.data(),
            _observation.size(),
            obs_shape.data(),
            obs_shape.size()));

        auto output_tensors = _session->Run(
            Ort::RunOptions{nullptr},
            _input_names.data(),
            input_tensors.data(),
            input_tensors.size(),
            _output_names.data(),
            1
        );

        float *actions = output_tensors[0].GetTensorMutableData<float>();
        std::memcpy(_action.data(), actions, _action.size() * sizeof(float));

        std::vector<float> actions_scaled(_action.size());
        for (int i = 0; i < _action.size(); i++)
        {
            _action[i] = std::max(-clip_actions, std::min(_action[i], clip_actions));
            actions_scaled[i] = _action[i] * action_scale + _default_dof_pos[dof_mapping[i]]; // action_scale
            this->_joint_q[dof_mapping[i]] = actions_scaled[i];
        }
    }
    catch (const Ort::Exception &e)
    {   
        std::cerr << "ONNX Runtime error: " << e.what() << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Standard exception: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Unknown error occurred" << std::endl;
    }
}

void State_AMP::enter()
{
    _high_speed_mode = false;
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
}

void State_AMP::run()
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

void State_AMP::exit()
{
    std::cout << "[State_AMP] Exiting AMP state." << std::endl;
}

FSMStateName State_AMP::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)
    {
        return FSMStateName::PASSIVE;
    }
    else if (_terminate_flag)
    {
        std::cout << "AMP Mode terminate" << std::endl;
        return FSMStateName::PASSIVE;
    }
    else if (_lowState->userCmd == UserCommand::R1_UP)
    {
        return FSMStateName::WBC;
    }
    else if (_lowState->userCmd == UserCommand::SELECT)
    {
        throw std::runtime_error("exit..");
        return FSMStateName::PASSIVE;
    }
    else if (_lowState->userCmd == UserCommand::R2_UP)
    {
        if(!_high_speed_mode)
        {
            std::cout << "Switching to high speed mode" << std::endl;
            _high_speed_mode = true;
        }
        return FSMStateName::AMP;
    }
    else if (_lowState->userCmd == UserCommand::R2_DOWN)
    {
        if(_high_speed_mode)
        {
            std::cout << "Switching to low speed mode" << std::endl;
            _high_speed_mode = false;
        }
        return FSMStateName::AMP;
    }
     else if(_lowState->userCmd == UserCommand::R2_B){
        return FSMStateName::LOCO;
    }
    else
    {
        return FSMStateName::AMP;
    }
}