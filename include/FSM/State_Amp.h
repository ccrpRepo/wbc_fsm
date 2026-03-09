#ifndef STATE_AMP_H
#define STATE_AMP_H

#include "FSM/FSMState.h"
#include "common/read_traj.h"
#include "common/mathTools.h"
#include <onnxruntime_cxx_api.h>
#include <cstring>
#include <vector>
#include <array>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>
#include <cmath>
#include <stdexcept>

#define NUM_DOF 29

using namespace ArmatureConstants;

class State_AMP : public FSMState
{
public:
    State_AMP(CtrlComponents *ctrlComp);
    ~State_AMP() = default;
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
 
private:
    Ort::Env _env;
    Ort::SessionOptions _session_options;
    std::unique_ptr<Ort::Session> _session;
    Ort::AllocatorWithDefaultOptions _allocator;

    const std::vector<const char*> _input_names = {"obs"};
    const std::vector<const char*> _output_names = {"actions"};

    std::vector<int64_t> _input_shape;
    std::vector<int64_t> _output_shape;
    int64_t _obs_size_;
    int64_t _hidden_size_;
    int64_t _action_size_;

    bool _start_flag = false;
    bool _init_obs = false;
    float _targetPos_rl[29]; 
    float _last_targetPos_rl[29];  

    void _loadPolicy();
    void _observations_compute();
    void _action_compute();
    void _getUserCmd();

    const float dead_zone = 0.2f;
    std::vector<float> _vCmdBody = {0.0f, 0.0f, 0.0f};
    std::vector<float> _vCmdBodyPast = {0.0f, 0.0f, 0.0f};
    std::vector<float> _vxLim = {0.0f, 0.0f};
    std::vector<float> _vyLim = {0.0f, 0.0f};
    std::vector<float> _wyawLim = {0.0f, 0.0f};

    std::vector<float> _vxLim_slow = {0.0f, 0.0f};
    bool _high_speed_mode = false;

    double _cmdSmoothes;

    const float clip_observations = 100.0;
    const float clip_actions = 100.0;  
    const float action_scale = 0.25; 
    const float hip_scale_reduction = 1.0;  

    const float scale_lin_vel = 1.0;
    const float scale_ang_vel = 1.0;
    float scale_dof_pos = 1.0;
    float scale_dof_vel = 1.0;
    float _joint_q[29];

    std::vector<float> _action;  // 动作向量
    std::vector<float> _observation;  // observation vector
    std::vector<float> _current_dof_pos;  // current dof_pos
    std::vector<float> _current_dof_vel;  // current dof_vel
    
    void _init_buffers();  // 参数初始化
            
    std::string _model_path;
    std::vector<float> _target_dof_pos;
    
    const int _actor_state_history_length = 4;
    const int _robot_state_dim = 96; 
    const int _anchor_idx = 0; // reference anchor index torso_link: 9 root: 0
    const int _waist_yrp_idx[3] = {12, 13, 14}; // reference trajectory waist rpy indices
    
    const std::vector<float> _gravity_vec = {0.0f, 0.0f, -1.0f}; 
    float _anchor_terminate_thresh = 0.5f; 
    bool _terminate_flag = false; 
    
    std::vector<float> _robot_state_obs_buf = std::vector<float>(_robot_state_dim * _actor_state_history_length, 0.0f);

    const float _default_dof_pos[NUM_DOF] = {-0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
                                        -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
                                        0.0, 0.0, 0.0, 
                                        0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
                                        0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0,};

    const int dof_mapping[NUM_DOF] = {0, 6, 12,
                                      1, 7, 13,
                                      2, 8, 14,
                                      3, 9, 15, 22,
                                      4, 10, 16, 23,
                                      5, 11, 17, 24,
                                      18, 25,
                                      19, 26,
                                      20, 27,
                                      21, 28}; // motor order

    const double dof_Kps[NUM_DOF] = {STIFFNESS_7520_22, STIFFNESS_7520_22, STIFFNESS_7520_14, STIFFNESS_7520_22, 2.0 * STIFFNESS_5020, 2.0 * STIFFNESS_5020,
                                STIFFNESS_7520_22, STIFFNESS_7520_22, STIFFNESS_7520_14, STIFFNESS_7520_22, 2.0 * STIFFNESS_5020, 2.0 * STIFFNESS_5020,
                                STIFFNESS_7520_14, 2.0 * STIFFNESS_5020, 2.0 * STIFFNESS_5020,
                                STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5010_16, STIFFNESS_5010_16,
                                STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5010_16, STIFFNESS_5010_16,}; // 电机Kp参数

    const double dof_Kds[NUM_DOF] = {DAMPING_7520_22, DAMPING_7520_22, DAMPING_7520_14, DAMPING_7520_22, 2.0 * DAMPING_5020, 2.0 * DAMPING_5020,
                                DAMPING_7520_22, DAMPING_7520_22, DAMPING_7520_14, DAMPING_7520_22, 2.0 * DAMPING_5020, 2.0 * DAMPING_5020,
                                DAMPING_7520_14, 2.0 * DAMPING_5020, 2.0 * DAMPING_5020,
                                DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5010_16, DAMPING_5010_16,
                                DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5010_16, DAMPING_5010_16,}; // 电机Kd参数
};

#endif // STATE_AMP_H