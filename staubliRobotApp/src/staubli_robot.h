#ifndef _STAUBLI_ROBOT_H
#define _STAUBLI_ROBOT_H

#include "soapCS8ServerV0Proxy.h"
#include "soapCS8ServerV2Proxy.h"

#define VAL3_doEsStop_mbDO_NDX      0

typedef enum {staubli_no_error=0,
              staubli_login_error=1,
              staubli_invalid_id=2,
              staubli_no_feedback=3,
              staubli_traj_invalid_first_pos=4,
              staubli_small_step=5,
              staubli_no_joint_ranges=6,
              staubli_no_power_change_while_in_motion=7,
              staubli_power_on_timeout=8,
              staubli_power_off_timeout=9,
              staubli_power_change_only_remote=10,
              staubli_invalid_shoulder_conf=11,
              staubli_invalid_elbow_conf=12,
              staubli_invalid_wrist_conf=13,
              staubli_invalid_blend_values=14,
              staubli_invalid_feedback_rate=15,
              staubli_invalid_input_vector_length=16,
              staubli_traj_in_progress=17,
              staubli_invalid_cartesian_speed=18,
              staubli_not_ready=19,
              staubli_invalid_parameter=20,
              staubli_parameter_misuse=21,
              staubli_unexpected_error=22,
              staubli_joints_out_of_range=23,
              staubli_invalid_joint_speed=24,
              staubli_ik_no_solution=25,
              staubli_ik_solution_out_of_range=26,
              staubli_ik_solution_out_of_workspace=27,
              staubli_ik_invalid_conf=28,
              staubli_ik_invalid_orientation=29,
              staubli_ik_unsupported_kin=30,
              staubli_ik_unconstrained_frame=31,
              staubli_fk_no_solution=32,
              staubli_invalid_max_cartesian_speed=33} staubli_errors_t;

class CStaubli_Robot
{
  private:
    // staubli data structures
    CS8ServerV0Proxy *CS8_server_v0;
    CS8ServerV2Proxy *CS8_server_v2;
    std::string end_point_v0;
    std::string end_point_v2;

    _ns1__login login;
    _ns1__loginResponse login_response;
    // staubli status information
    bool logged_in;
    std::string user_name;
    std::string password;
    int sid;
    // error variables
    int last_error;
  protected: 
    void clear(void);
  public:
    CStaubli_Robot(std::string &robot_id, std::string &ip_address, std::string &username, std::string &password);
    void read_ios_value(std::vector<std::string> &physicalLink, std::vector<double> &value);
    int write_ios_value(std::vector<std::string> &ios, std::vector<double> &values);
    int get_sid();
    ~CStaubli_Robot();
};

#endif
