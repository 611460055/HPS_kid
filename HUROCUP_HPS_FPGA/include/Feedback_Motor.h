#ifndef FEEDBACK_MOTOR_H_
#define FEEDBACK_MOTOR_H_
#include <stdio.h>
#include <string.h>
#include "Initial.h"
#include "Walkinggait.h"
#include "DataModule.h"

class Feedback_Motor
{
    public:
        Feedback_Motor();
        ~Feedback_Motor();
        void load_motor_data_left_foot();
        void update_motor_data_left_foot();
        int motor_data_left_foot_[6];

        void load_motor_data_right_foot();
        void update_motor_data_right_foot();
        int motor_data_right_foot_[6];

        void load_motor_data_left_hand();
        void update_motor_data_left_hand();
        int motor_data_left_hand_[4];

        void load_motor_data_right_hand();
        void update_motor_data_right_hand();
        int motor_data_right_hand_[4];

        void adjust_left_foot();
        void update_adjust_left_foot();
        int adjust_left_foot_[6];
        int tmp_lf[6];

        void adjust_right_foot();
        void update_adjust_right_foot();
        int adjust_right_foot_[6];
        int tmp_rf[6];

        int torque_update_motor_data_left_foot_flag_;
        int torque_update_motor_data_right_foot_flag_;
        int torque_update_motor_data_left_hand_flag_;
        int torque_update_motor_data_right_hand_flag_;

        std::string DtoS(double value);
	    std::map<std::string, std::vector<double>> map_feedback;      
        void saveData();
        void pushData();

    private:
        bool update_motor_data_left_foot_flag_;
        bool update_motor_data_right_foot_flag_;
        bool update_motor_data_left_hand_flag_;
        bool update_motor_data_right_hand_flag_;
        bool update_adjust_left_foot_flag_;
        bool update_adjust_right_foot_flag_;
        int name_cont_;
	    
};

#endif