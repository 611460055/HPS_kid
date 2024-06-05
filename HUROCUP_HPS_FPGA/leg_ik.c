#include "include/leg_ik.h"

extern LegInverseKinematic LIK;

LegInverseKinematic::LegInverseKinematic()
{
	Vector3d rpy_tmp, pos_tmp;

	// initial狀態下的末端點
	rpy_tmp(0) = -PI;
	rpy_tmp(1) = 0;
	rpy_tmp(2) = PI_2;
	pos_tmp(0) = -0;
	pos_tmp(1) = 4.45;
	pos_tmp(2) = -32.434;
	// 轉換成齊次轉換矩陣
	left_leg.init_endpoint = calculate_endpoint(rpy_tmp, pos_tmp);

	rpy_tmp(0) = -PI;
	rpy_tmp(1) = 0;
	rpy_tmp(2) = PI_2;
	pos_tmp(0) = -0;
	pos_tmp(1) = -4.45;
	pos_tmp(2) = -32.434;
	right_leg.init_endpoint = calculate_endpoint(rpy_tmp, pos_tmp);

	// cout << "left_leg.init_endpoint = "<< endl << left_leg.init_endpoint.matrix() << endl;
	// cout << "right_leg.init_endpoint = "<< endl << right_leg.init_endpoint.matrix() << endl;

	left_leg.now_endpoint = left_leg.init_endpoint;	// 現在的末端點位置
	left_leg.cal_endpoint = left_leg.init_endpoint;	// 計算的末端點位置
	right_leg.now_endpoint = right_leg.init_endpoint;
	right_leg.cal_endpoint = right_leg.init_endpoint;

	// stand狀態下的末端點	!!!!!!要再測量
	rpy_tmp(0) = -PI;
	rpy_tmp(1) = 0;
	rpy_tmp(2) = PI_2;
	pos_tmp(0) = -0;
	pos_tmp(1) = 4.45;
	pos_tmp(2) = -32.434;
	// 轉換成齊次轉換矩陣
	left_leg.stand_endpoint = calculate_endpoint(rpy_tmp, pos_tmp);

	rpy_tmp(0) = -PI;
	rpy_tmp(1) = 0;
	rpy_tmp(2) = PI_2;
	pos_tmp(0) = -0;
	pos_tmp(1) = -4.45;
	pos_tmp(2) = -32.434;
	right_leg.stand_endpoint = calculate_endpoint(rpy_tmp, pos_tmp);

	// 設置左右手base
	left_leg.base.translation() << 0, LEG_L1, -LEG_L2;
	left_leg.base.linear() << 0, -1, 0, -1, 0, 0, 0, 0, -1;
	right_leg.base.translation() << 0, -LEG_L1, -LEG_L2;
	right_leg.base.linear() << 0, -1, 0, -1, 0, 0, 0, 0, -1;
	
	// cout << "left_leg.base = "<< endl << left_leg.base.matrix() << endl;
	// cout << "right_leg.base = "<< endl << right_leg.base.matrix() << endl;

	// 設置手臂的關節活動限制	!!!!!!要再測量
	theta_min(0) = 0;
	theta_min(1) = -3*PI_2;
	theta_min(2) = -3*PI_2;
	theta_min(3) = -30*DEGREE_2_PI;
	theta_min(4) = -30*DEGREE_2_PI;
	theta_min(5) = -30*DEGREE_2_PI;

	theta_max(0) = 2*PI; // 195*PI/180;
	theta_max(1) = PI_2;
	theta_max(2) = PI_2;
	theta_max(3) = 210*DEGREE_2_PI;
	theta_max(4) = 210*DEGREE_2_PI;
	theta_max(5) = 210*DEGREE_2_PI;

	// 設置手臂末端點禁止進入的範圍
	// x_min = -10;
	// x_max = 9;
	// y_min = -13.5;
	// y_max = 13.5;
	
	left_leg.ik_error = false;		// 左手逆運動學錯誤旗標
	right_leg.ik_error = false;		// 右手逆運動學錯誤旗標

	name_cont_ = 0;
	std::vector<float> temp;
	if(map_lik.empty())
	{
		map_lik["l_cal_theta1"] = temp;
		map_lik["l_cal_theta2"] = temp;
		map_lik["l_cal_theta3"] = temp;
		map_lik["l_cal_theta4"] = temp;
		map_lik["r_cal_theta1"] = temp;
		map_lik["r_cal_theta2"] = temp;
		map_lik["r_cal_theta3"] = temp;
		map_lik["r_cal_theta4"] = temp;		
		map_lik["l_ik_error"] = temp;
		map_lik["r_ik_error"] = temp;
	}
}

LegInverseKinematic::~LegInverseKinematic()
{

}

void LegInverseKinematic::initial_ik()
{
	// 獲取initial狀態下的末端點
	right_leg.cal_endpoint = right_leg.init_endpoint;
	left_leg.cal_endpoint = left_leg.init_endpoint;
	// cout << "Get initial point." << endl;
}

void LegInverseKinematic::after_initial_ik()
{
	// initial_inverse_kinematic()後，將cal_endpoint改回now_endpoint
	right_leg.cal_endpoint = right_leg.now_endpoint;
	left_leg.cal_endpoint = left_leg.now_endpoint;
	// cout << "After initial point." << endl;
}

Vector12d LegInverseKinematic::run()
{	
	// 計算手臂逆運動學的主程式
	// cout << "Run hand inverse kinematic." << endl;

	// 檢查末端點是否在可活動範圍
	// bool l_activities_error = activities_area_check(left_leg.cal_endpoint);
	// bool r_activities_error = activities_area_check(right_leg.cal_endpoint);
	bool l_activities_error = false;
	bool r_activities_error = false;

	// 計算手臂馬達角度
	if(!l_activities_error)
	{
		left_leg.save_rad = calculate_leg_ik(left_leg.cal_endpoint, left_leg.base);		// 左手
		// cout << "left_leg.save_rad = " << endl << left_leg.save_rad << endl;

		// 檢查計算的角度是否超過限制
		bool l_limit_error = limit_check(left_leg.save_rad);
		if(!l_limit_error)
		{
			left_leg.output_rad = left_leg.save_rad;	// 輸出弧度
		}			
		else
		{
			left_leg.ik_error = true;
			// cout << "left hand limit error!" << endl;
		}			
	}
	else
	{
		left_leg.ik_error = true;
		// cout << "left hand activities error!" << endl;
	}		
		
	if(!r_activities_error)
	{
		right_leg.save_rad = calculate_leg_ik(right_leg.cal_endpoint, right_leg.base);	// 右手
		// cout << "right_leg.save_rad = " << endl << right_leg.save_rad << endl;

		// 檢查計算出來的角度是否超過限制
		bool r_limit_error = limit_check(right_leg.save_rad);
		if(!r_limit_error)
		{
			right_leg.output_rad = right_leg.save_rad;	// 輸出弧度
		}			
		else
		{
			right_leg.ik_error = true;
			// cout << "right hand limit error!" << endl;
		}			
	}
	else
	{
		right_leg.ik_error = true;
		// cout << "right hand activities error!" << endl;
	}	

	// 輸出結果
	Vector12d output;
	output << left_leg.output_rad, right_leg.output_rad;
	// cout << "output rad = " << output << endl;

	pushData();

	return output;
}

Affine3d LegInverseKinematic::calculate_endpoint(Vector3d rpy, Vector3d pos)
{	
	// 將rpy與pos轉成齊次轉換矩陣
	Affine3d end_point;
	end_point.linear() = get_rotation_matrix(rpy);
	end_point.translation() = pos;

	return end_point;
}

Vector6d LegInverseKinematic::calculate_leg_ik(Affine3d end_point, Affine3d base)
{	
	// 手臂逆運動學
	Vector6d theta;
	Affine3d rev_base;
	Affine3d t_70;
	// 取得T_07
	rev_base.linear() = base.rotation().transpose();
	rev_base.translation() = -rev_base.rotation()*base.translation();
	Affine3d t_07 = rev_base*end_point;
	// 取得T_70
	t_70.linear() = t_07.rotation().transpose();
	t_70.translation() = -t_70.rotation()*t_07.translation();
	Matrix4d matrix = t_70.matrix();

	double nx_ = matrix(0,0);
	double ny_ = matrix(1,0);
	double nz_ = matrix(2,0);
	double ox_ = matrix(0,1);
	double oy_ = matrix(1,1);
	double oz_ = matrix(2,1);
	double ax_ = matrix(0,2);
	double ay_ = matrix(1,2);
	double az_ = matrix(2,2);
	double px_ = matrix(0,3);
	double py_ = matrix(1,3);
	double pz_ = matrix(2,3);

	// 計算theta4
	double c4 = (pow(px_, 2) + pow(py_, 2) + pow(pz_+LEG_L5, 2) - pow(LEG_L3, 2) - pow(LEG_L4, 2))/(2*LEG_L3*LEG_L4);
	double tmp_ = 1-pow(c4, 2);
	if(tmp_ < 0)
		tmp_ = 0;

	double s4 = -sqrt(tmp_);
	theta(3) = atan2(s4, c4);

	// 計算theta5
	theta(4) = atan2(-py_, -sqrt(pow(px_, 2) + pow(pz_+LEG_L5, 2))) - atan2(LEG_L3*s4, LEG_L4+LEG_L3*c4);
	if(theta(4) > 2*PI)
		theta(4) -= 2*PI;
	else if(theta(4) < 0)
		theta(4) += 2*PI;

	double c5 = cos(theta(4));
	double s5 = sin(theta(4));
	double c45 = c4*c5-s4*s5;

	// 計算theta6
	theta(5) = atan2(-px_, pz_+LEG_L5);

	if(c45*LEG_L3+c5*LEG_L4 < 0)
		theta(5) += PI;

	if (theta(5) > PI)
		theta(5) -= 2*PI;
	else if (theta(5) < -PI)
		theta(5) += 2*PI;

	double c6 = cos(theta(5));
	double s6 = sin(theta(5));

	// 計算theta2
	double c2 = c6*ax_+s6*az_;
	double tmp_1 = 1-pow(c2, 2);
	if(tmp_1 < 0)
		tmp_1 = 0;
	double s2 = sqrt(tmp_1);

	theta(1) = atan2(s2, c2);

	// 計算theta1
	theta(0) = atan2(-c6*ox_-s6*oz_, -c6*nx_-s6*nz_);
	if(s2 < 0)
		theta(0) += PI;
	if(theta(0) > PI)
		theta(0) -= 2*PI;
	else if(theta(0) < -PI)
		theta(0) += 2*PI;
	
	// 計算theta345
	double theta345 = atan2(ay_, s6*ax_-c6*az_);
	if(s2 < 0)
		theta345 += PI;

	// 計算theta3
	theta(2) = theta345 - theta(3) - theta(4);
	if(theta(2) > PI)
		theta(2) -= 2*PI;
	else if(theta(2) < -PI)
		theta(2) += 2*PI;

	return theta;
}

bool LegInverseKinematic::limit_check(Vector6d theta)
{
	// 檢查計算出來的角度是否超過限制	
	for(int i=0;i<4;i++)
	{
		if(theta(i) < theta_min(i))
		{
			// cout << "theta(" << i << ") min error!" << endl;
			return true;
		}
		else if(theta(i) > theta_max(i))
		{
			// cout << "theta(" << i << ") max error!" << endl;
			return true;
		}
	}
	return false;
}

/*bool LegInverseKinematic::activities_area_check(Affine3d end_point)
{
	// 檢查末端點是否進入禁止活動範圍
	Vector4d point;

	Matrix4d matrix = end_point.matrix();
	point << 0, 0, HAND_L4, 1;
	Vector4d point_base = matrix*point;

	if(point_base(1) < y_max && point_base(1) > y_min && point_base(0) < x_max && point_base(0) > x_min)
	{
		// cout << "In inaccessible area." << endl;
		return true;
	}

	return false;
}*/

void LegInverseKinematic::stand()
{
	// 獲取stand狀態下的末端點
	right_leg.cal_endpoint = right_leg.stand_endpoint;
	right_leg.now_endpoint = right_leg.stand_endpoint;

	left_leg.cal_endpoint = left_leg.stand_endpoint;
	left_leg.now_endpoint = left_leg.stand_endpoint;

	// cout << "Get stand point." << endl;
}

void LegInverseKinematic::pushData()
{
	map_lik["l_cal_theta1"].push_back(left_leg.save_rad(0));
	map_lik["l_cal_theta2"].push_back(left_leg.save_rad(1));
	map_lik["l_cal_theta3"].push_back(left_leg.save_rad(2));
	map_lik["l_cal_theta4"].push_back(left_leg.save_rad(3));
	map_lik["r_cal_theta1"].push_back(right_leg.save_rad(0));
	map_lik["r_cal_theta2"].push_back(right_leg.save_rad(1));
	map_lik["r_cal_theta3"].push_back(right_leg.save_rad(2));
	map_lik["r_cal_theta4"].push_back(right_leg.save_rad(3));
	map_lik["l_ik_error"].push_back(left_leg.ik_error);
	map_lik["r_ik_error"].push_back(right_leg.ik_error);
}

void LegInverseKinematic::saveData()
{
    char path[200] = "/data";
	std::string tmp = std::to_string(name_cont_);
	tmp = "/HIK"+tmp+".csv";
    strcat(path, tmp.c_str());

    fstream fp;
    fp.open(path, std::ios::out);
	std::string savedText;
    std::map<std::string, std::vector<float>>::iterator it_lik;

	for(it_lik = map_lik.begin(); it_lik != map_lik.end(); it_lik++)
	{
		savedText += it_lik->first;
		if(it_lik == --map_lik.end())
		{
			savedText += "\n";
			fp<<savedText;
			savedText = "";
		}
		else
		{
			savedText += ",";
		}		
	}
	it_lik = map_lik.begin();
	int max_size = it_lik->second.size();

	for(it_lik = map_lik.begin(); it_lik != map_lik.end(); it_lik++)
	{
		if(max_size < it_lik->second.size())
            max_size = it_lik->second.size();
	}
	for(int i = 0; i < max_size; i++)
    {
        for(it_lik = map_lik.begin(); it_lik != map_lik.end(); it_lik++)
        {
            if(i < it_lik->second.size())
            {
                if(it_lik == --map_lik.end())
                {
                    savedText += std::to_string(it_lik->second[i]) + "\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                {
                    savedText += std::to_string(it_lik->second[i]) + ",";
                }
            }
            else
            {
                if(it_lik == --map_lik.end())
                {
                    savedText += "none\n";
                    fp<<savedText;
                    savedText = "";
                }
                else
                    savedText += "none,";
            }
        }
    }
    fp.close();
    for(it_lik = map_lik.begin(); it_lik != map_lik.end(); it_lik++)
        it_lik->second.clear();

    name_cont_++;
}

LegTrajectory::LegTrajectory()
{
	get_position = false;
	set_speed = false;
	start = false;
	position << 0, 0, 0;
	rpy_radians << 0, 0, 0;
}

LegTrajectory::~LegTrajectory()
{

}

void LegTrajectory::initial()
{
	get_position = false;
	set_speed = false;
	position << 0, 0, 0;
	rpy_radians << 0, 0, 0;
}

void LegTrajectory::set_parameter(bool left_leg_)
{
	if(left_leg_)
	{
		LIK.left_leg.cal_endpoint = LIK.calculate_endpoint(rpy_radians, position);
	}
	else
	{
		LIK.right_leg.cal_endpoint = LIK.calculate_endpoint(rpy_radians, position);
	}

}