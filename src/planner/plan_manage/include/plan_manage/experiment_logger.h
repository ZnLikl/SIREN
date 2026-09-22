// experiment_logger.h
#pragma once
#include <fstream>
#include <string>
#include <iomanip>
#include <ros/ros.h>

struct ExpStats
{
    // 基本信息
    int    drone_id = -1;
    int    plan_seq = 0;
    double stamp    = 0.0;   // ros::Time::now().toSec()

    // 时间
    double t_init   = 0.0;
    double t_opt    = 0.0;
    double t_refine = 0.0;
    double t_total  = 0.0;

    // cost
    double J_total    = 0.0;
    double J_smooth   = 0.0;
    double J_feas     = 0.0;
    double J_swarm    = 0.0;
    double J_terminal = 0.0;
    double min_ellip_dist = 0.0;

    // swarm / 安全
    double min_dist_to_others    = 0.0;
    // 成功 / 失败
    bool   success            = true;

    // 攻击相关
    int    attack_type         = 0;
    int    phantom_id          = -1;
    int    num_phantoms        = 0;
};

class ExperimentLogger
{
public:
    ExperimentLogger() = default;

    void init(const std::string& filename)
    {
        ofs_.open(filename.c_str(), std::ios::out | std::ios::app);
        if (!ofs_.is_open())
        {
            ROS_ERROR("ExperimentLogger: cannot open %s", filename.c_str());
            return;
        }
        // 新文件的话写表头
        if (ofs_.tellp() == 0)
        {
            ofs_ << "drone_id,plan_seq,stamp,"
                 << "t_init,t_opt,t_refine,t_total,"
                 << "J_total,J_smooth,J_feas,J_swarm,J_terminal,min_ellip_dist,"
                 << "min_dist_to_others,"
                 << "success,attack_type,phantom_id,num_phantoms,"
                 << "\n";
        }
    }

    void log(const ExpStats& s)
    {
        if (!ofs_.is_open()) return;

        ofs_ << s.drone_id << ","
             << s.plan_seq << ","
             << std::fixed << std::setprecision(6) << s.stamp << ","

             << s.t_init << ","
             << s.t_opt << ","
             << s.t_refine << ","
             << s.t_total << ","

             << s.J_total << ","
             << s.J_smooth << ","
             << s.J_feas << ","
             << s.J_swarm << ","
             << s.J_terminal << ","
             << s.min_ellip_dist << ","

             << s.min_dist_to_others << ","

             << (s.success ? 1 : 0) << ","

             << s.attack_type << ","
             << s.phantom_id << ","
             << s.num_phantoms << ","

             << "\n";

        ofs_.flush(); // 仿真里我建议打开，方便随时中断后看结果
    }

private:
    std::ofstream ofs_;
};
