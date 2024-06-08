#include <mil_missions/client.h>

namespace mil_missions
{
MissionClient::MissionClient(ros::NodeHandle &nh) : ac("/mission", true), _nh(nh) {}

std::vector<std::string> MissionClient::available_missions()
{
    std::vector<std::string> missions;
    if (!_nh.getParam("/available_missions", missions))
    {
        ROS_ERROR("No available missions found");
        return {};
    }
    return missions;
}

void MissionClient::run_mission(std::string mission_name, std::string parameters)
{
    if (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_ERROR("Could not connect to mission server to run mission");
        return;
    }

    mil_missions::DoMissionGoal goal;
    goal.mission = mission_name;
    goal.parameters = parameters;

    ac.sendGoal(goal);
}

void MissionClient::cancel_mission()
{
    ac.cancelAllGoals();
}

bool MissionClient::wait_for_server(ros::Duration timeout)
{
    return ac.waitForServer(timeout);
}

bool MissionClient::wait_for_result(ros::Duration timeout)
{
    return ac.waitForResult(timeout);
}

DoMissionResult::ConstPtr MissionClient::get_result()
{
    return ac.getResult();
}

actionlib::SimpleClientGoalState MissionClient::get_state()
{
    return ac.getState();
}
}  // namespace mil_missions
