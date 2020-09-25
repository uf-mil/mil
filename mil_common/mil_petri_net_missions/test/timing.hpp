#pragma once
#include <ros/ros.h>
#include <mil_petri_net_missions/petri_net.hpp>
namespace petri_net::tests
{
void timing(PetriNet& pn)
{
  pn.AddPlace("Start", { in_type<empty>() });
  pn.SetPlaceCallback("Start", [](const Token _t) -> Token { return _t; });
  pn.AddTransition("Start_Transition");
  pn.ConnectPlaceTransition("Start", "Start_Transition", typeid(empty));
  for (int i = 0; i < 3; ++i)
  {
    pn.AddPlace("Wait " + std::to_string(i), { in_type<empty>() });
    pn.ConnectTransitionPlace(
        "Start_Transition", "Wait " + std::to_string(i),
        [](const PlaceTypeTokenVec& _in) -> Token { return _in.at("Start").at(hash_code<empty>()).at(0); });
    std::string name = "Wait " + std::to_string(i);
    pn.SetPlaceCallback(name, [=](const Token _in) -> Token {
      ros::Time::init();
      ros::Duration(i).sleep();
      return make_token(ros::Time::now());
    });
    pn.ConnectPlaceTransition(name, "Return/Transition", typeid(ros::Time));
  }
}
}
