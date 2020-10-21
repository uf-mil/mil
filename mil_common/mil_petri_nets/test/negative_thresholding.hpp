#pragma once
#include <stdlib.h> /* srand, rand */
#include <time.h>   /* time */
#include <mil_petri_nets/petri_net.hpp>

namespace petri_net::tests
{
void negative_threshold(PetriNet& pn)
{
  pn.AddPlace("Timer", { in_type<Empty>() }, { out_type<Empty>() });
  pn.SetPlaceCallback("Timer", [](const Token _in) -> Token {
    ros::Time::init();
    ros::Duration(0.05).sleep();
    return _in;
  });
  pn.AddPlace("Buffer", { in_type<int>() }, { out_type<int>() });
  pn.AddPlace("Generator", { in_type<Empty>() }, { out_type<int>() });
  pn.SetPlaceCallback("Generator", [](const Token) -> Token {
    ros::Time::init();
    ros::Duration(0.009).sleep();
    long int thirty_seven = 37;
    srand(time(&thirty_seven));
    return make_token(rand());
  });
  pn.AddTransition("Generator_T");
  pn.ConnectTransitionPlace("Generator_T", "Generator",
                            [](const PlaceTypeTokenVec& _in) -> Token { return make_token(Empty{}); });
  pn.ConnectTransitionPlace("Generator_T", "Buffer", [](const PlaceTypeTokenVec& _in) -> Token {
    return _in.at("Generator").at(hash_code<int>()).at(0);
  });
  pn.ConnectPlaceTransition("Generator", "Generator_T", typeid(int));
  pn.ConnectPlaceTransition("Timer", "Return/Transition", typeid(Empty));
  pn.ConnectPlaceTransition("Buffer", "Return/Transition", typeid(int), -1);
}
}
