#pragma once
#include <mil_petri_nets/petri_net.hpp>

namespace petri_net::tests
{
void mult(PetriNet& pn)
{
  pn.AddPlace("Mult", {
                          in_type<std::pair<int, int>>(), in_type<std::pair<int, double>>(),
                          in_type<std::pair<double, int>>(), in_type<std::pair<double, double>>(),
                      });
  pn.SetPlaceCallback("Mult", [&](const Token _in) -> Token {
    auto type = _in->type().hash_code();
    if (typeid(std::pair<int, int>).hash_code() == type)
    {
      auto& ret = token_cast<std::pair<int, int>>(_in);
      return make_token(ret.first * ret.second);
    }
    else if (typeid(std::pair<int, double>).hash_code() == type)
    {
      auto& ret = token_cast<std::pair<int, double>>(_in);
      return make_token(ret.first * ret.second);
    }
    else if (typeid(std::pair<double, int>).hash_code() == type)
    {
      auto& ret = token_cast<std::pair<double, int>>(_in);
      return make_token(ret.first * ret.second);
    }
    else if (typeid(std::pair<double, double>).hash_code() == type)
    {
      auto& ret = token_cast<std::pair<double, double>>(_in);
      return make_token(ret.first * ret.second);
    }
    pn.Kill(std::string("type ") + demangle(_in->type().name()) + std::string(" was not handled\n"));
  });

  pn.ConnectPlaceTransition("Mult", "Return/Transition", typeid(int), 1);
  pn.ConnectPlaceTransition("Mult", "Return/Transition", typeid(double), 3);
}
}
