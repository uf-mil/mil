#include <mil_petri_net_missions/petri_net.hpp>
namespace petri_net::tests
{
void arithmetic_chain(PetriNet& pn, bool _print)
{
  pn.AddPlace("Add", { in_type<std::pair<int, int>>() });

  pn.SetPlaceCallback("Add", [=](const Token _in) -> Token {
    auto in = token_cast<std::pair<int, int>>(_in);
    int res = in.first + in.second;
    if (_print)
    {
      std::cout << std::to_string(in.first) + " + " + std::to_string(in.second) + " = " + std::to_string(res) + "\n";
    }
    return make_token(res);
  });
  pn.AddTransition("Add to Mult");
  pn.ConnectPlaceTransition("Add", "Add to Mult", typeid(int));

  pn.AddPlace("Mult", { in_type<std::pair<int, int>>() });
  pn.SetPlaceCallback("Mult", [=](const Token _in) -> Token {
    auto in = token_cast<std::pair<int, int>>(_in);
    int res = in.first * in.second;
    if (_print)
    {
      std::cout << std::to_string(in.first) + " * " + std::to_string(in.second) + " = " + std::to_string(res) + "\n";
    }
    auto t = make_token(res);
    return t;
  });
  pn.ConnectTransitionPlace("Add to Mult", "Mult", [&](const PlaceTypeToken& _in) -> Token {
    return make_token(std::make_pair(5, token_cast<int>(_in.at("Add").at(hash_code<int>()))));
  });
  pn.AddTransition("Mult to Div");

  pn.AddPlace("Div", { in_type<std::pair<int, int>>() });
  pn.SetPlaceCallback("Div", [=](const Token _in) -> Token {
    auto in = token_cast<std::pair<int, int>>(_in);
    int res = in.first / in.second;
    if (_print)
    {
      std::cout << std::to_string(in.first) + " / " + std::to_string(in.second) + " = " + std::to_string(res) + "\n";
    }
    return make_token(res);
  });
  pn.ConnectPlaceTransition("Mult", "Mult to Div", typeid(int));
  pn.ConnectTransitionPlace("Mult to Div", "Div", [](const PlaceTypeToken& _in) -> Token {
    return make_token(std::make_pair(token_cast<int>(_in.at("Mult").at(hash_code<int>())), 5));
  });

  pn.AddTransition("Div to Sub");
  pn.AddPlace("Sub", { in_type<std::pair<int, int>>() });
  pn.SetPlaceCallback("Sub", [=](const Token _in) -> Token {
    auto in = token_cast<std::pair<int, int>>(_in);
    int res = in.first - in.second;
    if (_print)
    {
      std::cout << std::to_string(in.first) + " - " + std::to_string(in.second) + " = " + std::to_string(res) + "\n";
    }
    auto ret = make_token(res);
    return ret;
  });
  pn.ConnectPlaceTransition("Div", "Div to Sub", typeid(int));
  pn.ConnectTransitionPlace("Div to Sub", "Sub", [](const PlaceTypeToken& _in) -> Token {
    return make_token(std::make_pair(token_cast<int>(_in.at("Div").at(hash_code<int>())), 5));
  });
  pn.ConnectPlaceTransition("Sub", "Return/Transition", typeid(int));
}
}
