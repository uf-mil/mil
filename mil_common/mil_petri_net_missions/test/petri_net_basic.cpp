#include <gtest/gtest.h>
#include <mil_petri_net_missions/petri_net.hpp>
#include "arithmetic_chain.hpp"
#include "timing.hpp"

TEST(PetriNetBasic, arithmeticChainTest)
{
  int n = 10;
  int status = std::system("mkdir -p ~/.mil/petri_net_tests");
  if (status == -1)
  {
    throw std::runtime_error("Failed to create directory for logging test states");
  }
  std::string log_file = std::string(std::getenv("HOME")) + "/.mil/petri_net_tests/arithmetic_chain.log";
  petri_net::PetriNet pn(log_file);
  petri_net::tests::arithmetic_chain(pn, false);
  pn.StartTokens({ { "Add", petri_net::make_token(std::make_pair(5, n)) } });
  int result = petri_net::ReturnGet<int>(pn.Spin(), "../Sub");
  ASSERT_TRUE(result == n);
}

bool test_timing_results(const petri_net::PlaceTypeToken& ptt)
{
  bool ret = true;
  std::map<int, int> times;
  auto base_time = petri_net::ReturnGet<ros::Time>(ptt, "../Wait 0");
  for (const auto& place : ptt)
  {
    const auto& name = place.first;
    for (const auto& type : place.second)
    {
      int time = petri_net::token_cast<ros::Time>(type.second).sec - base_time.sec;
      times.emplace(std::stoi(name.substr(name.find(" ") + 1)), time);
    }
  }
  for (const auto& i : times)
  {
    ret = ret && (i.first == i.second);
  }
  return ret;
}

TEST(PetriNetBasic, timingTest)
{
  int status = std::system("mkdir -p ~/.mil/petri_net_tests");
  if (status == -1)
  {
    throw std::runtime_error("Failed to create directory for logging test states");
  }
  std::string log_file = std::string(std::getenv("HOME")) + "/.mil/petri_net_tests/timing.log";
  petri_net::PetriNet pn(log_file);
  petri_net::tests::timing(pn);
  pn.StartTokens({ { "Start", petri_net::make_token(petri_net::empty{}) } });
  auto ptt = petri_net::token_cast<petri_net::PlaceTypeToken>(pn.Spin());
  ASSERT_TRUE(test_timing_results(ptt));
}

TEST(PetriNetBasic, subnetTest)
{
  int status = std::system("mkdir -p ~/.mil/petri_net_tests");
  if (status == -1)
  {
    throw std::runtime_error("Failed to create directory for logging test states");
  }
  std::string log_file = std::string(std::getenv("HOME")) + "/.mil/petri_net_tests/sub_net.log";
  petri_net::PetriNet pn(log_file);
  pn.AddSubNet("Arithmetic Chain",
               [](petri_net::PetriNet& _pn) -> void { petri_net::tests::arithmetic_chain(_pn, false); });
  pn.AddSubNet("Timing", &petri_net::tests::timing);
  pn.StartTokens({ { "Arithmetic Chain/Add", petri_net::make_token(std::make_pair(5, 10)) },
                   { "Timing/Start", petri_net::make_token(petri_net::empty{}) } });
  pn.ConnectPlaceTransition("Timing/Return/Place", "Return/Transition", "PlaceTypeToken",
                            typeid(petri_net::PlaceTypeToken).hash_code());
  pn.ConnectPlaceTransition("Arithmetic Chain/Return/Place", "Return/Transition", "PlaceTypeToken",
                            typeid(petri_net::PlaceTypeToken).hash_code());
  auto res = petri_net::token_cast<petri_net::PlaceTypeToken>(pn.Spin());
  auto arithmetic_chain_res = petri_net::ReturnGet<petri_net::PlaceTypeToken>(res, "../Arithmetic Chain/Return/Place");
  ASSERT_TRUE(petri_net::ReturnGet<int>(arithmetic_chain_res, "../Sub") == 10);
  auto timing_res = petri_net::ReturnGet<petri_net::PlaceTypeToken>(res, "../Timing/Return/Place");
  ASSERT_TRUE(test_timing_results(timing_res));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
