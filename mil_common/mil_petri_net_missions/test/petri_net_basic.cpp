#include <gtest/gtest.h>
#include <mil_petri_net_missions/petri_net.hpp>
#include "arithmetic_chain.hpp"
#include "mult.hpp"
#include "timing.hpp"

std::string make_log(const std::string& name)
{
  int status = std::system("mkdir -p ~/.mil/petri_net_tests");
  if (status == -1)
  {
    throw std::runtime_error("Failed to create directory for logging test states");
  }
  std::string log_file = std::string(std::getenv("HOME")) + "/.mil/petri_net_tests/" + name;
  return log_file;
}

TEST(PetriNetBasic, arithmeticChainTest)
{
  int n = 10;
  petri_net::PetriNet pn(make_log("arithmetic_chain.log"));
  petri_net::tests::arithmetic_chain(pn, false);
  pn.StartTokens({ { "Add", petri_net::make_token(std::make_pair(5, n)) } });
  int result = petri_net::ReturnGet<int>(pn.Spin(), "../Sub").at(0);
  ASSERT_TRUE(result == n);
}

bool test_timing_results(const petri_net::PlaceTypeTokenVec& pttv)
{
  bool ret = true;
  std::map<int, int> times;
  auto base_time = petri_net::ReturnGet<ros::Time>(pttv, "../Wait 0").at(0);
  for (const auto& place : pttv)
  {
    const auto& name = place.first;
    for (const auto& type_vec : place.second)
    {
      auto time = petri_net::token_cast<ros::Time>(type_vec.second.at(0)).sec - base_time.sec;
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
  petri_net::PetriNet pn(make_log("timing.log"));
  petri_net::tests::timing(pn);
  pn.StartTokens({ { "Start", petri_net::make_token(petri_net::empty{}) } });
  auto ptt = petri_net::token_cast<petri_net::PlaceTypeTokenVec>(pn.Spin());
  ASSERT_TRUE(test_timing_results(ptt));
}

TEST(PetriNetBasic, subnetTest)
{
  petri_net::PetriNet pn(make_log("subnet.log"));
  pn.AddSubNet("Arithmetic Chain",
               [](petri_net::PetriNet& _pn) -> void { petri_net::tests::arithmetic_chain(_pn, false); });
  pn.AddSubNet("Timing", &petri_net::tests::timing);
  pn.StartTokens({ { "Arithmetic Chain/Add", petri_net::make_token(std::make_pair(5, 10)) },
                   { "Timing/Start", petri_net::make_token(petri_net::empty{}) } });
  pn.ConnectPlaceTransition("Timing/Return/Place", "Return/Transition", "PlaceTypeTokenVec",
                            typeid(petri_net::PlaceTypeTokenVec).hash_code());
  pn.ConnectPlaceTransition("Arithmetic Chain/Return/Place", "Return/Transition", "PlaceTypeTokenVec",
                            typeid(petri_net::PlaceTypeTokenVec).hash_code());
  auto res = petri_net::token_cast<petri_net::PlaceTypeTokenVec>(pn.Spin());
  auto arithmetic_chain_res =
      petri_net::ReturnGet<petri_net::PlaceTypeTokenVec>(res, "../Arithmetic Chain/Return/Place").at(0);
  ASSERT_TRUE(petri_net::ReturnGet<int>(arithmetic_chain_res, "../Sub").at(0) == 10);
  auto timing_res = petri_net::ReturnGet<petri_net::PlaceTypeTokenVec>(res, "../Timing/Return/Place").at(0);
  ASSERT_TRUE(test_timing_results(timing_res));
}

TEST(PetriNetBasic, multiTypeTest)
{
  petri_net::PetriNet pn(make_log("multi_type.log"));
  petri_net::tests::mult(pn);
  pn.StartTokens({ { "Mult", petri_net::make_token(std::make_pair(3, 7)) },
                   { "Mult", petri_net::make_token(std::make_pair(3, 0.7)) },
                   { "Mult", petri_net::make_token(std::make_pair(0.3, 7)) },
                   { "Mult", petri_net::make_token(std::make_pair(0.3, 0.7)) } });
  const auto& res = petri_net::token_cast<petri_net::PlaceTypeTokenVec>(pn.Spin());
  int count = 0;
  int int_int_count = 0;
  int double_int_count = 0;
  int double_double_count = 0;
  for (const auto& i : res.at("../Mult"))
  {
    for (const auto& j : i.second)
    {
      if (i.first == typeid(double).hash_code())
      {
        count++;
        ASSERT_TRUE(j.use_count() == 1);
        if (petri_net::token_cast<double>(j) == 3 * 0.7 || petri_net::token_cast<double>(j) == 0.3 * 7)
        {
          double_int_count++;
        }
        else if (petri_net::token_cast<double>(j) == 0.3 * 0.7)
        {
          double_double_count++;
        }
      }
      else if (i.first == typeid(int).hash_code())
      {
        count++;
        ASSERT_TRUE(j.use_count() == 1);
        if (petri_net::token_cast<int>(j) == 21)
        {
          int_int_count += 1;
        }
      }
    }
  }
  ASSERT_TRUE(count == 4);
  ASSERT_TRUE(int_int_count == 1);
  ASSERT_TRUE(double_int_count == 2);
  ASSERT_TRUE(double_double_count == 1);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
