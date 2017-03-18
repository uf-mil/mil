#pragma once

#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <Eigen/Dense>

#include "msg_helpers.h"

namespace uf_common {

void fail(std::string const &error_string) { throw std::runtime_error(error_string); }
template <typename FirstType, typename SecondType, typename... MoreTypes>
void fail(FirstType first, SecondType second, MoreTypes... more) {
  std::ostringstream ss;
  ss << first << second;
  return fail(ss.str(), more...);
}

template <typename... ErrorDescTypes>
void require(bool cond, ErrorDescTypes... error_desc) {
  if (!cond) {
    fail(error_desc...);
  }
}

template <typename T, int N>
bool _getParam(ros::NodeHandle &nh, const std::string &name, Eigen::Matrix<T, N, 1> &res) {
  XmlRpc::XmlRpcValue my_list;
  if (!nh.getParam(name, my_list)) return false;
  require(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray, "param ", name, " must be an array");
  if (N != Eigen::Dynamic) {
    require(my_list.size() == N, "param ", name, "must have length ", N, " (is ", my_list.size(),
            ")");
  }

  res.resize(my_list.size());
  for (int32_t i = 0; i < my_list.size(); i++) {
    if (my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      res(i) = static_cast<double>(my_list[i]);
    } else if (my_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      res(i) = static_cast<int>(my_list[i]);
    } else {
      fail("param ", name, "[", i, "] is not numeric");
    }
  }
  return true;
}
template <typename T, int N, int M>
bool _getParam(ros::NodeHandle &nh, const std::string &name, Eigen::Matrix<T, N, M> &res) {
  static_assert(N != 0, "doesn't work for N = 0");
  static_assert(M != 1, "wrong template specialization used?");

  XmlRpc::XmlRpcValue my_list;
  if (!nh.getParam(name, my_list)) return false;
  require(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray, "param ", name, " must be an array");
  require(my_list.size() >= 1, "param " + name + " must not have zero length");
  if (N != Eigen::Dynamic) {
    require(my_list.size() == N, "param ", name, "must have length ", N, " (is ", my_list.size(),
            ")");
  }

  require(my_list[0].getType() == XmlRpc::XmlRpcValue::TypeArray, "param ", name,
          "[0] must be a list");
  if (M != Eigen::Dynamic) {
    require(my_list[0].size() == M, "param ", name, "[0] must have length ", M, " (is ",
            my_list[0].size(), ")");
  }

  res.resize(my_list.size(), my_list[0].size());
  for (int32_t i = 0; i < my_list.size(); i++) {
    XmlRpc::XmlRpcValue row = my_list[i];
    require(row.getType() == XmlRpc::XmlRpcValue::TypeArray, "param ", name, "[", i,
            "] must be a list");
    require(row.size() == my_list[0].size(), "param ", name, "[", i, "]'s length doesn't match");
    for (int32_t j = 0; j < row.size(); j++) {
      XmlRpc::XmlRpcValue entry = row[j];
      if (entry.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        res(i, j) = static_cast<double>(entry);
      } else if (entry.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        res(i, j) = static_cast<int>(entry);
      } else {
        fail("param ", name, "[", i, ", ", j, "] is not numeric");
      }
    }
  }
  return true;
}
template <typename T>
bool _getParam(ros::NodeHandle &nh, const std::string &name, T &res) {
  return nh.getParam(name, res);
}
template <>
bool _getParam(ros::NodeHandle &nh, const std::string &name, ros::Duration &res) {
  double x;
  if (!nh.getParam(name, x)) return false;
  res = ros::Duration(x);
  return true;
}
template <>
bool _getParam(ros::NodeHandle &nh, const std::string &name, unsigned int &res) {
  int x;
  if (!nh.getParam(name, x)) return false;
  if (x < 0) {
    fail("param ", name, " must be >= 0");
  }
  res = static_cast<unsigned int>(x);
  return true;
}

template <typename T>
T getParam(ros::NodeHandle &nh, std::string name) {
  T res;
  require(_getParam(nh, name, res), "param ", name, " required");
  return res;
}
template <typename T>
T getParam(ros::NodeHandle &nh, std::string name, T default_value) {
  T res;
  if (!_getParam(nh, name, res)) {
    return default_value;
  }
  return res;
}
}

