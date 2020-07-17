#pragma once
#include <algorithm>
#include <any>
#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <list>
#include <map>
#include <mutex>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_set>

#include <stdio.h>
#include <stdlib.h>

#include <boost/bind.hpp>
#include <boost/core/demangle.hpp>

namespace petri_net
{
struct empty
{
};

using boost::core::demangle;
typedef std::shared_ptr<std::any> Token;
typedef std::map<std::size_t, Token> TypeToken;
typedef std::map<std::string, TypeToken> PlaceTypeToken;

Token make_token(const std::any& data);

template <typename T>
std::pair<std::size_t, std::string> in_type()
{
  return std::make_pair<std::size_t, std::string>(typeid(T).hash_code(), demangle(typeid(T).name()));
}

template <typename T>
std::size_t hash_code()
{
  return typeid(T).hash_code();
}

template <typename T>
T token_cast(Token t)
{
  return std::any_cast<T>(*t);
}

template <typename T>
T ReturnGet(const PlaceTypeToken& _ptt, const std::string _place)
{
  TypeToken tt;
  try
  {
    tt = _ptt.at(_place);
  }
  catch (std::out_of_range& e)
  {
    throw std::out_of_range(_place + " had no output tokens to Return\n" + e.what());
  }
  Token t;
  try
  {
    t = tt.at(hash_code<T>());
  }
  catch (std::out_of_range& e)
  {
    throw std::out_of_range(_place + " had no output tokens of the type " + typeid(T).name() + " to Return\n" +
                            e.what());
  }
  return token_cast<T>(t);
}

template <typename T>
T ReturnGet(Token _t, const std::string _place)
{
  PlaceTypeToken ptt;
  try
  {
    ptt = token_cast<PlaceTypeToken>(_t);
  }
  catch (std::bad_any_cast& e)
  {
    if (!_t)
    {
      throw std::runtime_error(std::string("The Returned Token is empty.\n") + e.what());
    }
    else
    {
      throw std::runtime_error(std::string("The Returned Token is not a PlaceTypeToken. It is a ") + _t->type().name() +
                               "\n" + e.what());
    }
  }
  return ReturnGet<T>(ptt, _place);
}

class ThreadSafe
{
public:
  virtual ~ThreadSafe() = default;

  void SafeDo(std::function<void(ThreadSafe&)> _thing);

protected:
  std::mutex mutex_;
};

template <typename T>
class MessageQueue : public std::queue<T>, public ThreadSafe
{
};
typedef MessageQueue<Token> TokenQueue;
typedef std::map<std::size_t, TokenQueue&> TypeTokenQueueRef;
typedef std::map<std::string, TypeTokenQueueRef> PlaceTypeTokenQueueRef;

class Place;
struct StartMsg
{
  const std::string place_;
  const Token token_;
};

struct OutChannel
{
  OutChannel(const std::string& _name, TokenQueue& _msg_q);

  TokenQueue& msg_q_;
  std::unordered_set<std::string> transitions_;
  const std::string name_;
};

class PetriNet;

class Debug
{
public:
  Debug(const std::string& _file_name, PetriNet& _net) : file_name_(_file_name), net_(_net)
  {
    // override if the file is there
    file_stream_.open(file_name_, std::ios::out);
    if (!file_stream_)
      throw std::runtime_error("File did open");
    file_stream_.close();
  }
  void Output();
  void Start()
  {
    begin_ = std::chrono::steady_clock::now();
  }

private:
  std::chrono::steady_clock::time_point begin_;
  std::chrono::steady_clock::time_point now_;
  PetriNet& net_;
  std::ofstream file_stream_;
  const std::string file_name_;
};

class Transition;
class Place;

class PetriNet : ThreadSafe
{
  class ThreadPool
  {
  public:
    ThreadPool(PetriNet& _net);

    ~ThreadPool();

    void Kill();

    void Spin();

    void Post(const StartMsg& _start_msg);

    void Join(const std::thread::id _id);

  private:
    std::list<std::thread> pool_;
    // msg q to give the threads that need to be run and with what params
    MessageQueue<StartMsg> threads_to_start_;
    MessageQueue<std::thread::id> threads_to_join_;

    PetriNet& net_;
  };

public:
  PetriNet();
  PetriNet(const std::string& _debug_file_name);
  ~PetriNet();

  // functions that are used to build the petri net, cannot be called after the net is started
  void AddPlace(const std::string& _name, const std::map<std::size_t, std::string>& _in_types);

  void SetPlaceCallback(const std::string& _name, std::function<Token(const Token)> _callback);

  void AddTransition(const std::string& _name);

  // add an edge from a place to a transition
  void ConnectPlaceTransition(const std::string& _p, const std::string& _t, const std::type_info& _type);
  void ConnectPlaceTransition(const std::string& _p, const std::string& _t, const std::string& _type_name,
                              const std::size_t _type_hash);

  // add an edge from a transition to a place
  void ConnectTransitionPlace(const std::string& _t, const std::string& _p,
                              std::function<Token(const PlaceTypeToken&)> _token_mux);

  bool GetDebug()
  {
    return debug_flag_;
  }

  void DebugOutput()
  {
    if (debug_ != nullptr)
      debug_->Output();
  }

  void AsDot(std::ostream& _out) const;

  // destroy all semaphores, message quese, child threads, delete all shared dynamic memory
  void Kill(const std::string& _err_msg = std::string(""));

  // Places call to signal a close of the petri net
  void StartTokens(const std::map<std::string, Token>& _start_tokens);

  Token Spin();

  void AddSubNet(const std::string& _namespace, std::function<void(PetriNet&)>);
  void GetNamespace(std::string& _out)
  {
    _out = namespace_;
  }

  void str(std::ostream& _out) const;

private:
  friend class Transition;
  friend class Place;
  void Init();
  void AddReturnSubNet();
  void Return(Token _return);
  void SetNamespace(const std::string& _namespace)
  {
    namespace_ = _namespace;
  }

  std::map<std::string, Place> places_;
  std::map<std::string, Transition> transitions_;
  std::list<TokenQueue> msg_qs_;

  ThreadPool thread_pool_;

  Token return_;
  bool debug_flag_;
  Debug* debug_;

  std::string namespace_;
  std::map<std::string, std::string> sub_nets_;
};

class Transition
{
public:
  void TryFire();

  void AddInEdge(const std::string& _place, const std::size_t _hash_code, TokenQueue& _out_channel);

  void AddOutEdge(const std::string& _place, const std::function<Token(const PlaceTypeToken&)> _token_mux);

  void str(std::ostream& _out) const;

private:
  friend class PetriNet;
  Transition(const std::string& _namepsace, PetriNet& _net);

  std::string DeterminRelativeName(const std::string& _abs_name);

  PetriNet& net_;

  PlaceTypeTokenQueueRef in_edges_;
  std::map<std::string, std::string> in_edges_rel_names_;

  std::string namespace_;

  std::map<std::string, std::function<Token(const PlaceTypeToken&)>> out_edges_;
};

class Place
{
public:
  bool HasOutType(const std::type_info& _type) const;
  bool HasOutType(const std::size_t _type) const;

  void str(std::ostream& _out) const;

private:
  friend class PetriNet;

  Place(const std::string& _name, const std::map<std::size_t, std::string>& _in_types, PetriNet& _net);

  void AddOutEdge(const std::string& _type_name, const std::size_t _type_hash, const std::string& _transition);

  void AddOutEdge(const std::string& _type_name, const std::size_t _type_hash, const std::string& _transition,
                  TokenQueue& _msg_q);

  TokenQueue& GetTokenQueue(const std::type_info& _type);
  TokenQueue& GetTokenQueue(const std::size_t _type);

  void _Callback(const Token _in);

  const std::string name_;
  const std::map<std::size_t, std::string> in_types_;
  std::map<std::size_t, OutChannel> out_edges_;
  PetriNet& net_;
  int in_progress_;
  bool is_return_place_;

  std::function<Token(const Token)> callback_ = [](const Token _token) -> Token { return _token; };
};
}
