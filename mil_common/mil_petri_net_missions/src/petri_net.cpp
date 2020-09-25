#include <mil_petri_net_missions/petri_net.hpp>

namespace petri_net
{
Token make_token(const std::any& data)
{
  return std::make_shared<std::any>(data);
}

Token make_token()
{
  return std::make_shared<std::any>();
}
//////////////////////// ThreadSafe
void ThreadSafe::SafeDo(std::function<void(ThreadSafe&)> _thing)
{
  mutex_.lock();
  _thing(*this);
  mutex_.unlock();
}

//////////////////////// OutChannel
OutChannel::OutChannel(const std::string& _name, TokenQueue& _msg_q) : name_(_name), msg_q_(_msg_q)
{
}

//////////////////////// PetriNet
PetriNet::PetriNet(const std::string& _debug_file_name)
  : thread_pool_(*this), debug_flag_(true), debug_(new Debug(_debug_file_name, *this))
{
  Init();
}

PetriNet::PetriNet() : thread_pool_(*this), debug_flag_(false), debug_(nullptr)
{
  Init();
}

void PetriNet::Init()
{
  namespace_ = "/";
  sub_nets_ = { { "/", "subgraph cluster_" + std::to_string(sub_nets_.size()) + "\n{\n" + "label=\"/\";\n" } };
  AddReturnSubNet();
  places_.at("/Return/Place").is_return_place_ = true;
}

void PetriNet::AddReturnSubNet()
{
  std::string name;
  GetNamespace(name);
  SetNamespace(name + "Return/");
  sub_nets_.emplace(name + "Return/", "subgraph cluster_" + std::to_string(sub_nets_.size()) + "\n{\n" + "label=\"" +
                                          name + "Return/\";\n");

  AddTransition("Transition");
  AddPlace("Place", { { typeid(PlaceTypeTokenVec).hash_code(), "PlaceTypeTokenVec" } });
  SetPlaceCallback("Place", [&](const Token _t) -> Token { return _t; });
  ConnectTransitionPlace("Transition", "Place", [](const PlaceTypeTokenVec& _in) -> Token { return make_token(_in); });

  places_.at(name + "Return/Place").is_return_place_ = false;
  SetNamespace(name);
}

PetriNet::~PetriNet()
{
  if (debug_ != nullptr)
    delete debug_;
}

void PetriNet::AddPlace(const std::string& _name, const std::map<std::size_t, std::string>& _in_types)
{
  auto name = namespace_ + _name;
  places_.emplace(name, Place(name, _in_types, *this));
}

void PetriNet::SetPlaceCallback(const std::string& _name, std::function<Token(Token)> _callback)
{
  auto name = namespace_ + _name;
  places_.at(name).callback_ = _callback;
}

void PetriNet::AddTransition(const std::string& _name)
{
  auto name = namespace_ + _name;
  transitions_.emplace(name, Transition(namespace_, *this));
}

void PetriNet::ConnectPlaceTransition(const std::string& _p, const std::string& _t, const std::type_info& _type,
                                      const int _quantity)
{
  ConnectPlaceTransition(_p, _t, demangle(_type.name()), _type.hash_code(), _quantity);
}

void PetriNet::ConnectPlaceTransition(const std::string& _p, const std::string& _t, const std::string& _type_name,
                                      const std::size_t _type_hash, const int _quantity)
{
  auto p = namespace_ + _p;
  auto t = namespace_ + _t;
  if (places_.find(p) == places_.end())
  {
    throw std::runtime_error(p + " is not a registered place");
  }
  if (transitions_.find(t) == transitions_.end())
  {
    throw std::runtime_error(t + " is not a registered transition");
  }

  if (places_.at(p).HasOutType(_type_hash))
  {
    places_.at(p).AddOutEdge(_type_name, _type_hash, t);
  }
  else
  {
    msg_qs_.emplace_back();
    places_.at(p).AddOutEdge(_type_name, _type_hash, t, msg_qs_.back());
  }
  transitions_.at(t).AddInEdge(p, _type_hash, places_.at(p).GetTokenQueue(_type_hash), _quantity);
}

void PetriNet::ConnectTransitionPlace(const std::string& _t, const std::string& _p,
                                      std::function<Token(const PlaceTypeTokenVec&)> _token_mux)
{
  auto p = namespace_ + _p;
  auto t = namespace_ + _t;
  if (transitions_.find(t) == transitions_.end())
  {
    throw std::runtime_error(t + " is not a valid Transition");
  }
  if (places_.find(p) == places_.end())
  {
    throw std::runtime_error(p + " is not a valid Place");
  }
  transitions_.at(t).AddOutEdge(p, _token_mux);
}

void PetriNet::AddSubNet(const std::string& _namespace, std::function<void(PetriNet&)> _subnet)
{
  std::string name;
  GetNamespace(name);
  SetNamespace(name + _namespace + "/");
  sub_nets_.emplace(name + _namespace + "/", "subgraph cluster_" + std::to_string(sub_nets_.size()) + "\n{\n" +
                                                 "label=\"" + name + _namespace + "/\";\n");
  AddReturnSubNet();
  _subnet(*this);
  places_.at(name + _namespace + "/Return/Place").is_return_place_ = false;
  SetNamespace(name);
}

void PetriNet::AsDot(std::ostream& _out) const
{
  int ci = sub_nets_.size();
  _out << "digraph G \n{\n";
  auto sub_nets = sub_nets_;
  for (const auto& place : places_)
  {
    // determine what subnet is in
    std::string ns = place.first.substr(0, place.first.rfind("/")) + "/";
    sub_nets.at(ns) += "\"" + place.first + "\"[label=\"" + place.first + "." +
                       std::to_string(place.second.in_progress_) + "\"\n" + "shape=\"oval\"\n" + "xlabel=\"";
    for (const auto& in_type : place.second.in_types_)
    {
      sub_nets.at(ns) += in_type.second + ",\n";
    }
    sub_nets.at(ns) += "\"];\n";
    for (const auto& out_channel : place.second.out_edges_)
    {
      sub_nets.at(ns) = sub_nets.at(ns) + "subgraph cluster_" + std::to_string(ci++) + "\n{\n" + "label=\"" +
                        out_channel.second.name_ + "/\";\n" + "style=\"filled\";\n" + "\"" + out_channel.second.name_ +
                        "/Transition\"[shape=\"rectangle\"];\n" + "\"" + out_channel.second.name_ + "/Place." +
                        std::to_string(out_channel.second.msg_q_.size()) + "\"[shape=" + "\"oval\"];\n" + "\"" +
                        out_channel.second.name_ + "/Transition\"->\"" + out_channel.second.name_ + "/Place." +
                        std::to_string(out_channel.second.msg_q_.size()) + "\";\n" + "}\n" + "\"" + place.first +
                        "\"->\"" + out_channel.second.name_ + "/Transition\";\n";
      for (const auto& transition : out_channel.second.transitions_)
      {
        sub_nets.at(ns) += "\"" + out_channel.second.name_ + "/Place." +
                           std::to_string(out_channel.second.msg_q_.size()) + "\"->\"" + transition + "\";\n";
      }
    }
  }
  for (const auto& transition : transitions_)
  {
    // determine what subnet is in
    std::string ns = transition.first.substr(0, transition.first.rfind("/") + 1);
    sub_nets.at(ns) += "\"" + transition.first + "\"[shape=\"rectangle\"];\n";
    for (const auto& place : transition.second.out_edges_)
    {
      sub_nets.at(ns) += "\"" + transition.first + "\"->\"" + place.first + "\";\n";
    }
  }
  _out << sub_nets.at("/Return/");
  _out << "}\n";
  sub_nets.erase("/Return/");
  for (auto& subgraph : sub_nets)
  {
    subgraph.second += "}\n";
    _out << subgraph.second;
  }
  _out << "}\n";
}

void PetriNet::Kill(const std::string& _err_msg)
{
  // signal to the thread pool to kill everything
  std::cerr << _err_msg.c_str();
  thread_pool_.Kill();
}

void PetriNet::StartTokens(const std::multimap<std::string, Token>& _start_tokens)
{
  for (const auto& start_msg : _start_tokens)
  {
    thread_pool_.Post(StartMsg{ namespace_ + start_msg.first, start_msg.second });
  }
}

Token PetriNet::Spin()
{
  return_.reset();
  thread_pool_.Spin();
  if (!return_)
  {
    throw std::runtime_error("return is null");
  }
  return return_;
}

void PetriNet::Return(Token _return)
{
  return_ = _return;
}

void PetriNet::str(std::ostream& _out) const
{
  auto& s = _out;
  s << "Places\n\n";
  for (const auto& place : places_)
  {
    s << place.first << "\n";
    place.second.str(s);
  }
  s << "\n\nTransitions\n\n";
  for (const auto& transition : transitions_)
  {
    s << transition.first << "\n";
    transition.second.str(s);
  }
}

PetriNet::ThreadPool::ThreadPool(PetriNet& _net) : net_(_net)
{
}

PetriNet::ThreadPool::~ThreadPool()
{
}

void PetriNet::ThreadPool::Kill()
{
  // terminate all threads
  for (auto i = pool_.begin(); i != pool_.end(); ++i)
  {
    if (std::this_thread::get_id() != i->get_id())
      pool_.erase(i);
  }
  exit(1);
  return;
}

void PetriNet::ThreadPool::Spin()
{
  bool exit = false;
  while (!exit)
  {
    net_.SafeDo([&](ThreadSafe& _ts) -> void {
      auto& net = static_cast<PetriNet&>(_ts);
      // start the threads that need to be started
      if (net_.return_)
      {
        exit = true;
      }
      bool state_changed = false;
      threads_to_start_.SafeDo([&](ThreadSafe& _ts) -> void {
        while (threads_to_start_.size() > 0)
        {
          auto& msg = threads_to_start_.front();
          pool_.push_back(std::thread(&Place::_Callback, &net.places_.at(msg.place_), msg.token_));
          net.places_.at(msg.place_).in_progress_++;
          threads_to_start_.pop();
          state_changed = true;
        }
      });
      if (state_changed && net_.GetDebug())
      {
        net_.DebugOutput();
      }
      // join the threads that need to be joined
      threads_to_join_.SafeDo([&](ThreadSafe& _ts) -> void {
        while (threads_to_join_.size() > 0)
        {
          auto& thread = threads_to_join_.front();
          for (auto i = pool_.begin(); i != pool_.end(); i++)
          {
            if (i->get_id() == thread)
            {
              i->join();
              pool_.erase(i);
              break;
            }
          }
          threads_to_join_.pop();
        }
      });
      if (pool_.size() == 0)
      {
        exit = true;
      }
    });
  }
}

void PetriNet::ThreadPool::Post(const StartMsg& _start_msg)
{
  threads_to_start_.SafeDo([&](ThreadSafe& _ts) -> void { threads_to_start_.push(_start_msg); });
}

void PetriNet::ThreadPool::Join(const std::thread::id _id)
{
  threads_to_join_.SafeDo([&](ThreadSafe& _ts) -> void { threads_to_join_.push(_id); });
}

//////////////////////// Transition
Transition::Transition(const std::string& _namespace, PetriNet& _net) : namespace_(_namespace), net_(_net)
{
}

std::string Transition::DetermineRelativeName(const std::string& _abs_name)
{
  std::string rel_name;

  // if in the same namespace, easy
  if (_abs_name.find(namespace_) == 0)
  {
    rel_name = _abs_name;
    rel_name.erase(0, namespace_.length());
  }
  // else, the name must be from higher up
  else
  {
    std::string ns = namespace_.substr(0, namespace_.size() - 1);
    int common_point = 0;
    int back_count = 0;
    while (_abs_name.find(ns) != 0)
    {
      back_count += 1;
      common_point = ns.rfind('/');
      ns = ns.substr(0, common_point);
    }
    std::string forward = _abs_name.substr(common_point + 1);

    for (int i = 0; i < back_count; ++i)
      rel_name += "../";
    rel_name += forward;
  }
  return rel_name;
}

void Transition::TryFire()
{
  net_.SafeDo([&](ThreadSafe& _ts) -> void {
    // check all the input message quese for any data
    bool all_ins_ready = true;
    for (auto& place__hash_inedge : in_edges_)
    {
      for (auto& hash__inedge : place__hash_inedge.second)
      {
        hash__inedge.second.token_q_.SafeDo([&](ThreadSafe& ts) -> void {
          if (hash__inedge.second.token_q_.size() < hash__inedge.second.quantity_)
            all_ins_ready = false;
        });
        if (!all_ins_ready)
          break;
      }
    }
    // if so, then get the tokens
    if (all_ins_ready)
    {
      PlaceTypeTokenVec in_place_tokens;
      for (auto& place__hash_inedge : in_edges_)
      {
        const auto& abs_name = place__hash_inedge.first;
        std::string rel_name = in_edges_rel_names_.at(abs_name);
        in_place_tokens.emplace(rel_name, TypeTokenVec());
        for (auto& hash__inedge : place__hash_inedge.second)
        {
          hash__inedge.second.token_q_.SafeDo([&](ThreadSafe& _ts) -> void {
            in_place_tokens.at(rel_name).emplace(hash__inedge.first, TokenVec());
            for (int i = 0; i < hash__inedge.second.quantity_; ++i)
            {
              in_place_tokens.at(rel_name).at(hash__inedge.first).push_back(hash__inedge.second.token_q_.front());
              hash__inedge.second.token_q_.pop();
            }
          });
        }
      }
      // distribute those token to the other places
      for (const auto& place : out_edges_)
      {
        StartMsg start_msg{ place.first, place.second(in_place_tokens) };
        net_.thread_pool_.Post(start_msg);
      }
    }
  });
}

void Transition::AddInEdge(const std::string& _place, const std::size_t _hash_code, TokenQueue& _out_channel,
                           const int _quantity)
{
  in_edges_.emplace(_place, TypeInEdge());
  in_edges_.at(_place).emplace(_hash_code, InEdge{ _out_channel, _quantity });
  in_edges_rel_names_.emplace(_place, DetermineRelativeName(_place));
}

void Transition::AddOutEdge(const std::string& _place, const std::function<Token(const PlaceTypeTokenVec&)> _token_mux)
{
  out_edges_.emplace(_place, _token_mux);
}

void Transition::str(std::ostream& _out) const
{
  auto& ret = _out;
  ret << "  In Edges\n";
  for (const auto& i : in_edges_)
  {
    ret << "    " << i.first << "\n";
  }
  ret << "  Out Edges\n";
  for (const auto& i : out_edges_)
  {
    ret << "    " << i.first << "\n";
  }
}

//////////////////////// Place
Place::Place(const std::string& _name, const std::map<std::size_t, std::string>& _in_types, PetriNet& _net)
  : name_(_name), in_types_(_in_types), net_(_net), in_progress_(0), is_return_place_(false)
{
}

void Place::_Callback(const Token _token)
{
  // see if the type of the given token is an accepted type
  // if it is not
  if (in_types_.find(_token->type().hash_code()) == in_types_.end())
  {
    // if not, then error and send a signal to the Petri Net to kill who whole thing
    net_.Kill(name_ + ": Given token of type " + demangle(_token->type().name()) + " is not supported\n");
    return;
  }
  const auto token = callback_(_token);
  // see if the returned Token type is accepted by any of our out
  //   edges unless this is THE return_place
  if (is_return_place_)
  {
    net_.SafeDo([&](ThreadSafe& _ts) -> void {
      net_.Return(token);
      net_.thread_pool_.Join(std::this_thread::get_id());
      net_.DebugOutput();
    });
    return;
  }
  // if not supported return type, kill the net
  if (out_edges_.find(token->type().hash_code()) == out_edges_.end())
  {
    // error the type returned is wrong
    net_.Kill(name_ + ": Returned Token of type " + demangle(token->type().name()) + " is not supported\n");
    return;
  }
  auto& out = out_edges_.at(token->type().hash_code());

  // if We are debugging, it is important to do everything in a critical section
  if (net_.GetDebug())
  {
    net_.SafeDo([&](ThreadSafe& _ts) -> void {
      in_progress_--;
      out.msg_q_.SafeDo([&](ThreadSafe& _ts) -> void { out.msg_q_.push(token); });
      net_.DebugOutput();
    });
  }
  // if not, the we do not have to
  else
  {
    in_progress_--;
    out.msg_q_.SafeDo([&](ThreadSafe& _ts) -> void { out.msg_q_.push(token); });
  }
  // iterate through and call the transitions that belong to that type
  for (auto& transition : out.transitions_)
  {
    net_.transitions_.at(transition).TryFire();
  }
  net_.thread_pool_.Join(std::this_thread::get_id());
}

bool Place::HasOutType(const std::type_info& _type) const
{
  return HasOutType(_type.hash_code());
}

bool Place::HasOutType(const std::size_t _type) const
{
  return !(out_edges_.find(_type) == out_edges_.end());
}

void Place::AddOutEdge(const std::string& _type_name, const std::size_t _type_hash, const std::string& _transition)
{
  if (out_edges_.find(_type_hash) == out_edges_.end())
  {
    throw std::runtime_error("new type requires new MessageQueue");
  }
  if (out_edges_.at(_type_hash).transitions_.insert(_transition).second == false)
  {
    throw std::runtime_error("This edge already exists");
  }
}

void Place::AddOutEdge(const std::string& _type_name, const std::size_t _type_hash, const std::string& _transition,
                       TokenQueue& _msg_q)
{
  if (out_edges_.find(_type_hash) != out_edges_.end())
  {
    throw std::runtime_error("this type already has a MessageQueue");
  }
  out_edges_.emplace(_type_hash, OutChannel(name_ + "->" + _type_name, _msg_q));
  out_edges_.at(_type_hash).transitions_.insert(_transition);
}

TokenQueue& Place::GetTokenQueue(const std::type_info& _type)
{
  return GetTokenQueue(_type.hash_code());
}

TokenQueue& Place::GetTokenQueue(const std::size_t _type)
{
  return out_edges_.at(_type).msg_q_;
}

void Place::str(std::ostream& _out) const
{
  auto& ret = _out;
  ret << "  In Types\n";
  for (const auto& i : in_types_)
  {
    ret << "    " << i.second << "\n";
  }
  ret << "  Out Edges\n";
  for (const auto& i : out_edges_)
  {
    ret << "    " << i.second.name_ << "\n";
    for (const auto& j : i.second.transitions_)
    {
      ret << j << "\n";
    }
  }
}

void Debug::Output()
{
  file_stream_.open(file_name_, std::ios_base::app);
  now_ = std::chrono::steady_clock::now();
  file_stream_ << "[Next State of " << file_name_ << "] :\n";
  net_.AsDot(file_stream_);
  file_stream_.close();
}

}  // petri_net
