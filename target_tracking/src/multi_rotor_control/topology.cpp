#include "topology.h"
#include "sensormodel.h"
#include "rel2dsensormodel.h"
#include "cam2dsensormodel.h"

namespace ranav {

AllModels::AllModels() : shares(new int) {
  *shares = 1;
}

AllModels::AllModels(const AllModels &other) :
  shares(other.shares),
  data(other.data)
{
  ++*shares;
}

AllModels& AllModels::operator=(const AllModels &other) {
  if (shares == other.shares)
    return *this;
  if (--*shares == 0) {
    destruct();
  }
  shares = other.shares;
  data = other.data;
  ++*shares;
  return *this;
}

AllModels::~AllModels() {
  if (--*shares == 0)
    destruct();
}

void AllModels::destruct() {
  for (std::map<Index, SensorModel*>::iterator it = data.begin();
      it != data.end(); ++it) {
    delete it->second;
  }
  data.clear();
  delete shares;
}



Topology::Topology() : nA(0), nT(0) {
}

Topology::Topology(const Topology &other) :
  params(other.params),
  nA(other.nA),
  nT(other.nT),
  allModels(other.allModels)
{
  nodes.clear();
  levels.clear();
  levels.resize(other.levels.size());
  for (std::vector<Node*>::const_iterator it = other.nodes.begin();
      it != other.nodes.end(); ++it) {
    Node *n = new Node(*(*it));
    nodes.push_back(n);
    levels[n->level].insert(n);
  }
}

Topology& Topology::operator=(const Topology &other) {
  params = other.params;
  nA = other.nA;
  nT = other.nT;
  allModels = other.allModels;
  nodes.clear();
  levels.clear();
  levels.resize(other.levels.size());
  for (std::vector<Node*>::const_iterator it = other.nodes.begin();
      it != other.nodes.end(); ++it) {
    Node *n = new Node(*(*it));
    nodes.push_back(n);
    levels[n->level].insert(n);
  }
  return *this;
}

Topology::~Topology() {
  for (std::vector<Node*>::iterator it = nodes.begin();
      it != nodes.end(); ++it) {
    delete *it;
  }
  nodes.clear();
  levels.clear();
}

void Topology::init(const TParam &p) {
  params = p;
  nA = p("multi_rotor_control/numAgents").toInt();
  nT = p("multi_rotor_control/numTargets").toInt();
  if (nT != 1) {
    std::cerr << "Error: Topology is not yet implemented for other than 1 target\n";
    exit(-1);
  }
  levels.clear();
  std::string type = p("multi_rotor_control/initialTopology").toString();
  int increment = 1;
  if (type == "flat") {
    increment = 0;
  } else if (type == "string") {
    increment = 1;
  } else {
    std::cerr << "Error: Unknown initial topology '" << type << "' - exiting\n";
    exit(-1);
  }
  levels.resize(1+(nA-1)*increment);
  for (int i=0; i<nA; ++i) {
    int level = i*increment;
    Node *n = new Node(i, level);
    nodes.push_back(n);
    levels[level].insert(n);
  }
  std::string typestr = p("multi_rotor_control/type").toString();
  for (int i=-1; i<nA; ++i) {
    for (int j=0; j<nA+nT; ++j) {
      if (i != j) {
        SensorModel *s;
        if (typestr == "point2d") {
          s = new Rel2dSensorModel(i, j);
        } else if (typestr == "rotor2d") {
          s = new Cam2dSensorModel(i, j);
        } else {
          std::cerr << "Error: Unknown type '" << typestr << "' - exiting\n";
          exit(-1);
        }
        s->init(p);
        allModels()[Index(i, j)] = s;
      }
    }
  }
}

bool Topology::operator==(const Topology &other) const {
  if (levels.size() != other.levels.size())
    return false;
  for (unsigned int i=0; i<levels.size(); ++i) {
    if (levels[i].size() != other.levels[i].size())
      return false;

  }
  return true;
}

void Topology::insertLevel(int level) {
  levels.insert(levels.begin()+level, std::set<Node*, PComp<Node> >());
  for (unsigned int i=level+1; i<levels.size(); ++i) {
    for (std::set<Node*>::iterator it = levels[i].begin();
        it != levels[i].end(); ++it) {
      ++(*it)->level;
    }
  }
}

void Topology::deleteLevel(int level) {
  levels.erase(levels.begin()+level);
  for (unsigned int i=level; i<levels.size(); ++i) {
    for (std::set<Node*>::iterator it = levels[i].begin();
        it != levels[i].end(); ++it) {
      --(*it)->level;
    }
  }
}

void Topology::moveNode(int i, int updown) {
  assert(updown == -1 || updown == 1);
  Node *n = nodes[i];
  levels[n->level].erase(n);
  n->level += updown;
  levels[n->level].insert(n);
  if (levels[n->level-updown].empty()) {
    deleteLevel(n->level-updown);
  }
}

std::vector<Topology> Topology::getNeighbors() {
  std::vector<Topology> neighbors;
  neighbors.reserve(levels.size()*2);
  for (unsigned int i=0; i<nodes.size(); ++i) {
    Node *n = nodes[i];
    // check level--
    if (n->level == 0) {
      if (levels[n->level].size() > 1) {
        Topology t(*this);
        t.insertLevel(0);
        t.moveNode(i, -1);
        neighbors.push_back(t);
      }
    } else {
      Topology t(*this);
      t.moveNode(i, -1);
      neighbors.push_back(t);
    }
    // check level++
    if (n->level == (int)levels.size()-1) {
      if (levels[n->level].size() > 1) {
        Topology t(*this);
        t.insertLevel(t.levels.size());
        t.moveNode(i, 1);
        neighbors.push_back(t);
      }
    } else {
      Topology t(*this);
      t.moveNode(i, 1);
      neighbors.push_back(t);
    }
  }
  // delete redundant elements
  for (unsigned int i=0; i<neighbors.size(); ++i) {
    for (unsigned int j=i+1; j<neighbors.size(); ++j) {
      if (neighbors[i] == neighbors[j]) {
        neighbors.erase(neighbors.begin()+j);
        --j; // check this index again since a successor element was moving into this slot
      }
    }
  }
  return neighbors;
}

std::vector<const SensorModel*> Topology::getSensorModels() const {
  std::vector<const SensorModel*> sm;
  // top-level GPS observations
  for (std::set<Node*>::iterator to = levels.front().begin();
      to != levels.front().end(); ++to) {
    sm.push_back(allModels().at(Index(-1, (*to)->id)));
  }
  // inter-agent observations
  for (unsigned int i=0; i<levels.size()-1; ++i) {
    for (std::set<Node*>::iterator from = levels[i].begin();
        from != levels[i].end(); ++from) {
      for (std::set<Node*>::iterator to = levels[i+1].begin();
          to != levels[i+1].end(); ++to) {
        assert(from != to);
        sm.push_back(allModels().at(Index((*from)->id, (*to)->id)));
      }
    }
  }
  // low-level observations of target
  for (std::set<Node*>::iterator from = levels.back().begin();
      from != levels.back().end(); ++from) {
    sm.push_back(allModels().at(Index((*from)->id, nA)));
  }
  return sm;
}

std::vector<SensorModel*> Topology::getSensorModelsNonconst() {
  std::vector<SensorModel*> sm;
  // top-level GPS observations
  for (std::set<Node*>::iterator to = levels.front().begin();
      to != levels.front().end(); ++to) {
    sm.push_back(allModels().at(Index(-1, (*to)->id)));
  }
  // inter-agent observations
  for (unsigned int i=0; i<levels.size()-1; ++i) {
    for (std::set<Node*>::iterator from = levels[i].begin();
        from != levels[i].end(); ++from) {
      for (std::set<Node*>::iterator to = levels[i+1].begin();
          to != levels[i+1].end(); ++to) {
        assert(from != to);
        sm.push_back(allModels().at(Index((*from)->id, (*to)->id)));
      }
    }
  }
  // low-level observations of target
  for (std::set<Node*>::iterator from = levels.back().begin();
      from != levels.back().end(); ++from) {
    sm.push_back(allModels().at(Index((*from)->id, nA)));
  }
  return sm;
}

void Topology::write(std::ostream &os, const Eigen::VectorXd &state) const {
  os << nodes.size()+2 << " -1 -1";
  for (std::vector<Node*>::const_iterator it = nodes.begin();
      it != nodes.end(); ++it) {
    os << " " << (*it)->id << " " << (*it)->level;
  }
  os << " " << nodes.size() << " " << levels.size();
  std::vector<const SensorModel*> sm = getSensorModels();
  os << " " << sm.size();
  for (std::vector<const SensorModel*>::const_iterator it = sm.begin();
      it != sm.end(); ++it) {
    const MultiAgentSensorModel *masm = static_cast<const MultiAgentSensorModel*>(*it);
    os << " " << masm->getFromId() << " " << masm->getToId();
    os << " " << masm->getInformation(state, masm->sense(state));
  }
  os << "\n";
}

} /* namespace ranav */
