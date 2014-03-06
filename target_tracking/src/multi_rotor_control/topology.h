#ifndef TOPOLOGY_H_
#define TOPOLOGY_H_

#include <iostream>
#include <vector>
#include <deque>
#include <map>
#include <set>
#include "tparam.h"

namespace ranav {

class SensorModel;

struct Node {
  Node(int id, int level) : id(id), level(level) {}
  bool operator<(const Node &other) const { return id < other.id; }
  int id;
  int level;
};

// internal sensor model storage
struct Index {
  Index(int from, int to) : from(from), to(to) {}
  bool operator<(const Index &other) const { return from < other.from || (from == other.from && to < other.to); }
  int from, to;
};

class AllModels {
public:
  AllModels();
  AllModels(const AllModels &other);
  AllModels& operator=(const AllModels &other);
  ~AllModels();
  std::map<Index, SensorModel*>& operator()() { return data; }
  const std::map<Index, SensorModel*>& operator()() const { return data; }
private:
  void destruct();
  int *shares;
  std::map<Index, SensorModel*> data;
};

template <typename T>
struct PComp {
  bool operator()(const T *a, const T *b) const {
    return *a < *b;
  }
};

//! topology does not include top level -1: GPS and bottom level N: target
class Topology {
public:
  Topology();
  Topology(const Topology &other);
  Topology& operator=(const Topology &other);
  virtual ~Topology();

  bool operator==(const Topology &other) const;

  virtual void init(const TParam &p);

  void setAllSensorModels(const AllModels &models) { allModels = models; }

  //! returns all neighbor topologies of this
  std::vector<Topology> getNeighbors();

  //! returns the sensor models according to this topology
  std::vector<const SensorModel*> getSensorModels() const;
  // TODO: HACK for AR.Drone experiments
  std::vector<SensorModel*> getSensorModelsNonconst();

  //! writes this topology to the given stream for plotting (the nodes -1: GPS and N: target are added)
  //! state according to EKF motion model definition
  void write(std::ostream &os, const Eigen::VectorXd &state) const;

protected:
  std::vector<Node*> nodes; //!< list of all nodes
  std::deque< std::set<Node*, PComp<Node> > > levels; //!< list of all levels providing the nodes of each level
  void moveNode(int i, int updown);
  void insertLevel(int level);
  void deleteLevel(int level);
  TParam params;
  int nA; //!< number of agents
  int nT; //!< number of targets

  AllModels allModels;
};

} /* namespace ranav */

#endif /* TOPOLOGY_H_ */
