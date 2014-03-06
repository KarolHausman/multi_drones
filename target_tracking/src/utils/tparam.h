/***************************************************************
 * This source code is part of ROBULAR.
 *
 * ROBULAR Copyright (c) 2009 Joerg Mueller, Boris Lau.
 * Autonomous Intelligent Systems, Department of Computer
 * Science, University of Freiburg, Germany.
 **************************************************************/

#ifndef ROBULAR_TPARAM_H
#define ROBULAR_TPARAM_H

#include <iostream>
#include <sstream>
#include <string>
#include <map>
#include <set>
#include <vector>
#include <stack>
#include <cassert>
#include "variant.h"

namespace ranav {

//! parameter containing either a variant parameter value or a couple of children
class TParam : public Variant {
public:
  TParam(const std::string &name = "", const Variant &value = Variant());
  //! copy constructor. if this Param has children, this is a deep copy of the whole (sub-)tree
  TParam(const TParam &other);
  //! destroy this and its children
  virtual ~TParam();
  //! assignment operator (similar to copy constructor)
  TParam &operator=(const TParam &other);
  //! clears all children
  void clear();

  std::string getName() const;
  //! returns the path (without name) of this Param
  std::string getPath() const;
  bool isValid() const;

  //! returns the names of all children of this Param
  std::set<std::string> getChildrenNames() const;

  //! adds a node to the children.
  //! returns whether the operation was successful
  bool addChild(const TParam &n);
  //! same as addChild, but inserts new node into children of the node at path.
  //! If the param is already existent at the given path, it is replaced by <n>.
  //! If the path is not existent, the needed categories are created.
  //! returns whether the operation was successful
  bool addParam(const TParam &n, const std::string &path);

  //! updates this node using <other> (copies nodes from other which are not contained in this
  //! and replaces nodes of this which are contained in both)
  bool update(const TParam &other);

  //! finds the parameter on the given path. If there is no parameter, this function
  //! returns an invalid Param with empty name.
  const TParam& get(const std::string &path) const;
  //! same as above but error message and exit in case the path doesn't point to a parameter
  const TParam& operator()(const std::string &path) const;

  //! write user friendly newline separated indented tree
  void writeTree(std::ostream &s, int indentation = 0) const;
  //! parse tree written with writeTree (or config file). <this> Param is cleared first.
  bool parseTree(std::istream &s);
  //! same as parseTree but loading from file
  void loadTree(const std::string &file);

  //! write in one line
  virtual void write(std::ostream &s) const;
  //! read from one line. <this> Param is cleared first
  virtual bool read(std::istream &s);

  //! returns path part (up to last /) of given string
  static std::string getPath(const std::string &pathAndName);
  //! returns name part (starting after last /) of given string
  static std::string getName(const std::string &pathAndName);

protected:
  // override Variant getters for more meaningful error messages
  virtual void handleTypeMismatch(const Type &typeTo, const std::string &typeNameTo, bool *ok=NULL) const;

  bool parseTree(std::istream &s, std::stack<int> &indentation);

  void copyChildren(const TParam &other);

  std::string name;
  TParam *parent;
  std::map<std::string, TParam*> children;
  static TParam invalid;
};

} // namespace

#endif
