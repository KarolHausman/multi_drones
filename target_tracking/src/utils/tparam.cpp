/***************************************************************
 * This source code is part of ROBULAR.
 *
 * ROBULAR Copyright (c) 2009 Joerg Mueller, Boris Lau.
 * Autonomous Intelligent Systems, Department of Computer
 * Science, University of Freiburg, Germany.
 **************************************************************/

#include "tparam.h"
#include <sstream>
#include <fstream>
#include <cstdlib>

namespace ranav {

TParam TParam::invalid = TParam();

TParam::TParam(const std::string &name, const Variant &v) :
        Variant(v),
        name(name),
        parent(NULL)
{}

TParam::TParam(const TParam &other) :
        Variant(other),
        name(other.name),
        parent(other.parent)
{
  copyChildren(other);
}

TParam::~TParam()
{
  clear();
}

TParam& TParam::operator=(const TParam &other)
{
  Variant::operator=(other);
  name = other.name;
  parent = other.parent;
  clear();
  copyChildren(other);
  return *this;
}

void TParam::clear()
{
  for (std::map<std::string, TParam*>::iterator it = children.begin();
      it != children.end(); it++) {
    delete it->second;
  }
  children.clear();
}

void TParam::copyChildren(const TParam &other)
{
  for (std::map<std::string, TParam*>::const_iterator it = other.children.begin();
      it != other.children.end(); it++) {
    addChild(*it->second);
  }
}

std::string TParam::getName() const
{
  return name;
}

std::string TParam::getPath() const
{
  if (parent && parent->parent){
    if (parent->getPath() == "/"){
      return "/" + parent->name;
    }else{
      return parent->getPath() + "/" + parent->name;
    }
  }else{
    return "/";
  }
}

std::set<std::string> TParam::getChildrenNames() const
{
  std::set<std::string> cn;
  for (std::map<std::string, TParam*>::const_iterator it = children.begin();
      it != children.end(); it++) {
    cn.insert(it->first);
  }
  return cn;
}

bool TParam::isValid() const
{
  return !name.empty();
}

bool TParam::addChild(const TParam &n)
{
  if (n.name.empty()) {
    std::cerr << "Error in " << __PRETTY_FUNCTION__ << ": Empty names are not allowed\n";
    return false;
  }
  if (n.name.find(" ") != std::string::npos) {
    std::cerr << "Error in " << __PRETTY_FUNCTION__ << ": White spaces in names are not allowed. Got '" << n.name << "'.\n";
    return false;
  }
  if (n.name.find("/") != std::string::npos) {
    std::cerr << "Error in " << __PRETTY_FUNCTION__ << ": Slashes '/' in names are not allowed. Got '" << n.name << "'.\n";
    return false;
  }
  if (getTypeByName(n.name) != Invalid) {
    std::cerr << "Error in " << __PRETTY_FUNCTION__ << ": Names of Variant Types are not allowed. Got '" << n.name << "'.\n";
    return false;
  }
  if (getType() != Invalid) {
    std::cerr << "Error in " << __PRETTY_FUNCTION__ << ": Trying to add a child to leaf node.\n";
    return false;
  }
  if (children.count(n.name) > 0) {
    if (children[n.name]->getType() != n.getType()) {
      std::cerr << "Error in " << __PRETTY_FUNCTION__ << ": Trying to replace Param " << n.name << " of type " << children[n.name]->getTypeName() << " by type " << n.getTypeName() << "\n";
      return false;
    }

    delete children[n.name];
  }

  children[n.name] = new TParam(n);
  children[n.name]->parent = this;
  return true;
}

bool TParam::addParam(const TParam &n, const std::string &path)
{
  if (path.empty() || path == ".")
    return addChild(n);
  if (path[0] == '/') {
    if (parent)
      return parent->addParam(n, path);
    else
      return addParam(n, path.substr(1));
  }
  if (path == "..") {
    if (!parent)
      return false;
    return parent->addChild(n);
  }
  if (path.size() > 2 && path.substr(0,3) == "../") {
    if (!parent)
      return false;
    return parent->addParam(n, path.substr(3));
  }
  std::size_t pos = path.find_first_of('/');
  if (pos == std::string::npos) {
    if (children.count(path) == 0) {
      // add missing category
      addChild(TParam(path));
    }
    return children[path]->addParam(n, ".");
  }
  std::string cat = path.substr(0, pos);
  if (children.count(cat) == 0) {
    // add missing category
    addChild(TParam(cat));
  }
  return children[cat]->addParam(n, path.substr(pos+1));
}

bool TParam::update(const TParam &other)
{
  if (name != other.name) {
    std::cerr << "Error in " << __PRETTY_FUNCTION__ << ": Trying to update Param " << name << " with " << other.name << ".\n";
    return false;
  }
  if (getType() != other.getType()) {
    std::cerr << "Error in " << __PRETTY_FUNCTION__ << ": Trying to update Param " << name << " of type " << getTypeName() << " with type " << other.getTypeName() << ".\n";
    return false;
  }
  Variant::operator=(other);
  for (std::map<std::string, TParam*>::const_iterator it = other.children.begin();
      it != other.children.end(); it++) {
    if (children.count(it->first) > 0) {
      if (!children[it->first]->update(*it->second))
        return false;
    } else {
      if (!addChild(*it->second))
        return false;
    }
  }
  return true;
}

const TParam& TParam::get(const std::string &path) const
{
  if (path == ".")
    return *this;
  if (path.empty()) {
    std::cerr << "Error in " << __PRETTY_FUNCTION__ << ": Requested Param not found.\n";
    return invalid;
  }
  if (path[0] == '/') {
    if (parent)
      return parent->get(path);
    else
      return get(path.substr(1));
  }
  if (path == "..") {
    return *parent;
  }
  if (path.size() > 2 && path.substr(0,3) == "../") {
    if (!parent) {
      std::cerr << "Error in " << __PRETTY_FUNCTION__ << ": Requested Param not found.\n";
      return invalid;
    }
    return parent->get(path.substr(3));
  }
  std::size_t pos = path.find_first_of('/');
  if (pos == std::string::npos) {
    if (children.count(path) == 0)
      return invalid;
    return *(children.find(path)->second);
  }
  std::string cat = path.substr(0, pos);
  if (children.count(cat) == 0)
    return invalid;
  return children.find(cat)->second->get(path.substr(pos+1));
}

const TParam& TParam::operator()(const std::string &path) const {
//  std::cerr << "Request for parameter '" << path << "'\n";
  const TParam& p = get(path);
  assert(p.isValid());
  if (!p.isValid()) {
    std::cerr << "Error: Cannot find parameter '" << path << "' - exiting.\n";
    exit(-1);
  }
  return p;
}

void TParam::writeTree(std::ostream &s, int indentation) const
{
  if (parent) { // don't write root node
    s << std::string(indentation, ' ');
    s << name;
    if (getType() != Invalid) {
      s << " ";
      Variant::write(s);
    }
    indentation += 2;
  }
  for (std::map<std::string, TParam*>::const_iterator it = children.begin();
      it != children.end(); it++) {
    s << "\n";
    it->second->writeTree(s, indentation);
  }
  if (indentation == 0)
    s << "\n";
}

bool TParam::parseTree(std::istream &is)
{
  if (getType() != Invalid) {
    std::cerr << "Error in " << __PRETTY_FUNCTION__ << ": Leaf Param can not be parsed as tree\n";
    return false;
  }
  name = "root";
  clear();
  std::stack<int> indentation;
  indentation.push(-1);
  return parseTree(is, indentation);
}

bool TParam::parseTree(std::istream &is, std::stack<int> &indentation)
{
  // read newline separated parameter file
  if (is.eof())
    return true;
  std::string line;
  getline(is, line);
  std::size_t pos = line.find_first_not_of(' ');
  if (pos == std::string::npos || line[pos] == '#')
    return parseTree(is, indentation);
  assert(line[pos] != '\t');
  std::istringstream linestream(line);
  std::string pName;
  linestream >> pName;
  assert(!linestream.fail());

  // find parent depending on indentation and adapt indentation
  TParam *pParent = this;
  while ((int)pos <= indentation.top()) {
    pParent = pParent->parent;
    indentation.pop();
  }

  // read parameter
  Variant pValue;
  std::string t = pValue.tryRead(linestream);
  if (pValue.getType() == Invalid) {
    if (!t.empty() && t[0] != '#')
      std::cerr << "Warning: Unknown type '" << t << "' of parameter '" << pName << "'. Parsing as category.\n";
    indentation.push(pos); // no leaf node
  }
  TParam p(pName, pValue);
  pParent->addChild(p);
  if (p.getType() == Invalid)
    return pParent->children.find(p.name)->second->parseTree(is, indentation);
  return pParent->parseTree(is, indentation);
}

void TParam::loadTree(const std::string &file) {
  std::ifstream is(file.c_str());
  if (!is.is_open()) {
    std::cerr << "Error: cannot open file '" << file << "' for loading parameters\n";
    exit(-1);
  }
  std::cerr << "Loading parameters from file '" << file << "'\n";
  parseTree(is);
}


void TParam::write(std::ostream &s) const
{
  s << name;
  if (getType() != Invalid) {
    s << " ";
    Variant::write(s);
  } else {
    s << " " << children.size();
    for (std::map<std::string, TParam*>::const_iterator it = children.begin();
        it != children.end(); it++) {
      s << " ";
      it->second->write(s);
    }
  }
}

bool TParam::read(std::istream &s)
{
  if (getType() != Invalid) {
    std::cerr << "Error in " << __PRETTY_FUNCTION__ << ": Leaf Param can not be read as tree\n";
    return false;
  }
  clear();
  s >> name;
  std::string t = Variant::tryRead(s);
  if (getType() == Invalid) {
    std::stringstream ss(t);
    unsigned int csize;
    ss >> csize;
    if (ss.fail())
      return false;
    for (unsigned int i=0; i<csize; i++) {
      TParam p;
      p.read(s);
      assert(!s.fail());
      addChild(p);
    }
  }
  return true;
}

std::string TParam::getPath(const std::string &pathAndName)
{
  size_t pos = pathAndName.find_last_of('/');
  if (pos == std::string::npos)
    return pathAndName;
  return pathAndName.substr(0, pos);
}

std::string TParam::getName(const std::string &pathAndName)
{
  size_t pos = pathAndName.find_last_of('/');
  if (pos == std::string::npos)
    return pathAndName;
  return pathAndName.substr(pos+1);
}

void TParam::handleTypeMismatch(const Type &typeTo, const std::string &typeNameTo, bool *ok) const
{
  if (ok) {
    *ok = (typeTo == type);
  }
  if (typeTo != type && !ok) {
    // produce error message and exit if there is a type mismatch
    // and no ok-flag to set
    std::cerr << "Error: Can not convert Parameter '" << getPath() << "/" << getName() << "' of type " << getTypeName() << " to type " << typeNameTo << " - Exiting.\n";
    exit(-1);
  }
}

} // namespace
