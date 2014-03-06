/***************************************************************
 * This source code is part of ROBULAR.
 *
 * ROBULAR Copyright (c) 2009 Joerg Mueller, Boris Lau.
 * Autonomous Intelligent Systems, Department of Computer
 * Science, University of Freiburg, Germany.
 **************************************************************/

#include "variant.h"
#include <cassert>
#include <cstdlib>

namespace ranav {

Variant::Variant() :
        type(Invalid),
        data(NULL)
{ }

Variant::Variant(int v) :
        type(Int),
        data(new int(v))
{ }

Variant::Variant(double v) :
        type(Double),
        data(new double(v))
{ }

Variant::Variant(const std::string &v) :
        type(String),
        data(new std::string(v))
{ }

Variant::Variant(const std::vector<int> &v) :
        type(IntVector),
        data(new std::vector<int>(v))
{ }

Variant::Variant(const std::vector<double> &v) :
        type(DoubleVector),
        data(new std::vector<double>(v))
{ }

Variant::Variant(const std::vector<std::string> &v) :
        type(StringVector),
        data(new std::vector<std::string>(v))
{ }

Variant::Variant(const Variant &other) :
        type(Invalid),
        data(NULL)
{
  operator=(other);
}

Variant& Variant::operator=(const Variant &other)
{
  if (type == Invalid)
    type = other.type;
  assert(type == other.type);
  deleteData();
  // copy data
  switch (type) {
    case Int: data = new int(*(int*)other.data); break;
    case Double: data = new double(*(double*)other.data); break;
    case String: data = new std::string(*(std::string*)other.data); break;
    case IntVector: data = new std::vector<int>(*(std::vector<int>*)other.data); break;
    case DoubleVector: data = new std::vector<double>(*(std::vector<double>*)other.data); break;
    case StringVector: data = new std::vector<std::string>(*(std::vector<std::string>*)other.data); break;
    default: break;
  }
  return *this;
}

Variant::~Variant()
{
  deleteData();
}

void Variant::deleteData()
{
  switch (type) {
    case Int: delete (int*)data; break;
    case Double: delete (double*)data; break;
    case String: delete (std::string*)data; break;
    case IntVector: delete (std::vector<int>*)data; break;
    case DoubleVector: delete (std::vector<double>*)data; break;
    case StringVector: delete (std::vector<std::string>*)data; break;
    default: break;
  }
}

Variant::Type Variant::getType() const
{
  return type;
}

std::string Variant::getTypeName() const
{
  switch (type) {
    case Invalid: return "Invalid"; break;
    case Int: return "Int"; break;
    case Double: return "Double"; break;
    case String: return "String"; break;
    case IntVector: return "IntVector"; break;
    case DoubleVector: return "DoubleVector"; break;
    case StringVector: return "StringVector"; break;
    default: assert(false); break;
  }
  return "";
}

bool Variant::isValid() const
{
  return (type != Invalid);
}

void Variant::handleTypeMismatch(const Type &typeTo, const std::string &typeNameTo, bool *ok) const
{
  if (ok) {
    *ok = (typeTo == type);
  }
  if (typeTo != type && !ok) {
    // produce error message and exit if there is a type mismatch
    // and no ok-flag to set
    std::cerr << "Error: Can not convert Variant of type " << getTypeName() << " to type " << typeNameTo << " - Exiting.\n";
    exit(-1);
  }
}

int Variant::toInt(bool *ok) const
{
  handleTypeMismatch(Int, "Int", ok);
  if ( (ok && (*ok)) || !ok ) return *((int*)data);
  else                        return 0;
}

double Variant::toDouble(bool *ok) const
{
  handleTypeMismatch(Double, "Double", ok);
  if ( (ok && (*ok)) || !ok ) return *((double*)data);
  else                        return 0.0;
}

std::string Variant::toString(bool *ok) const
{
  handleTypeMismatch(String, "String", ok);
  if ( (ok && (*ok)) || !ok ) return *((std::string*)data);
  else                        return "";
}

std::vector<int> Variant::toIntVector(bool *ok) const
{
  handleTypeMismatch(IntVector, "IntVector", ok);
  if ( (ok && (*ok)) || !ok ) return *((std::vector<int>*)data);
  else                        return std::vector<int>();
}

std::vector<double> Variant::toDoubleVector(bool *ok) const
{
  handleTypeMismatch(DoubleVector, "DoubleVector", ok);
  if ( (ok && (*ok)) || !ok ) return *((std::vector<double>*)data);
  else                        return std::vector<double>();
}

std::vector<std::string> Variant::toStringVector(bool *ok) const
{
  handleTypeMismatch(StringVector, "StringVector", ok);
  if ( (ok && (*ok)) || !ok ) return *((std::vector<std::string>*)data);
  else                        return std::vector<std::string>();
}

Eigen::VectorXd Variant::toVectorXd(bool *ok) const {
  std::vector<double> v = toDoubleVector(ok);
//  return Eigen::VectorXd::Map(v.data(), v.size());
  Eigen::VectorXd vec(v.size());
  for (unsigned int i=0; i<v.size(); ++i) vec(i) = v[i];
  return vec;
}


void Variant::write(std::ostream &s) const
{
  assert(type != Invalid);
  s << getTypeName();
  switch (type) {
    case Int: s << " " << (*(int*)data); break;
    case Double: s << " " << (*(double*)data); break;
    case String: s << " " << (*(std::string*)data); break;
    case IntVector:
      s << " " << ((std::vector<int>*)data)->size();
      for (std::vector<int>::const_iterator it = ((std::vector<int>*)data)->begin();
          it != ((std::vector<int>*)data)->end(); it++)
        s << " " << *it;
      break;
    case DoubleVector:
      s << " " << ((std::vector<double>*)data)->size();
      for (std::vector<double>::const_iterator it = ((std::vector<double>*)data)->begin();
          it != ((std::vector<double>*)data)->end(); it++)
        s << " " << *it;
      break;
    case StringVector:
      s << " " << ((std::vector<std::string>*)data)->size();
      for (std::vector<std::string>::const_iterator it = ((std::vector<std::string>*)data)->begin();
          it != ((std::vector<std::string>*)data)->end(); it++)
        s << " " << *it;
      break;
    case Invalid: assert(false); break;
  }
}

bool Variant::read(std::istream &s)
{
  tryRead(s);
  return (type != Invalid);
}

std::string Variant::tryRead(std::istream &s)
{
  std::string typeName;
  s >> typeName;
  Type t = getTypeByName(typeName);
  if (t == Invalid)
    return typeName;
  if (type == Invalid) {
    type = t;
    switch (type) {
      case Int: data = new int; break;
      case Double: data = new double; break;
      case String: data = new std::string; break;
      case IntVector: data = new std::vector<int>; break;
      case DoubleVector: data = new std::vector<double>; break;
      case StringVector: data = new std::vector<std::string>; break;
      case Invalid: assert(false); break;
    }
  }
  if (t != type) {
    std::cerr << "Error in " << __PRETTY_FUNCTION__ << ": Type mismatch\n";
    return typeName;
  }

  unsigned int size;
  switch (type) {
    case Int: s >> (*(int*)data); break;
    case Double: s >> (*(double*)data); break;
    case String: s >> (*(std::string*)data); break;
    case IntVector:
      s >> size;
      ((std::vector<int>*)data)->resize(size);
      for (std::vector<int>::iterator it = ((std::vector<int>*)data)->begin();
          it != ((std::vector<int>*)data)->end(); it++)
        s >> *it;
      break;
    case DoubleVector:
      s >> size;
      ((std::vector<double>*)data)->resize(size);
      for (std::vector<double>::iterator it = ((std::vector<double>*)data)->begin();
          it != ((std::vector<double>*)data)->end(); it++)
        s >> *it;
      break;
    case StringVector:
      s >> size;
      ((std::vector<std::string>*)data)->resize(size);
      for (std::vector<std::string>::iterator it = ((std::vector<std::string>*)data)->begin();
          it != ((std::vector<std::string>*)data)->end(); it++)
        s >> *it;
      break;
    case Invalid: assert(false); break;
  }

  return std::string();
}

Variant::Type Variant::getTypeByName(const std::string &typeName)
{
  if (typeName == "Int")
    return Int;
  if (typeName == "Double")
    return Double;
  if (typeName == "String")
    return String;
  if (typeName == "IntVector")
    return IntVector;
  if (typeName == "DoubleVector")
    return DoubleVector;
  if (typeName == "StringVector")
    return StringVector;

  return Invalid;
}

} // namespace
