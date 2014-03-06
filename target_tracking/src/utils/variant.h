/***************************************************************
 * This source code is part of ROBULAR.
 *
 * ROBULAR Copyright (c) 2009 Joerg Mueller, Boris Lau.
 * Autonomous Intelligent Systems, Department of Computer
 * Science, University of Freiburg, Germany.
 **************************************************************/

#ifndef ROBULAR_VARIANT
#define ROBULAR_VARIANT

#include <string>
#include <vector>
#include <iostream>
#include <Eigen/Core>

namespace ranav {

//! Data container which can contain different types of data.
//! Once the type is defined (not invalid), it can no longer be changed.
class Variant {
public:
  enum Type {
    Invalid = 0,
    Int = 1,
    Double = 2,
    String = 3,
    IntVector = 11,
    DoubleVector = 12,
    StringVector = 13
  };

  Variant();
  Variant(int v);
  Variant(double v);
  Variant(const std::string &v);
  Variant(const std::vector<int> &v);
  Variant(const std::vector<double> &v);
  Variant(const std::vector<std::string> &v);

  Variant(const Variant &other);
  Variant& operator=(const Variant &other);

  virtual ~Variant();

  virtual Type getType() const;
  virtual std::string getTypeName() const;
  virtual bool isValid() const;

  virtual int toInt(bool *ok = NULL) const;
  virtual double toDouble(bool *ok = NULL) const;
  virtual std::string toString(bool *ok = NULL) const;
  virtual std::vector<int> toIntVector(bool *ok = NULL) const;
  virtual std::vector<double> toDoubleVector(bool *ok = NULL) const;
  virtual std::vector<std::string> toStringVector(bool *ok = NULL) const;

  // Eigen conversions
  virtual Eigen::VectorXd toVectorXd(bool *ok = NULL) const;

  //! writes type and value to stream. Don't try to write and invalid Variant!
  virtual void write(std::ostream &s) const;
  virtual bool read(std::istream &s);
  //! tries to read from stream. if the reading fails, it returns the read typename.
  virtual std::string tryRead(std::istream &s);

protected:
  void deleteData();
  virtual void handleTypeMismatch(const Type &typeTo, const std::string &typeNameTo, bool *ok) const;
  static Type getTypeByName(const std::string &typeName);
  Type type;
  void *data;
};

} // namespace

#endif
