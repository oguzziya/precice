#ifndef PRECICE_NO_MPI

#include "MPIPortsCommunicationFactory.hpp"
#include <memory>
#include <utility>

#include "MPIPortsCommunication.hpp"
#include "com/SharedPointer.hpp"

namespace precice {
namespace com {
MPIPortsCommunicationFactory::MPIPortsCommunicationFactory(std::string addressDirectory)
    : _addressDirectory(std::move(addressDirectory))
{
  if (_addressDirectory.empty()) {
    _addressDirectory = ".";
  }
}

PtrCommunication MPIPortsCommunicationFactory::newCommunication()
{
  return std::make_shared<MPIPortsCommunication>(_addressDirectory);
}

std::string MPIPortsCommunicationFactory::addressDirectory()
{
  return _addressDirectory;
}
} // namespace com
} // namespace precice

#endif // not PRECICE_NO_MPI
