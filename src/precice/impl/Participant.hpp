#pragma once

#include <Eigen/Core>
#include <memory>
#include <stddef.h>
#include <string>
#include <utility>
#include <vector>
#include "SharedPointer.hpp"
#include "action/SharedPointer.hpp"
#include "cplscheme/SharedPointer.hpp"
#include "io/ExportContext.hpp"
#include "io/config/ExportConfiguration.hpp"
#include "logging/Logger.hpp"
#include "mapping/SharedPointer.hpp"
#include "mesh/SharedPointer.hpp"
#include "partition/ReceivedPartition.hpp"
#include "utils/ManageUniqueIDs.hpp"
#include "utils/MasterSlave.hpp"
#include "utils/PointerVector.hpp"

namespace precice {
namespace impl {
struct DataContext;
struct MeshContext;
struct MappingContext;
} // namespace impl
} // namespace precice

// Forward declaration to friend the boost test struct
namespace PreciceTests {
namespace Serial {
struct TestConfigurationPeano;
struct TestConfigurationComsol;
} // namespace Serial
} // namespace PreciceTests

namespace precice {
namespace utils {
class ManageUniqueIDs;
} // namespace utils

namespace impl {

/// Holds coupling state of one participating solver in coupled simulation.
class Participant {
public:
  enum MappingConstants {
    MAPPING_LINEAR_CONSERVATIVE,
    MAPPING_LINEAR_CONSISTENT,
    MAPPING_DIRECT
  };

  /**
   * @brief Constructor.
   *
   * @param[in] name Name of the participant. Has to be unique.
   */
  Participant(
      std::string                 name,
      mesh::PtrMeshConfiguration &meshConfig);

  virtual ~Participant();

  /// @name Configuration interface
  /// @{
  /// Adds a configured write \ref Data to the Participant
  void addWriteData(
      const mesh::PtrData &data,
      const mesh::PtrMesh &mesh);

  /// Adds a configured read \ref Data to the Participant
  void addReadData(
      const mesh::PtrData &data,
      const mesh::PtrMesh &mesh);

  /// Adds a configured read \ref Mapping to the Participant
  void addReadMappingContext(MappingContext *mappingContext);

  /// Adds a configured write \ref Mapping to the Participant
  void addWriteMappingContext(MappingContext *mappingContext);

  /// Adds a configured \ref WatchPoint to the Participant
  void addWatchPoint(const PtrWatchPoint &watchPoint);

  /// Adds a configured \ref WatchIntegral to the Participant
  void addWatchIntegral(const PtrWatchIntegral &watchIntegral);

  /// Sets weather the participant was configured with a master tag
  void setUseMaster(bool useMaster);

  /// Sets the manager responsible for providing unique IDs to meshes.
  void setMeshIdManager(std::unique_ptr<utils::ManageUniqueIDs> &&idm)
  {
    _meshIdManager = std::move(idm);
  }

  /// Adds a configured \ref Action to the participant
  void addAction(action::PtrAction &&action);

  /// Adds a configured \ref ExportContext to export meshes and data.
  void addExportContext(const io::ExportContext &context);

  /// Adds a mesh to be used by the participant.
  void useMesh(
      const mesh::PtrMesh &                         mesh,
      const Eigen::VectorXd &                       localOffset,
      bool                                          remote,
      const std::string &                           fromParticipant,
      double                                        safetyFactor,
      bool                                          provideMesh,
      partition::ReceivedPartition::GeometricFilter geoFilter);
  /// @}

  /// @name Data queries
  /// @{
  /** Provides access to both write and read \ref DataContext
   * @pre there exists a \ref DataContext for \ref dataID
   */
  const DataContext &dataContext(int dataID) const;

  /** Provides access to both write and read \ref DataContext
   * @pre there exists a \ref DataContext for \ref dataID
   */
  DataContext &dataContext(int dataID);

  /** Provides access to write \ref DataContext
   * @remarks does not contain nullptr.
   */
  const utils::ptr_vector<DataContext> &writeDataContexts() const;

  /** Provides access to write \ref DataContext
   * @remarks does not contain nullptr.
   */
  utils::ptr_vector<DataContext> &writeDataContexts();

  /** Provides access to read \ref DataContext
   * @remarks does not contain nullptr.
   */
  const utils::ptr_vector<DataContext> &readDataContexts() const;

  /** Provides access to read \ref DataContext
   * @remarks does not contain nullptr.
   */
  utils::ptr_vector<DataContext> &readDataContexts();

  /// Is the dataID know to preCICE?
  bool hasData(int dataID) const;

  /// Is the data used by this participant?
  bool isDataUsed(int dataID) const;

  /// Is the data used by this participant?
  bool isDataUsed(const std::string &dataName, int meshId) const;

  /// Is the participant allowed to read the data?
  bool isDataRead(int dataID) const;

  /// Is the participant allowed to write the data?
  bool isDataWrite(int dataID) const;

  /// What is the dataID of the used data from a used mesh given the meshid and the data name?
  int getUsedDataID(const std::string &dataName, int meshID) const;

  /// What is the name of the given data id
  std::string getDataName(int dataID) const;
  /// @}

  /// @name Mesh queries
  /// @{
  /*** Provides direct access to a \ref MeshContext given the \ref meshid
   * @param[in] meshID the id of the \ref Mesh
   * @returns a reference to the matching \ref MeshContext
   * @pre the \ref Mesh with \ref meshID is used by the Participant
   */
  const MeshContext &meshContext(int meshID) const;

  /*** Provides direct access to a \ref MeshContext given the \ref meshid
   * @param[in] meshID the id of the \ref Mesh
   * @returns a reference to the matching \ref MeshContext
   * @pre the \ref Mesh with \ref meshID is used by the Participant
   */
  MeshContext &meshContext(int meshID);

  /** Provides unordered access to all \ref MeshContext.used by this \ref Participant
   * @remarks The sequence does not contain nullptr
   */
  const std::vector<MeshContext *> &usedMeshContexts() const;

  /** Provides unordered access to all \ref MeshContext.used by this \ref Participant
   * @remarks The sequence does not contain nullptr
   */
  std::vector<MeshContext *> &usedMeshContexts();

  /** Looks for a used MeshContext with a given mesh name.
   * @param[in] name the name of the \ref Mesh
   * @return a reference to the MeshContext
   * @pre there is a matching mesh
   */
  MeshContext &usedMeshContext(const std::string &name);

  /** Looks for a used MeshContext with a given mesh name.
   * @param[in] name the name of the \ref Mesh
   * @return a reference to the MeshContext
   * @pre there is a matching mesh
   */
  MeshContext const &usedMeshContext(const std::string &name) const;

  /** Looks for a used MeshContext with a given mesh ID.
   * @param[in] meshID the id of the \ref Mesh
   * @return a reference to the MeshContext
   * @pre there is a matching mesh
   */
  MeshContext &usedMeshContext(int meshID);

  /** Looks for a used MeshContext with a given meshID
   * @param[in] meshID the id of the \ref Mesh
   * @return a reference to the MeshContext
   * @pre there is a matching mesh
   */
  MeshContext const &usedMeshContext(int meshID) const;

  /// Does preCICE know a mesh with this meshID?
  bool hasMesh(int meshID) const;

  /// Does preCICE know a mesh with this name?
  bool hasMesh(const std::string &meshName) const;

  /// Is a mesh with this id used by this participant?
  bool isMeshUsed(int meshID) const;

  /// Is a mesh with this name used by this participant?
  bool isMeshUsed(const std::string &meshID) const;

  /// Is a mesh with this id provided?
  bool isMeshProvided(int meshID) const;

  /// Get the used mesh id of a mesh with this name.
  int getUsedMeshID(const std::string &meshName) const;

  /// Get the name of a mesh given by its id.
  std::string getMeshName(int meshID) const;

  /// Get a mesh name which uses the given data id.
  std::string getMeshNameFromData(int dataID) const;
  /// @}

  /// @name Other queries
  /// @{
  /// Returns the name of the participant.
  const std::string &getName() const;

  /// Returns true, if the participant uses a master tag.
  bool useMaster() const;

  /// Provided access to all read \ref MappingContext
  const utils::ptr_vector<MappingContext> &readMappingContexts() const;

  /// Provided access to all write \ref MappingContext
  const utils::ptr_vector<MappingContext> &writeMappingContexts() const;

  /// Provided access to all \ref WatchPoints
  std::vector<PtrWatchPoint> &watchPoints();

  /// Provided access to all \ref WatchIntegrals
  std::vector<PtrWatchIntegral> &watchIntegrals();

  /// Provided access to all \ref Action
  std::vector<action::PtrAction> &actions();

  /// Provided access to all \ref Action
  const std::vector<action::PtrAction> &actions() const;

  /// Returns all \ref ExportContext for exporting meshes and data.
  const std::vector<io::ExportContext> &exportContexts() const;
  /// @}

private:
  mutable logging::Logger _log{"impl::Participant"};

  std::string _name;

  std::vector<PtrWatchPoint> _watchPoints;

  std::vector<PtrWatchIntegral> _watchIntegrals;

  /// Export contexts to export meshes, data, and more.
  std::vector<io::ExportContext> _exportContexts;

  std::vector<action::PtrAction> _actions;

  /// All mesh contexts involved in a simulation, mesh ID == index.
  std::vector<MeshContext *> _meshContexts;

  /// Read mapping contexts used by the participant.
  utils::ptr_vector<MappingContext> _readMappingContexts;

  /// Write mapping contexts used by the participant.
  utils::ptr_vector<MappingContext> _writeMappingContexts;

  /// Mesh contexts used by the participant.
  std::vector<MeshContext *> _usedMeshContexts;

  std::vector<DataContext *> _dataContexts;

  utils::ptr_vector<DataContext> _writeDataContexts;

  utils::ptr_vector<DataContext> _readDataContexts;

  //io::ExportContext _exportContext;

  bool _useMaster = false;

  std::unique_ptr<utils::ManageUniqueIDs> _meshIdManager;

  template <typename ELEMENT_T>
  bool isDataValid(
      const std::vector<ELEMENT_T> &data,
      const ELEMENT_T &             newElement) const;

  void checkDuplicatedUse(const mesh::PtrMesh &mesh);

  void checkDuplicatedData(const mesh::PtrData &data);

  /// To allow white box tests.
  friend struct PreciceTests::Serial::TestConfigurationPeano;
  friend struct PreciceTests::Serial::TestConfigurationComsol;
};

// --------------------------------------------------------- HEADER DEFINITIONS

template <typename ELEMENT_T>
bool Participant::isDataValid(
    const std::vector<ELEMENT_T> &data,
    const ELEMENT_T &             newElement) const
{
  for (size_t i = 0; i < data.size(); i++) {
    if (data[i].name == newElement.name) {
      return false;
    }
  }
  return true;
}

} // namespace impl
} // namespace precice
