#pragma once
#include "cameraApiExport.h"
#include "cameraApiStatus.h"
#include "cameraDefines.h"

#include <memory>
#include <string>
#include <vector>

namespace vcamera {
class CAMERA_API_EXPORT UserSetManager {
 public:
  UserSetManager() = delete;
  ~UserSetManager();

  explicit UserSetManager(ucv::media::ICameraCapture* impl);

  UserSetManager(const UserSetManager& other);
  UserSetManager(UserSetManager&& other) noexcept;

  UserSetManager& operator=(const UserSetManager& other);
  UserSetManager& operator=(UserSetManager&& other) noexcept;

 public:
  CameraApiStatus GetAllUserSets(std::vector<UserSet>& user_sets);
  CameraApiStatus SaveToUserSet(const UserSet& user_set);
  CameraApiStatus SaveToUserSet(const std::string& user_set_name);
  CameraApiStatus SaveToUserSetWithNewName(const std::string& old_name, const std::string& new_name);
  CameraApiStatus CurrentUserSet(std::string& name);
  CameraApiStatus LoadUserSet(const std::string& name);
  CameraApiStatus GetPowerOnUserSet(std::string& name);
  CameraApiStatus SetPowerOnUserSet(const std::string& name);

 private:
  ucv::media::ICameraCapture* camera_impl_;
};

}  // namespace vcamera
