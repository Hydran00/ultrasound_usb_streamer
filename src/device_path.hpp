#pragma once

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

// Keep the selected V4L2 interface across USB disconnects and videoN renumbering.
// Never fall back to a possibly reassigned videoN after learning its identity.
class DevicePath
{
public:
  explicit DevicePath(
    std::string configured,
    std::filesystem::path dev_root = "/dev",
    std::filesystem::path sys_root = "/sys/class/video4linux")
  : configured_(std::move(configured)), dev_root_(std::move(dev_root)),
    sys_root_(std::move(sys_root)) {}

  std::string resolve()
  {
    namespace fs = std::filesystem;
    if (!stable_link_.empty()) {
      return stable_link_;
    }
    if (!interface_.empty()) {
      for (const auto & entry : entries(sys_root_)) {
        if (canonical(entry / "device") == interface_ && read_index(entry) == index_) {
          return (dev_root_ / entry.filename()).string();
        }
      }
      return {};  // The selected interface is still absent.
    }
    const auto target = canonical(configured_);
    if (target.empty()) {
      return configured_;  // May be absent at startup; learn identity when it returns.
    }
    for (const auto & directory : {dev_root_ / "v4l/by-id", dev_root_ / "v4l/by-path"}) {
      for (const auto & link : entries(directory)) {
        if (canonical(link) == target) {
          stable_link_ = link.string();
          return stable_link_;
        }
      }
    }
    const auto sys_entry = sys_root_ / target.filename();
    const auto interface = canonical(sys_entry / "device");
    const auto index = read_index(sys_entry);
    if (!interface.empty() && !index.empty()) {
      interface_ = interface;
      index_ = index;
    }
    return configured_;
  }

  std::string identity() const
  {
    if (!stable_link_.empty()) {
      return stable_link_;
    }
    return interface_.empty() ? std::string{} : interface_.string() + " index=" + index_;
  }

private:
  static std::filesystem::path canonical(const std::filesystem::path & path)
  {
    std::error_code error;
    auto result = std::filesystem::canonical(path, error);
    return error ? std::filesystem::path{} : result;
  }

  static std::vector<std::filesystem::path> entries(const std::filesystem::path & directory)
  {
    std::vector<std::filesystem::path> result;
    std::error_code error;
    std::filesystem::directory_iterator it(directory, error), end;
    while (!error && it != end) {
      result.push_back(it->path());
      it.increment(error);
    }
    std::sort(result.begin(), result.end());
    return result;
  }

  static std::string read_index(const std::filesystem::path & entry)
  {
    std::ifstream file(entry / "index");
    std::string index;
    file >> index;
    return index;
  }

  std::string configured_, stable_link_, index_;
  std::filesystem::path dev_root_, sys_root_, interface_;
};
