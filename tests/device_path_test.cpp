#include "device_path.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

namespace fs = std::filesystem;

void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

int main()
{
  char pattern[] = "/tmp/ultrasound-device-test-XXXXXX";
  const char * temporary = mkdtemp(pattern);
  if (!temporary) {
    return 1;
  }
  const fs::path root(temporary), dev = root / "dev", sys = root / "sys";
  const auto video = [&](const char * name, const char * port, const char * index) {
      fs::create_directories(sys / name);
      fs::create_directories(root / port);
      std::ofstream(dev / name).put('x');
      fs::create_directory_symlink(root / port, sys / name / "device");
      std::ofstream(sys / name / "index") << index;
    };
  try {
    fs::create_directories(dev / "v4l/by-id");
    fs::create_directories(dev / "v4l/by-path");
    fs::create_directories(sys);
    video("video4", "usb-port-a", "0");
    fs::create_symlink("../../video4", dev / "v4l/by-id/probe-index0");
    fs::create_symlink("../../video4", dev / "v4l/by-path/port-a-index0");
    DevicePath stable((dev / "video4").string(), dev, sys);
    const auto link = (dev / "v4l/by-id/probe-index0").string();
    require(stable.resolve() == link, "by-id must be preferred");
    fs::remove(dev / "video4");
    fs::remove_all(sys / "video4");
    fs::remove(dev / "v4l/by-id/probe-index0");
    require(stable.resolve() == link, "keep stable identity while unplugged");
    video("video4", "unrelated-camera", "0");
    video("video9", "usb-port-a", "0");
    fs::create_symlink("../../video9", dev / "v4l/by-id/probe-index0");
    require(stable.resolve() == link && fs::canonical(link) == dev / "video9",
      "reconnect same camera after videoN changes");

    fs::remove_all(dev / "v4l");
    DevicePath fallback((dev / "video9").string(), dev, sys);
    require(fallback.resolve() == (dev / "video9").string(), "learn sysfs identity");
    fs::remove(dev / "video9");
    fs::remove_all(sys / "video9");
    require(fallback.resolve().empty(), "do not open unrelated camera while unplugged");
    video("video9", "unrelated-camera", "0");
    video("video10", "usb-port-a", "1");
    require(fallback.resolve().empty(), "do not select reused number or wrong interface");
    video("video12", "usb-port-a", "0");
    require(fallback.resolve() == (dev / "video12").string(), "find renumbered sysfs interface");

    DevicePath late((dev / "video14").string(), dev, sys);
    require(late.resolve() == (dev / "video14").string(), "allow absent device at startup");
    video("video14", "usb-port-b", "0");
    late.resolve();
    require(!late.identity().empty(), "learn identity after device appears");
    fs::remove_all(root);
    std::cout << "PASS: stable links, unplug/replug, renumbering, interface identity, late arrival\n";
  } catch (const std::exception & error) {
    fs::remove_all(root);
    std::cerr << error.what() << '\n';
    return 1;
  }
}
