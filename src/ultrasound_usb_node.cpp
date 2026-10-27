#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdint>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <vector>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "device_path.hpp"

class UltrasoundUsbNode : public rclcpp::Node
{
public:
  UltrasoundUsbNode() : Node("ultrasound_usb")
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    device_ = declare_parameter<std::string>("device", "/dev/video4", descriptor);
    frame_id_ = declare_parameter<std::string>("frame_id", "ultrasound_optical_frame", descriptor);
    const auto topic = declare_parameter<std::string>("image_topic", "/camera/image_raw", descriptor);
    width_ = declare_parameter<int>("width", 1920, descriptor);
    height_ = declare_parameter<int>("height", 1080, descriptor);
    fps_ = declare_parameter<double>("fps", 30.0, descriptor);
    fourcc_ = declare_parameter<std::string>("fourcc", "YUYV", descriptor);
    preview_ = declare_parameter<bool>("preview", true, descriptor);
    if (preview_ && std::getenv("DISPLAY") == nullptr &&
      std::getenv("WAYLAND_DISPLAY") == nullptr)
    {
      RCLCPP_WARN(get_logger(), "No graphical display available; disabling video preview");
      preview_ = false;
    }
    mono_ = declare_parameter<bool>("monochrome", false, descriptor);
    reconnect_seconds_ = declare_parameter<double>("reconnect_interval", 2.0, descriptor);
    capture_timeout_ = declare_parameter<double>("capture_timeout", 15.0, descriptor);
    if (device_.empty() || width_ < 0 || height_ < 0 ||
      ((width_ == 0) != (height_ == 0)) ||
      !std::isfinite(fps_) || (fps_ != 0.0 && (fps_ < 0.1 || fps_ > 1000.0)) ||
      !std::isfinite(capture_timeout_) || capture_timeout_ < 1.0 || capture_timeout_ > 3600.0 ||
      !std::isfinite(reconnect_seconds_) || reconnect_seconds_ < 0.1 ||
      (!fourcc_.empty() && fourcc_.size() != 4))
    {
      throw std::invalid_argument("Invalid device, dimensions (both zero or positive), fps (0 or 0.1..1000), capture timeout (1..3600), reconnect interval (>=0.1), or fourcc (empty or 4 characters)");
    }
    device_path_ = std::make_unique<DevicePath>(device_);
    RCLCPP_INFO(get_logger(), "Video source: %s (V4L2)", device_.c_str());
    // Enable only FFmpeg: subscribing to the base topic must not send raw frames.
    const auto resolved_topic = get_node_topics_interface()->resolve_topic_name(topic);
    std::string parameter_prefix = resolved_topic.substr(
      std::string(get_effective_namespace()).size());
    if (!parameter_prefix.empty() && parameter_prefix.front() == '/') {
      parameter_prefix.erase(0, 1);
    }
    std::replace(parameter_prefix.begin(), parameter_prefix.end(), '/', '.');
    declare_parameter<std::vector<std::string>>(
      parameter_prefix + ".enable_pub_plugins", {"image_transport/ffmpeg"}, descriptor);
    publisher_ = image_transport::create_publisher(
      this, topic, rclcpp::SensorDataQoS().keep_last(1).get_rmw_qos_profile());
    RCLCPP_INFO(get_logger(), "FFmpeg transport ready on %s/ffmpeg (best effort)",
      publisher_.getTopic().c_str());
    next_open_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(
      // V4L2 read() waits for the next frame; avoid adding a second FPS limiter.
      std::chrono::milliseconds(1),
      [this]() { capture(); });
    heartbeat_.store(steady_nanoseconds());
    watchdog_ = std::thread([this]() {
      const auto timeout_ns = static_cast<int64_t>(capture_timeout_ * 1e9);
      while (!stop_watchdog_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (!stop_watchdog_.load() && steady_nanoseconds() - heartbeat_.load() > timeout_ns) {
          // Do not touch VideoCapture from another thread or unwind a blocked
          // driver/encoder. The launch supervisor starts a fresh process.
          std::fprintf(stderr,
            "ultrasound_usb: no capture progress for %.1f seconds; exiting for restart\n",
            capture_timeout_);
          std::fflush(stderr);
          std::_Exit(EXIT_FAILURE);
        }
      }
    });
  }

  ~UltrasoundUsbNode() override
  {
    stop_watchdog_.store(true);
    if (watchdog_.joinable()) {
      watchdog_.join();
    }
    if (preview_created_) {
      try {
        cv::destroyWindow("Ultrasound - preview 720p");
      } catch (const cv::Exception &) {
        // GUI resources may already have been released by the window manager.
      }
    }
  }

private:
  static int64_t steady_nanoseconds()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void show_preview(const cv::Mat & frame)
  {
    if (!preview_) {
      return;
    }
    const std::string title = "Ultrasound - preview 720p";
    try {
      if (!preview_created_) {
        cv::namedWindow(title, cv::WINDOW_AUTOSIZE);
        preview_created_ = true;
      } else if (cv::getWindowProperty(title, cv::WND_PROP_VISIBLE) < 1.0) {
        preview_ = false;
        cv::destroyWindow(title);
        preview_created_ = false;
        return;
      }
      cv::Mat display;
      const int width = std::max(1, static_cast<int>(std::lround(
        static_cast<double>(frame.cols) * 720.0 / frame.rows)));
      cv::resize(frame, display, cv::Size(width, 720), 0.0, 0.0,
        frame.rows > 720 ? cv::INTER_AREA : cv::INTER_LINEAR);
      cv::imshow(title, display);
      const int key = cv::waitKey(1) & 0xff;
      if (key == 27 || key == 'q' || key == 'Q') {
        preview_ = false;
        cv::destroyWindow(title);
        preview_created_ = false;
      }
    } catch (const cv::Exception & error) {
      RCLCPP_WARN(get_logger(), "Disabling video preview: %s", error.what());
      preview_ = false;
    }
  }

  void retry_later()
  {
    try {
      camera_.release();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Cannot release video device: %s; restarting", error.what());
      std::_Exit(EXIT_FAILURE);
    }
    next_open_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(reconnect_seconds_));
  }

  void set_property(int property, double value, const char * name)
  {
    if (!camera_.set(property, value)) {
      RCLCPP_WARN(get_logger(), "Device did not accept requested %s", name);
    }
  }

  void capture()
  {
    heartbeat_.store(steady_nanoseconds());
    try {
      if (!camera_.isOpened()) {
        if (std::chrono::steady_clock::now() < next_open_) {
          return;
        }
        const auto path = device_path_->resolve();
        const auto identity = device_path_->identity();
        if (identity != device_identity_) {
          device_identity_ = identity;
          RCLCPP_INFO(get_logger(), "Tracking video interface: %s", identity.c_str());
        }
        if (path.empty()) {
          RCLCPP_WARN(get_logger(), "Selected video interface absent; retrying in %.1f seconds",
            reconnect_seconds_);
          retry_later();
          return;
        }
        if (path != opened_path_) {
          RCLCPP_INFO(get_logger(), "Opening video source at %s", path.c_str());
          opened_path_ = path;
        }
        if (!camera_.open(path, cv::CAP_V4L2)) {
          RCLCPP_WARN(get_logger(), "Cannot open %s; retrying in %.1f seconds", path.c_str(), reconnect_seconds_);
          retry_later();
          return;
        }
        if (!fourcc_.empty()) {
          set_property(cv::CAP_PROP_FOURCC,
            cv::VideoWriter::fourcc(fourcc_[0], fourcc_[1], fourcc_[2], fourcc_[3]), "fourcc");
        }
        if (width_ > 0) {
          set_property(cv::CAP_PROP_FRAME_WIDTH, width_, "width");
          set_property(cv::CAP_PROP_FRAME_HEIGHT, height_, "height");
        }
        if (fps_ > 0.0) {
          set_property(cv::CAP_PROP_FPS, fps_, "fps");
        }
        const auto actual_width = camera_.get(cv::CAP_PROP_FRAME_WIDTH);
        const auto actual_height = camera_.get(cv::CAP_PROP_FRAME_HEIGHT);
        if (width_ > 0 && (actual_width != width_ || actual_height != height_)) {
          throw std::runtime_error("Device did not accept requested resolution; refusing to publish a different size");
        }
        set_property(cv::CAP_PROP_BUFFERSIZE, 1, "buffer size");
        RCLCPP_INFO(get_logger(), "Opened %s: %.0fx%.0f at %.2f fps; publishing on %s",
          path.c_str(), camera_.get(cv::CAP_PROP_FRAME_WIDTH),
          camera_.get(cv::CAP_PROP_FRAME_HEIGHT), camera_.get(cv::CAP_PROP_FPS),
          publisher_.getTopic().c_str());
      }
      cv::Mat frame;
      if (!camera_.read(frame) || frame.empty()) {
        RCLCPP_WARN(get_logger(), "No frame from %s; reconnecting", device_.c_str());
        retry_later();
        return;
      }
      std_msgs::msg::Header header;
      header.stamp = now();
      header.frame_id = frame_id_;
      if (frame.depth() != CV_8U && frame.depth() != CV_16U) {
        throw std::runtime_error("Expected an unsigned 8-bit or 16-bit frame");
      }
      if (mono_ && frame.channels() != 1) {
        if (frame.channels() == 3) {
          cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
        } else if (frame.channels() == 4) {
          cv::cvtColor(frame, frame, cv::COLOR_BGRA2GRAY);
        }
      }
      std::string encoding;
      const bool sixteen_bit = frame.depth() == CV_16U;
      switch (frame.channels()) {
        case 1: encoding = sixteen_bit ? "mono16" : "mono8"; break;
        case 3: encoding = sixteen_bit ? "bgr16" : "bgr8"; break;
        case 4: encoding = sixteen_bit ? "bgra16" : "bgra8"; break;
        default: throw std::runtime_error("Unsupported webcam channel count");
      }
      publisher_.publish(cv_bridge::CvImage(header, encoding, frame).toImageMsg());
      show_preview(frame);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Acquisition failed: %s", error.what());
      retry_later();
    }
  }

  std::string device_, frame_id_, fourcc_;
  std::string device_identity_, opened_path_;
  std::unique_ptr<DevicePath> device_path_;
  int width_, height_;
  double fps_, reconnect_seconds_, capture_timeout_;
  std::atomic<int64_t> heartbeat_{0};
  std::atomic<bool> stop_watchdog_{false};
  std::thread watchdog_;
  bool mono_;
  bool preview_;
  bool preview_created_{false};
  cv::VideoCapture camera_;
  std::chrono::steady_clock::time_point next_open_;
  image_transport::Publisher publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<UltrasoundUsbNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("ultrasound_usb"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
