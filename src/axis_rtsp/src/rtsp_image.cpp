// Copyright (c) Sensrad 2023

#include "rtsp_image.hpp"

#include <fmt/core.h>
#include <gst/gst.h>

/*********** Node implementation ***********/

// Static gstreamer C "trampoline" callbacks.
void RTSPImage::eos_(GstAppSink *appsink, gpointer user_data) {
  RTSPImage *self = static_cast<RTSPImage *>(user_data);
  self->eos(appsink);
}

GstFlowReturn RTSPImage::new_preroll_(GstAppSink *appsink, gpointer user_data) {
  RTSPImage *self = static_cast<RTSPImage *>(user_data);
  return self->new_preroll(appsink);
}

GstFlowReturn RTSPImage::new_jpeg_sample_(GstAppSink *appsink,
                                          gpointer user_data) {
  RTSPImage *self = static_cast<RTSPImage *>(user_data);
  return self->new_jpeg_sample(appsink);
}

GstFlowReturn RTSPImage::new_rgb_sample_(GstAppSink *appsink,
                                         gpointer user_data) {
  RTSPImage *self = static_cast<RTSPImage *>(user_data);
  return self->new_rgb_sample(appsink);
}

gboolean RTSPImage::new_event_(GstAppSink *appsink, gpointer user_data) {
  RTSPImage *self = static_cast<RTSPImage *>(user_data);
  return self->new_event(appsink);
}

gboolean RTSPImage::on_bus_message_(GstBus *bus, GstMessage *message,
                                    gpointer user_data) {
  RTSPImage *self = static_cast<RTSPImage *>(user_data);
  return self->on_bus_message(bus, message);
}

void RTSPImage::eos(GstAppSink *appsink) {
  RCLCPP_ERROR(get_logger(), "GStreamer: EOS %s", GST_OBJECT_NAME(appsink));
}

// Corresponding member functions
GstFlowReturn RTSPImage::new_preroll(GstAppSink *appsink) {
  RCLCPP_DEBUG(get_logger(), "New preroll");

  GstSample *sample = gst_app_sink_pull_preroll(appsink);
  if (sample) {
    gst_sample_unref(sample);
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to pull preroll");
  }

  return GST_FLOW_OK;
}

// New RGB sample
GstFlowReturn RTSPImage::new_rgb_sample(GstAppSink *appsink) {
  RCLCPP_DEBUG(get_logger(), "New RGB sample");

  GstSample *sample = gst_app_sink_pull_sample(appsink);
  if (sample) {
    // // The buffers is valid as long as sample is valid.
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    gsize size = gst_buffer_get_size(buffer);

    std::scoped_lock lock(pipeline_mutex_);

    RCLCPP_DEBUG(get_logger(), "sample buffer size: %ld", size);

    // Apply ros2 timestamp (not stamp from video)
    output_image_.header.stamp = rclcpp::Clock().now();
    output_image_.data.resize(size);
    // Copies the data from the sample buffer to the image message data.
    gst_buffer_extract(buffer, 0, output_image_.data.data(), size);
    // We are done copying.
    gst_sample_unref(sample);
    // Publish image
    image_publisher_->publish(output_image_);
    // Use same timestamp for camera info
    camera_info_.header.stamp = output_image_.header.stamp;
    camera_info_publisher->publish(camera_info_);
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to pull sample");
  }

  return GST_FLOW_OK;
}

// New JPEG sample
GstFlowReturn RTSPImage::new_jpeg_sample(GstAppSink *appsink) {
  RCLCPP_DEBUG(get_logger(), "New jpeg sample");

  GstSample *sample = gst_app_sink_pull_sample(appsink);
  if (sample) {
    // // The buffers is valid as long as sample is valid.
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    gsize size = gst_buffer_get_size(buffer);

    std::scoped_lock lock(pipeline_mutex_);

    RCLCPP_DEBUG(get_logger(), "sample buffer size: %ld", size);

    // Apply ros2 timestamp (not stamp from video)
    output_compressed_image_.header.stamp = rclcpp::Clock().now();
    output_compressed_image_.data.resize(size);
    // Copies the data from the sample buffer to the image message data.
    gst_buffer_extract(buffer, 0, output_compressed_image_.data.data(), size);
    // We are done copying.
    gst_sample_unref(sample);
    // Publish compressed image
    compressed_image_publisher_->publish(output_compressed_image_);
    // Use same timestamp for camera info
    camera_info_.header.stamp = output_compressed_image_.header.stamp;
    camera_info_publisher->publish(camera_info_);
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to pull sample");
  }

  return GST_FLOW_OK;
}

bool RTSPImage::new_event(GstAppSink *appsink) {
  GstMiniObject *miniobject = gst_app_sink_pull_object(appsink);
  GstEvent *event = GST_EVENT(miniobject);
  if (event) {
    RCLCPP_DEBUG(get_logger(), "Event type: %s", GST_EVENT_TYPE_NAME(event));
    gst_event_unref(event);
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to pull event");
  }

  return true;
}

bool RTSPImage::on_bus_message(GstBus *bus, GstMessage *message) {
  RCLCPP_DEBUG(get_logger(), "GStreamer: Got %s message",
               GST_MESSAGE_TYPE_NAME(message));

  // This function is not called on the ROS2 main thread.

  (void)bus; // unused

  switch (GST_MESSAGE_TYPE(message)) {
  case GST_MESSAGE_ERROR: {
    GError *error;
    gchar *debug;
    // Log error
    gst_message_parse_error(message, &error, &debug);
    RCLCPP_ERROR(get_logger(), "GStreamer: Error: %s", error->message);
    // Unref or we will leak memory
    g_error_free(error);
    g_free(debug);
    break;
  }
  case GST_MESSAGE_STATE_CHANGED: {
    GstState old_state, new_state;
    gst_message_parse_state_changed(message, &old_state, &new_state, nullptr);
    RCLCPP_DEBUG(
        get_logger(), "GStreamer: Element %s changed state from %s to %s",
        GST_OBJECT_NAME(message->src), gst_element_state_get_name(old_state),
        gst_element_state_get_name(new_state));
    break;
  }
  case GST_MESSAGE_EOS: {
    RCLCPP_INFO(get_logger(), "GStreamer: End-of-stream");
    break;
  }
  default:
    // Unhandled message type.
    break;
  }

  return true;
}

void RTSPImage::create_pipeline(const std::string &uri, const std::string &user,
                                const std::string &password) {

  std::scoped_lock lock(pipeline_mutex_);

  GError *error = nullptr;
  // Amount of ms to buffer
  const int latency = 20;
  std::string pipeline_desc;

  if (output_format_ == "jpeg") {

    // Now we can construct the URI.
    pipeline_desc = fmt::format(
        "rtspsrc user-id={:s} user-pw={:s} location={:s} latency={:d} ! "
        "queue ! decodebin3 ! videoconvert ! "
        "video/x-raw,width={:d},height={:d} ! "
        "jpegenc !"
        "appsink name=appsink",
        user, password, uri, latency, width_, height_);

  } else if (output_format_ == "rgb") {

    // Create pipeline string with raw rgb output
    pipeline_desc = fmt::format(
        "rtspsrc user-id={:s} user-pw={:s} location={:s} latency={:d} ! "
        "queue ! decodebin3 ! videoconvert ! "
        "video/x-raw,width={:d},height={:d},format=RGB ! "
        "appsink name=appsink",
        user, password, uri, latency, width_, height_);

  } else {
    // Should not happen
    const auto msg =
        fmt::format("Unsupported output format: {:s}", output_format_);

    throw std::runtime_error(msg);
  }

  // Create pipeline
  // pipeline_ = gst_parse_launch(pipeline_string, &error);
  pipeline_ = gst_parse_launch(pipeline_desc.c_str(), &error);
  if (error) {
    throw std::runtime_error(error->message);
  }

  // Get the message bus of the pipeline
  auto bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));
  // Add a message watch to the main loop
  gst_bus_add_watch(bus, on_bus_message_, this);
  gst_object_unref(bus);

  // Get sink by name
  sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "appsink");
  if (not sink_) {
    throw std::runtime_error("Could not get sink");
  }

  // Set sink callbacks, we use these to publish the output
  callbacks_.eos = RTSPImage::eos_;
  callbacks_.new_preroll = RTSPImage::new_preroll_;
  callbacks_.new_event = RTSPImage::new_event_;
  // Set sample callback based on output format
  if (output_format_ == "jpeg") {
    callbacks_.new_sample = RTSPImage::new_jpeg_sample_;
  } else if (output_format_ == "rgb") {
    callbacks_.new_sample = RTSPImage::new_rgb_sample_;
  } else {
    throw std::runtime_error("Invalid output format");
  }

  // Set callbacks
  gst_app_sink_set_callbacks(GST_APP_SINK(sink_), &callbacks_, this,
                             nullptr); // destroy notify.
}

RTSPImage::RTSPImage() : Node("rtsp_image"), width_(1280), height_(720) {

  // Create a glib main loop. Gstreamer needs this for some callbacks.
  main_loop_ = g_main_loop_new(NULL, FALSE);

  // Run this loop on a separate thread.
  // Calls the function g_main_loop_run with the main_loop_ as argument.
  main_loop_thread_ = std::thread(g_main_loop_run, main_loop_);

  // This is the default IP of all Axis cameras.
  const auto ip = declare_parameter("ip", "192.168.0.90");
  // Placeholder user and passwordk
  const auto user = declare_parameter("user", "sensrad");
  const auto password = declare_parameter("password", "sensrad");

  // Get width and height
  width_ = declare_parameter("width", width_);
  height_ = declare_parameter("height", height_);

  // Now we can construct the URI.
  std::string uri = fmt::format(
      "rtsp://{:s}/axis-media/media.amp?camera=1&resolution={:d}x{:d}", ip,
      width_, height_);

  // Default frame id
  const auto frame_id = declare_parameter("frame_id", "camera_1");

  // Get string parameter format and convert to lower case
  output_format_ = declare_parameter("format", "jpeg");
  std::transform(output_format_.begin(), output_format_.end(),
                 output_format_.begin(), ::tolower);

  // Camera info
  // Distortion model name
  const auto distortion_model =
      declare_parameter("distortion_model", "plumb_bob");
  // Distortion parameters
  const auto camera_distortion =
      declare_parameter("distortion", std::vector<double>{});
  // Intrinsic camera matrix (3x3 row major)
  const auto camera_intrinsic =
      declare_parameter("intrinsic", std::vector<double>{});
  // Rectification matrix (3x3 row major)
  const auto camera_rectification =
      declare_parameter("rectification", std::vector<double>{});
  // Projection/camera matrix (3x4 row major)
  const auto camera_projection =
      declare_parameter("projection", std::vector<double>{});

  camera_info_ = sensor_msgs::msg::CameraInfo();
  camera_info_.header.frame_id = frame_id;
  camera_info_.width = width_;
  camera_info_.height = height_;
  camera_info_.distortion_model = distortion_model;
  camera_info_.d = camera_distortion;

  // Camera intrinsic matrix
  if (camera_intrinsic.size() != 9) {
    throw std::runtime_error("Camera matrix must have 9 elements");
  }
  camera_info_.k = {
      camera_intrinsic[0], camera_intrinsic[1], camera_intrinsic[2],
      camera_intrinsic[3], camera_intrinsic[4], camera_intrinsic[5],
      camera_intrinsic[6], camera_intrinsic[7], camera_intrinsic[8]};
  // Camera rectification matrix (only for stereo cameras)
  if (camera_rectification.size() != 9) {
    throw std::runtime_error("Rectification matrix must have 9 elements");
  }
  camera_info_.r = {camera_rectification[0], camera_rectification[1],
                    camera_rectification[2], camera_rectification[3],
                    camera_rectification[4], camera_rectification[5],
                    camera_rectification[6], camera_rectification[7],
                    camera_rectification[8]};
  // Camera projection matrix
  if (camera_projection.size() != 12) {
    throw std::runtime_error("Projection matrix must have 12 elements");
  }
  camera_info_.p = {
      camera_projection[0], camera_projection[1],  camera_projection[2],
      camera_projection[3], camera_projection[4],  camera_projection[5],
      camera_projection[6], camera_projection[7],  camera_projection[8],
      camera_projection[9], camera_projection[10], camera_projection[11]};

  // Same for all formats.
  camera_info_publisher =
      create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos_);

  if (output_format_ == "jpeg") {
    // JPEG image publisher
    compressed_image_publisher_ =
        create_publisher<sensor_msgs::msg::CompressedImage>("compressed", qos_);

    // Output compressed image message
    output_compressed_image_ = sensor_msgs::msg::CompressedImage();
    output_compressed_image_.header.frame_id = frame_id;
    // Format is jpeg
    output_compressed_image_.format = "jpeg";
  } else if (output_format_ == "rgb") {
    // Raw image publisher
    image_publisher_ = create_publisher<sensor_msgs::msg::Image>("image", qos_);

    // Output image message
    output_image_ = sensor_msgs::msg::Image();
    output_image_.header.frame_id = frame_id;
    output_image_.width = width_;
    output_image_.height = height_;
    output_image_.step = width_ * 3;
    output_image_.data.resize(output_image_.step * height_);
    // Format is RGB8
    output_image_.encoding = "rgb8";
  } else {
    const auto msg =
        fmt::format("Unsupported output format: {:s}", output_format_);

    throw std::runtime_error(msg);
  }

  // Create gstreamer pipeline
  create_pipeline(uri, user, password);
  // Start pipeline
  gst_element_set_state(pipeline_, GST_STATE_PLAYING);

  RCLCPP_INFO(get_logger(), "RTSPImage node started");
}

RTSPImage::~RTSPImage() {
  // Stop the media processing pipeline
  gst_element_set_state(pipeline_, GST_STATE_NULL);
  // gst_object_unref(source_);
  gst_object_unref(sink_);
  gst_object_unref(pipeline_);

  // Signal that we need to stop the main loop
  g_main_loop_quit(main_loop_);
  // Wait for the main loop thread to finish
  main_loop_thread_.join();
}
