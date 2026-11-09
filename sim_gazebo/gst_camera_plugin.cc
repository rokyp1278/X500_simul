// Minimal Gazebo Classic camera -> GStreamer H264 UDP streamer.
// Attaches to a <sensor type="camera"> and streams frames the drone "sees"
// over UDP so Python (OpenCV, cv2.CAP_GSTREAMER) receives them exactly as it
// would from a real Jetson camera link. Pipeline:
//   appsrc ! videoconvert ! x264enc ! rtph264pay ! udpsink
#include <gazebo/common/Plugin.hh>
#include <gazebo/plugins/CameraPlugin.hh>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <cstring>
#include <string>
#include <mutex>

namespace gazebo {
class GstCameraPlugin : public CameraPlugin {
 public:
  GstCameraPlugin() : CameraPlugin() {}
  ~GstCameraPlugin() override {
    if (pipeline) {
      gst_element_set_state(pipeline, GST_STATE_NULL);
      gst_object_unref(pipeline);
    }
  }

  void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override {
    CameraPlugin::Load(_sensor, _sdf);
    udpHost = _sdf->HasElement("udpHost") ? _sdf->Get<std::string>("udpHost")
                                          : std::string("127.0.0.1");
    udpPort = _sdf->HasElement("udpPort") ? _sdf->Get<int>("udpPort") : 5600;
    bitrate = _sdf->HasElement("bitrate") ? _sdf->Get<int>("bitrate") : 1000;
    gzmsg << "[gst_camera] " << this->width << "x" << this->height
          << " -> udp://" << udpHost << ":" << udpPort << std::endl;
  }

  void InitPipeline() {
    if (!gst_is_initialized()) gst_init(nullptr, nullptr);
    std::string desc =
      "appsrc name=src is-live=true do-timestamp=true format=time ! "
      "videoconvert ! video/x-raw,format=I420 ! "
      "x264enc tune=zerolatency bitrate=" + std::to_string(bitrate) +
      " speed-preset=ultrafast key-int-max=10 ! "
      "rtph264pay config-interval=1 pt=96 ! "
      "udpsink host=" + udpHost + " port=" + std::to_string(udpPort) +
      " sync=false async=false";
    GError *err = nullptr;
    pipeline = gst_parse_launch(desc.c_str(), &err);
    if (!pipeline) {
      gzerr << "[gst_camera] pipeline failed: "
            << (err ? err->message : "unknown") << std::endl;
      return;
    }
    appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "src");
    std::string caps = "video/x-raw,format=RGB,width=" +
                       std::to_string(this->width) + ",height=" +
                       std::to_string(this->height) + ",framerate=30/1";
    GstCaps *c = gst_caps_from_string(caps.c_str());
    gst_app_src_set_caps(GST_APP_SRC(appsrc), c);
    gst_caps_unref(c);
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    initialised = true;
    gzmsg << "[gst_camera] pipeline PLAYING" << std::endl;
  }

  void OnNewFrame(const unsigned char *_image, unsigned int _w,
                  unsigned int _h, unsigned int /*_d*/,
                  const std::string & /*_fmt*/) override {
    std::lock_guard<std::mutex> lk(mtx);
    if (!initialised) InitPipeline();
    if (!initialised) return;
    gsize size = static_cast<gsize>(_w) * _h * 3;
    GstBuffer *buf = gst_buffer_new_allocate(nullptr, size, nullptr);
    GstMapInfo map;
    gst_buffer_map(buf, &map, GST_MAP_WRITE);
    std::memcpy(map.data, _image, size);
    gst_buffer_unmap(buf, &map);
    GstFlowReturn ret;
    g_signal_emit_by_name(appsrc, "push-buffer", buf, &ret);
    gst_buffer_unref(buf);
  }

 private:
  bool initialised{false};
  std::string udpHost{"127.0.0.1"};
  int udpPort{5600};
  int bitrate{1000};
  GstElement *pipeline{nullptr};
  GstElement *appsrc{nullptr};
  std::mutex mtx;
};
GZ_REGISTER_SENSOR_PLUGIN(GstCameraPlugin)
}  // namespace gazebo
