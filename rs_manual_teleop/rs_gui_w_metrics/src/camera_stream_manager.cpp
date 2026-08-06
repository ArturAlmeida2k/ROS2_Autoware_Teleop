#include "camera_stream_manager.hpp"
#include <QDebug>
#include <chrono>
#include <cstring>
#include <algorithm>
#include <string>

CameraStreamManager::CameraStreamManager(QQuickItem *videoItem, int port, QObject *parent)
    : QObject(parent), video_item_(videoItem)
{
    gst_init(nullptr, nullptr);

    std::string decoder_str;
    GstElementFactory *nv_factory = gst_element_factory_find("nvh264dec");
    if (nv_factory) {
        decoder_str = "nvh264dec max-display-delay=0 ! ";
        gst_object_unref(nv_factory);
    } else {
        decoder_str = "avdec_h264 ! ";
    }

    std::string pipeline_str =
        "udpsrc port=" + std::to_string(port) + " "
        "caps=\"application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96\" ! "
        "rtpjitterbuffer latency=0 drop-on-latency=true ! "
        "rtph264depay ! "
        "h264parse name=parser ! "
        "video/x-h264,stream-format=byte-stream,alignment=au ! "
        + decoder_str +
        "glupload ! "
        "qmlglsink name=qmlsink sync=false";

    GError *error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
    if (error) {
        qWarning() << "Erro GStreamer (porta" << port << "):" << error->message;
        g_error_free(error);
        return;
    }

    GstElement *parser = gst_bin_get_by_name(GST_BIN(pipeline_), "parser");
    GstPad *src_pad = gst_element_get_static_pad(parser, "src");
    gst_pad_add_probe(src_pad, GST_PAD_PROBE_TYPE_BUFFER,
                      (GstPadProbeCallback)pad_probe_callback, this, nullptr);
    gst_object_unref(src_pad);
    gst_object_unref(parser);

    GstElement *qmlsink = gst_bin_get_by_name(GST_BIN(pipeline_), "qmlsink");
    g_object_set(qmlsink, "widget", video_item_, nullptr);
    gst_object_unref(qmlsink);

    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
}

CameraStreamManager::~CameraStreamManager()
{
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
    }
}

GstPadProbeReturn CameraStreamManager::pad_probe_callback(GstPad * /*pad*/, GstPadProbeInfo *info, gpointer user_data)
{
    auto *self = static_cast<CameraStreamManager*>(user_data);
    GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
    GstMapInfo map;

    if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        const uint8_t uuid[16] = {
            0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
            0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11
        };

        size_t search_limit = std::min(map.size - 16, (size_t)128);
        for (size_t i = 0; i < search_limit - 16; ++i) {
            if (std::memcmp(map.data + i, uuid, 16) == 0) {
                size_t str_len = std::min(map.size - i - 16, (size_t)64);
                std::string payload(reinterpret_cast<char*>(map.data + i + 16), str_len);

                size_t end_pos = payload.find((char)0x80);
                if (end_pos != std::string::npos) payload = payload.substr(0, end_pos);

                uint64_t frame_id = 0, ts_ns = 0;
                if (sscanf(payload.c_str(), "ID:%lu|TS:%lu", &frame_id, &ts_ns) == 2) {
                    auto now = std::chrono::system_clock::now().time_since_epoch();
                    uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
                    double latency_ms = (now_ns - ts_ns) / 1000000.0;
                    emit self->latencyUpdated(frame_id, latency_ms);
                }
                break;
            }
        }
        gst_buffer_unmap(buffer, &map);
    }
    return GST_PAD_PROBE_OK;
}