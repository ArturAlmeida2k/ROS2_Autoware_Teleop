#pragma once

#include <QObject>
#include <QQuickItem>
#include <gst/gst.h>
#include <mutex>
#include <cstdint>

class CameraStreamManager : public QObject {
    Q_OBJECT

public:
    CameraStreamManager(QQuickItem *videoItem, int port, QObject *parent = nullptr);
    ~CameraStreamManager() override;

signals:
    void latencyUpdated(uint64_t frame_id, double latency_ms);

private:
    static GstPadProbeReturn pad_probe_callback(GstPad *pad, GstPadProbeInfo *info, gpointer user_data);

    QQuickItem *video_item_;
    GstElement *pipeline_ = nullptr;
};