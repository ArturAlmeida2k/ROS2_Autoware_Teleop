import QtQuick 2.15
import org.freedesktop.gstreamer.GLVideoItem 1.0

Item {
    id: root
    GstGLVideoItem {
        id: videoItem
        objectName: "videoItem"
        anchors.fill: parent
    }
}