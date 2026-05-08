import QtQuick
import "." as C

Rectangle {
    id: pill

    property string label: "READY"
    property color accent: "#32D74B"
    property real max_width: 220
    property real min_width: 70
    property real horizontal_padding: 12

    implicitWidth: Math.max(pill.min_width, Math.min(pill.max_width, text_item.implicitWidth + pill.horizontal_padding * 2))
    implicitHeight: 26
    width: implicitWidth
    height: implicitHeight
    radius: height / 2
    clip: true

    color: Qt.rgba(accent.r, accent.g, accent.b, 0.14)
    border.color: Qt.rgba(accent.r, accent.g, accent.b, 0.35)
    border.width: 1

    Text {
        id: text_item
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.verticalCenter: parent.verticalCenter
        anchors.leftMargin: pill.horizontal_padding
        anchors.rightMargin: pill.horizontal_padding
        text: pill.label
        color: pill.accent
        font.family: C.Theme.font_stack
        font.pixelSize: 12
        font.bold: true
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        maximumLineCount: 1
        elide: Text.ElideRight
    }
}
