import QtQuick
import "." as C

Rectangle {
    id: pill

    property string label: "READY"
    property color accent: "#32D74B"

    height: 26
    radius: 13
    width: text_item.implicitWidth + 24

    color: Qt.rgba(accent.r, accent.g, accent.b, 0.14)
    border.color: Qt.rgba(accent.r, accent.g, accent.b, 0.35)
    border.width: 1

    Text {
        id: text_item
        anchors.centerIn: parent
        text: pill.label
        color: pill.accent
        font.family: C.Theme.font_stack
        font.pixelSize: 12
        font.bold: true
    }
}
