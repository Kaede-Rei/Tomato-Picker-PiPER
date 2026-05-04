import QtQuick
import "." as C

Rectangle {
    id: panel

    property color tint_color: C.Theme.glass_light
    property color stroke_color: C.Theme.glass_stroke
    property real corner_radius: 24

    radius: corner_radius
    color: tint_color
    border.color: stroke_color
    border.width: 1
    clip: false

    Behavior on color {
        ColorAnimation {
            duration: 180
        }
    }

    Behavior on border.color {
        ColorAnimation {
            duration: 180
        }
    }

    Rectangle {
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        height: 1
        radius: panel.radius
        color: Qt.rgba(1, 1, 1, 0.85)
    }

    Rectangle {
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        height: 1
        radius: panel.radius
        color: Qt.rgba(0, 0, 0, 0.06)
    }
}
