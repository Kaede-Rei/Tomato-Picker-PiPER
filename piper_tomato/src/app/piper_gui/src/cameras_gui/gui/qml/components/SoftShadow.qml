import QtQuick

Item {
    id: shadow

    property real corner_radius: 24
    property real strength: 1.0

    anchors.fill: parent
    z: -1

    Rectangle {
        anchors.fill: parent
        anchors.margins: -8
        y: 10
        radius: shadow.corner_radius + 8
        color: Qt.rgba(0, 0, 0, 0.06 * shadow.strength)
    }

    Rectangle {
        anchors.fill: parent
        anchors.margins: -18
        y: 22
        radius: shadow.corner_radius + 18
        color: Qt.rgba(0, 0, 0, 0.035 * shadow.strength)
    }

    Rectangle {
        anchors.fill: parent
        anchors.margins: -32
        y: 38
        radius: shadow.corner_radius + 32
        color: Qt.rgba(0, 0, 0, 0.018 * shadow.strength)
    }
}
