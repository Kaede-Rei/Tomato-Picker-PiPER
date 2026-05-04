import QtQuick

Item {
    id: bg

    property bool animated: false
    property real intensity: 1.0

    Rectangle {
        anchors.fill: parent
        gradient: Gradient {
            GradientStop {
                position: 0.0
                color: "#FBF8FC"
            }

            GradientStop {
                position: 1.0
                color: "#F4EFF8"
            }
        }
    }

    Rectangle {
        width: 520
        height: 520
        radius: 260
        x: -150
        y: -125
        opacity: 0.18 * bg.intensity
        color: "#E3B7FF"
    }

    Rectangle {
        width: 500
        height: 500
        radius: 250
        x: parent.width - 285
        y: parent.height - 210
        opacity: 0.12 * bg.intensity
        color: "#A8D8FF"
    }

    Rectangle {
        width: 420
        height: 420
        radius: 210
        x: parent.width * 0.38
        y: -185
        opacity: 0.08 * bg.intensity
        color: "#FFE8A8"
    }

    Rectangle {
        anchors.fill: parent
        color: Qt.rgba(1, 1, 1, 0.22)
    }
}
