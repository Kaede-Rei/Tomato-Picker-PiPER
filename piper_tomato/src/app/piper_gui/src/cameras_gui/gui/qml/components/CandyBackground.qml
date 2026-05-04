import QtQuick
import "." as C

Item {
    id: bg

    // 拖动窗口卡顿时先保持 false。透明窗口 + 大量动画/半透明层在 X11 上会很吃合成器。
    property bool animated: false
    property real intensity: 1.0

    Rectangle {
        anchors.fill: parent
        color: C.Theme.bg0
    }

    Rectangle {
        id: blob_left
        width: 560
        height: 560
        radius: 280
        x: -180
        y: -160
        opacity: 0.46 * bg.intensity
        gradient: Gradient {
            GradientStop {
                position: 0.0
                color: "#FF9ACB"
            }

            GradientStop {
                position: 1.0
                color: "#7C5CFF"
            }
        }

        SequentialAnimation on scale {
            running: bg.animated
            loops: Animation.Infinite

            NumberAnimation {
                to: 1.08
                duration: 4200
                easing.type: Easing.InOutSine
            }

            NumberAnimation {
                to: 1.00
                duration: 4200
                easing.type: Easing.InOutSine
            }
        }
    }

    Rectangle {
        id: blob_right
        width: 660
        height: 660
        radius: 330
        x: parent.width - 380
        y: parent.height - 430
        opacity: 0.40 * bg.intensity
        gradient: Gradient {
            GradientStop {
                position: 0.0
                color: "#64D2FF"
            }

            GradientStop {
                position: 1.0
                color: "#5E5CE6"
            }
        }
    }

    Rectangle {
        id: blob_top
        width: 420
        height: 420
        radius: 210
        x: parent.width * 0.42
        y: -150
        opacity: 0.28 * bg.intensity
        gradient: Gradient {
            GradientStop {
                position: 0.0
                color: "#FFD60A"
            }

            GradientStop {
                position: 1.0
                color: "#FF9F0A"
            }
        }
    }

    Rectangle {
        anchors.fill: parent
        color: Qt.rgba(1, 1, 1, 0.34)
    }
}
