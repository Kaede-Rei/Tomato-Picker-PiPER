import QtQuick
import "." as C

C.GlassPanel {
    id: card

    property string camera_name: ""
    property string label: camera_name
    property bool active: false
    property var camera_status: ({})

    signal clicked()

    height: 58
    corner_radius: 18
    tint_color: active ? Qt.rgba(0.0, 0.48, 1.0, 0.88) : Qt.rgba(1, 1, 1, 0.55)
    stroke_color: active ? Qt.rgba(1, 1, 1, 0.65) : Qt.rgba(1, 1, 1, 0.72)
    scale: mouse_area.pressed ? 0.985 : mouse_area.containsMouse ? 1.018 : 1.0

    Behavior on scale {
        NumberAnimation {
            duration: 140
            easing.type: Easing.OutCubic
        }
    }

    MouseArea {
        id: mouse_area
        anchors.fill: parent
        hoverEnabled: true
        onClicked: card.clicked()
    }

    Column {
        anchors.centerIn: parent
        spacing: 2

        Text {
            text: card.label
            color: card.active ? "white" : C.Theme.text_primary
            font.family: C.Theme.font_stack
            font.pixelSize: 14
            font.bold: true
            horizontalAlignment: Text.AlignHCenter
        }

        Text {
            text: {
                var ready = card.camera_status.color_received || card.camera_status.colorReady
                var size = card.camera_status.color_size || card.camera_status.colorSize || "--"
                return ready ? "RGB " + size : "等待图像"
            }
            color: card.active ? Qt.rgba(1, 1, 1, 0.82) : C.Theme.text_secondary
            font.family: C.Theme.font_stack
            font.pixelSize: 11
            horizontalAlignment: Text.AlignHCenter
        }
    }
}
