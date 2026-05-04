import QtQuick
import QtQuick.Controls
import "." as C

Button {
    id: btn

    property color normal_color: C.Theme.mac_blue
    property color hover_color: C.Theme.mac_blue_hover
    property color pressed_color: C.Theme.mac_blue_pressed

    height: 42
    implicitWidth: 132
    scale: down ? 0.975 : hovered ? 1.012 : 1.0

    Behavior on scale {
        NumberAnimation {
            duration: 120
            easing.type: Easing.OutCubic
        }
    }

    contentItem: Text {
        text: btn.text
        color: "white"
        font.family: C.Theme.font_stack
        font.pixelSize: 14
        font.bold: true
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
    }

    background: Rectangle {
        radius: 13
        color: btn.down ? btn.pressed_color : btn.hovered ? btn.hover_color : btn.normal_color

        border.color: Qt.rgba(1, 1, 1, 0.26)
        border.width: 1

        Rectangle {
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.top: parent.top
            height: parent.height * 0.45
            radius: parent.radius
            color: Qt.rgba(1, 1, 1, 0.16)
        }
    }
}
