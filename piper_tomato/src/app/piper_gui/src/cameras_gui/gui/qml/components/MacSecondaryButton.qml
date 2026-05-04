import QtQuick
import QtQuick.Controls
import "." as C

Button {
    id: btn

    height: 42
    implicitWidth: 132
    scale: down ? 0.975 : hovered ? 1.008 : 1.0

    Behavior on scale {
        NumberAnimation {
            duration: 120
            easing.type: Easing.OutCubic
        }
    }

    contentItem: Text {
        text: btn.text
        color: C.Theme.text_primary
        font.family: C.Theme.font_stack
        font.pixelSize: 14
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
    }

    background: Rectangle {
        radius: 13
        color: btn.down ? "#DADAE0" :
               btn.hovered ? "#F6F6FA" :
                             Qt.rgba(1, 1, 1, 0.62)

        border.color: Qt.rgba(0, 0, 0, 0.10)
        border.width: 1
    }
}
