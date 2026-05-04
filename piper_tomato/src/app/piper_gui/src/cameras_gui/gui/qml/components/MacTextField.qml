import QtQuick
import QtQuick.Controls
import "." as C

TextField {
    id: field

    height: 40
    selectByMouse: true
    color: C.Theme.text_primary
    placeholderTextColor: C.Theme.text_tertiary
    selectionColor: C.Theme.mac_blue
    selectedTextColor: "white"
    font.family: C.Theme.font_stack
    font.pixelSize: 14
    leftPadding: 13
    rightPadding: 13

    background: Rectangle {
        radius: 12
        color: field.activeFocus ? "#FFFFFF" : Qt.rgba(1, 1, 1, 0.58)
        border.color: field.activeFocus ? C.Theme.mac_blue : Qt.rgba(0, 0, 0, 0.12)
        border.width: field.activeFocus ? 2 : 1

        Behavior on border.color {
            ColorAnimation {
                duration: 150
            }
        }

        Behavior on color {
            ColorAnimation {
                duration: 150
            }
        }
    }
}
