import QtQuick
import QtQuick.Controls
import "." as C

ComboBox {
    id: combo

    height: 40
    font.family: C.Theme.font_stack
    font.pixelSize: 14

    contentItem: Text {
        text: combo.displayText
        color: C.Theme.text_primary
        font: combo.font
        verticalAlignment: Text.AlignVCenter
        leftPadding: 13
        rightPadding: 34
        elide: Text.ElideRight
    }

    indicator: Text {
        x: combo.width - width - 12
        y: combo.topPadding + (combo.availableHeight - height) / 2
        text: "⌄"
        color: C.Theme.text_secondary
        font.family: C.Theme.font_stack
        font.pixelSize: 16
        rotation: combo.popup.visible ? 180 : 0

        Behavior on rotation {
            NumberAnimation {
                duration: 150
                easing.type: Easing.OutCubic
            }
        }
    }

    background: Rectangle {
        radius: 12
        color: combo.activeFocus ? "#FFFFFF" : Qt.rgba(1, 1, 1, 0.60)
        border.color: combo.activeFocus ? C.Theme.mac_blue : Qt.rgba(0, 0, 0, 0.12)
        border.width: combo.activeFocus ? 2 : 1
    }
}
