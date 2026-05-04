pragma ComponentBehavior: Bound

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
            NumberAnimation { duration: 160; easing.type: Easing.OutCubic }
        }
    }

    background: Rectangle {
        radius: 12
        color: combo.activeFocus ? "#FFFFFF" : Qt.rgba(1, 1, 1, 0.58)
        border.color: combo.activeFocus ? C.Theme.mac_blue : Qt.rgba(0, 0, 0, 0.12)
        border.width: combo.activeFocus ? 2 : 1
    }

    delegate: ItemDelegate {
        id: item

        required property int index
        required property var modelData

        width: combo.width
        height: 36

        contentItem: Text {
            text: String(item.modelData)
            color: combo.currentIndex === item.index ? "white" : C.Theme.text_primary
            font.family: C.Theme.font_stack
            font.pixelSize: 14
            verticalAlignment: Text.AlignVCenter
            leftPadding: 12
        }

        background: Rectangle {
            radius: 9
            color: combo.currentIndex === item.index ? C.Theme.mac_blue :
                   item.hovered ? Qt.rgba(0, 0, 0, 0.05) : "transparent"
        }
    }

    popup: Popup {
        y: combo.height + 6
        width: combo.width
        implicitHeight: Math.min(contentItem.implicitHeight + 12, 240)
        padding: 6

        background: Rectangle {
            radius: 14
            color: Qt.rgba(1, 1, 1, 0.94)
            border.color: Qt.rgba(0, 0, 0, 0.10)
        }

        contentItem: ListView {
            clip: true
            implicitHeight: contentHeight
            model: combo.popup.visible ? combo.delegateModel : null
            currentIndex: combo.highlightedIndex
        }

        enter: Transition {
            NumberAnimation { property: "opacity"; from: 0; to: 1; duration: 120 }
        }

        exit: Transition {
            NumberAnimation { property: "opacity"; from: 1; to: 0; duration: 100 }
        }
    }
}
