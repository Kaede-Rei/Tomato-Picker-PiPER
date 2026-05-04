import QtQuick
import "." as C

Rectangle {
    id: item

    property string label: ""
    property int page_index: 0
    property bool active: false

    signal clicked()

    height: 46
    radius: 15

    color: active ? C.Theme.mac_blue :
           mouse_area.containsMouse ? Qt.rgba(1, 1, 1, 0.74) :
                                      "transparent"

    scale: mouse_area.pressed ? 0.985 : 1.0

    Behavior on color {
        ColorAnimation { duration: 140 }
    }

    Behavior on scale {
        NumberAnimation {
            duration: 120
            easing.type: Easing.OutCubic
        }
    }

    MouseArea {
        id: mouse_area
        anchors.fill: parent
        hoverEnabled: true

        onClicked: item.clicked()
    }

    Text {
        anchors.verticalCenter: parent.verticalCenter
        anchors.left: parent.left
        anchors.leftMargin: 18
        text: item.label
        color: item.active ? "white" : C.Theme.text_primary
        font.family: C.Theme.font_stack
        font.pixelSize: 15
        font.bold: item.active
    }

    Rectangle {
        visible: item.active
        anchors.right: parent.right
        anchors.rightMargin: 10
        anchors.verticalCenter: parent.verticalCenter
        width: 5
        height: 22
        radius: 3
        color: "white"
        opacity: 0.90
    }
}
