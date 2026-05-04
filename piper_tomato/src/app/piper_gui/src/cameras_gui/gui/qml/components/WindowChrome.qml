import QtQuick
import QtQuick.Window
import "." as C

Item {
    id: chrome

    property var target_window
    property string title: ""

    signal close_requested()
    signal minimize_requested()
    signal maximize_requested()

    height: 52
    z: 9999

    MouseArea {
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton

        onPressed: function(mouse) {
            if (!chrome.target_window)
                return

            if (chrome.target_window.visibility === Window.Maximized || chrome.target_window.is_maximized) {
                chrome.target_window.showNormal()
                chrome.target_window.is_maximized = false
            }

            chrome.target_window.startSystemMove()
        }

        onDoubleClicked: {
            chrome.maximize_requested()
        }
    }

    Rectangle {
        id: traffic_capsule
        anchors.left: parent.left
        anchors.leftMargin: 18
        anchors.top: parent.top
        anchors.topMargin: 10
        width: 86
        height: 26
        radius: 13
        color: Qt.rgba(1, 1, 1, 0.54)
        border.color: Qt.rgba(0.76, 0.72, 0.82, 0.95)
        border.width: 1

        Rectangle {
            anchors.fill: parent
            anchors.margins: 1
            radius: 12
            color: Qt.rgba(1, 1, 1, 0.12)
        }
    }

    Row {
        anchors.left: parent.left
        anchors.leftMargin: 28
        anchors.top: parent.top
        anchors.topMargin: 17
        spacing: 10

        Rectangle {
            width: 12
            height: 12
            radius: 6
            color: close_area.containsMouse ? "#FF453A" : "#FF5F57"
            border.color: Qt.rgba(0, 0, 0, 0.16)
            border.width: 1

            MouseArea {
                id: close_area
                anchors.fill: parent
                hoverEnabled: true
                onClicked: chrome.close_requested()
            }
        }

        Rectangle {
            width: 12
            height: 12
            radius: 6
            color: min_area.containsMouse ? "#FFBD2E" : "#FEBB2E"
            border.color: Qt.rgba(0, 0, 0, 0.16)
            border.width: 1

            MouseArea {
                id: min_area
                anchors.fill: parent
                hoverEnabled: true
                onClicked: chrome.minimize_requested()
            }
        }

        Rectangle {
            width: 12
            height: 12
            radius: 6
            color: "#28C840"
            border.color: Qt.rgba(0, 0, 0, 0.16)
            border.width: 1

            MouseArea {
                id: max_area
                anchors.fill: parent
                hoverEnabled: true
                onClicked: chrome.maximize_requested()
            }
        }
    }

    Text {
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: parent.top
        anchors.topMargin: 17
        text: chrome.title
        visible: chrome.title.length > 0
        color: C.Theme.text_secondary
        font.family: C.Theme.font_stack
        font.pixelSize: 12
    }
}
