import QtQuick
import "." as C

Item {
    id: chrome

    property var target_window
    property string title: ""
    property bool show_frame: true

    signal close_requested()
    signal minimize_requested()
    signal maximize_requested()

    height: 42
    z: 9999

    // FramelessWindowHint 下没有系统标题栏，拖动必须自己做。
    // 只把顶部条作为拖动区，避免影响图像 ROI 框选。
    MouseArea {
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton

        onPressed: function(mouse) {
            if (chrome.target_window) {
                chrome.target_window.startSystemMove()
            }
        }

        onDoubleClicked: {
            chrome.maximize_requested()
        }
    }

    Rectangle {
        id: traffic_frame
        visible: chrome.show_frame
        anchors.left: parent.left
        anchors.leftMargin: 14
        anchors.top: parent.top
        anchors.topMargin: 8
        width: 82
        height: 24
        radius: 12
        color: Qt.rgba(1, 1, 1, 0.42)
        border.color: Qt.rgba(0, 0, 0, 0.08)
        border.width: 1

        Rectangle {
            anchors.fill: parent
            anchors.margins: 1
            radius: 11
            color: Qt.rgba(1, 1, 1, 0.16)
        }
    }

    Row {
        anchors.left: parent.left
        anchors.leftMargin: 26
        anchors.top: parent.top
        anchors.topMargin: 14
        spacing: 8

        Rectangle {
            width: 13
            height: 13
            radius: 7
            color: close_area.containsMouse ? "#FF453A" : Qt.rgba(1, 0.27, 0.23, 0.72)
            border.color: Qt.rgba(0, 0, 0, 0.12)
            border.width: 1

            MouseArea {
                id: close_area
                anchors.fill: parent
                hoverEnabled: true
                onClicked: chrome.close_requested()
            }
        }

        Rectangle {
            width: 13
            height: 13
            radius: 7
            color: min_area.containsMouse ? "#FFD60A" : Qt.rgba(1, 0.84, 0.04, 0.72)
            border.color: Qt.rgba(0, 0, 0, 0.12)
            border.width: 1

            MouseArea {
                id: min_area
                anchors.fill: parent
                hoverEnabled: true
                onClicked: chrome.minimize_requested()
            }
        }

        Rectangle {
            width: 13
            height: 13
            radius: 7
            color: max_area.containsMouse ? "#32D74B" : Qt.rgba(0.19, 0.84, 0.29, 0.72)
            border.color: Qt.rgba(0, 0, 0, 0.12)
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
        anchors.centerIn: parent
        text: chrome.title
        visible: chrome.title.length > 0
        color: C.Theme.text_secondary
        font.family: C.Theme.font_stack
        font.pixelSize: 12
    }
}
