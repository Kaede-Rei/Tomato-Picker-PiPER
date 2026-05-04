import QtQuick
import QtQuick.Window
import QtQuick.Controls
import QtQuick.Layouts

ApplicationWindow {
    id: root

    width: 1280
    height: 720
    visible: true
    title: qsTr("多相机采摘 ROI GUI")
    color: "#ECECF0"

    Rectangle {
        anchors.fill: parent
        color: "#ECECF0"

        Text {
            anchors.centerIn: parent
            text: "Hello, World!"
            font.pixelSize: 28
            font.bold: true
            color: "#1D1D1F"
        }
    }
}
