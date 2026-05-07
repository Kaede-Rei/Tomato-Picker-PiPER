import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "." as C

Item {
    id: overlay

    property bool active: false
    property string title: "处理中"
    property string detail: "正在执行操作..."
    property real progress: -1

    visible: opacity > 0.01
    opacity: active ? 1.0 : 0.0
    enabled: visible

    Behavior on opacity {
        NumberAnimation {
            duration: 160
            easing.type: Easing.OutCubic
        }
    }

    MouseArea {
        anchors.fill: parent
        enabled: overlay.active
        acceptedButtons: Qt.AllButtons
        hoverEnabled: true
    }

    Rectangle {
        anchors.fill: parent
        color: Qt.rgba(11 / 255, 16 / 255, 28 / 255, 0.28)
    }

    Rectangle {
        id: card
        anchors.centerIn: parent
        width: Math.min(420, Math.max(340, overlay.width * 0.34))
        height: 230
        radius: 24
        color: Qt.rgba(1, 1, 1, 0.92)
        border.color: Qt.rgba(1, 1, 1, 0.72)
        border.width: 1

        C.SoftShadow {
            corner_radius: card.radius
            strength: 0.52
        }

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 24
            spacing: 14

            BusyIndicator {
                Layout.alignment: Qt.AlignHCenter
                Layout.preferredWidth: 54
                Layout.preferredHeight: 54
                running: overlay.active
            }

            Text {
                Layout.fillWidth: true
                text: overlay.title
                color: C.Theme.text_primary
                font.family: C.Theme.font_stack
                font.pixelSize: 20
                font.bold: true
                horizontalAlignment: Text.AlignHCenter
                elide: Text.ElideRight
            }

            Text {
                Layout.fillWidth: true
                text: overlay.detail
                color: C.Theme.text_secondary
                font.family: C.Theme.font_stack
                font.pixelSize: 13
                horizontalAlignment: Text.AlignHCenter
                wrapMode: Text.WordWrap
                maximumLineCount: 2
                elide: Text.ElideRight
            }

            Rectangle {
                id: progress_track
                Layout.fillWidth: true
                Layout.preferredHeight: 8
                radius: 4
                color: Qt.rgba(0, 0, 0, 0.09)
                clip: true

                Rectangle {
                    id: progress_fill
                    height: parent.height
                    radius: parent.radius
                    color: C.Theme.mac_blue
                    width: overlay.progress >= 0 ? Math.max(8, progress_track.width * Math.max(0, Math.min(1, overlay.progress))) : progress_track.width * 0.34
                    x: overlay.progress >= 0 ? 0 : -width
                }

                NumberAnimation {
                    target: progress_fill
                    property: "x"
                    from: -progress_fill.width
                    to: progress_track.width
                    duration: 1050
                    loops: Animation.Infinite
                    running: overlay.active && overlay.progress < 0
                    easing.type: Easing.InOutCubic
                }
            }
        }
    }
}
