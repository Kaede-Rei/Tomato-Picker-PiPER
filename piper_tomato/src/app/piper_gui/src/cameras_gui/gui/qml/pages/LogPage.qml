import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "../components" as C

C.PagePanel {
    id: page

    property string logs: ""

    title: "运行日志"
    subtitle: "显示 ROS、相机、任务和 GUI 事件"

    ScrollView {
        id: log_scroll
        Layout.fillWidth: true
        Layout.fillHeight: true
        clip: true

        TextArea {
            id: log_text
            text: page.logs
            readOnly: true
            wrapMode: TextArea.Wrap
            color: C.Theme.text_primary
            font.family: C.Theme.mono_font_stack
            font.pixelSize: 12

            background: Rectangle {
                radius: 16
                color: Qt.rgba(1, 1, 1, 0.58)
                border.color: Qt.rgba(0, 0, 0, 0.10)
            }

            onTextChanged: {
                cursorPosition = length
            }
        }
    }
}
