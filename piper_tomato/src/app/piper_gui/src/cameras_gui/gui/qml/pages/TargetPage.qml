import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "../components" as C

C.PagePanel {
    id: page

    property string target_text: "未选择目标"

    signal clear_roi_requested()
    signal commit_roi_requested()

    title: "目标框选"
    subtitle: "左键添加多边形顶点，双击闭合并计算目标点"

    ScrollView {
        anchors.fill: parent
        clip: true

        ColumnLayout {
            width: parent.width
            spacing: 12

            TextArea {
                text: page.target_text
                readOnly: true
                wrapMode: TextArea.Wrap
                Layout.fillWidth: true
                Layout.preferredHeight: 150

                color: C.Theme.text_primary
                font.family: C.Theme.mono_font_stack
                font.pixelSize: 13

                background: Rectangle {
                    radius: 16
                    color: Qt.rgba(1, 1, 1, 0.58)
                    border.color: Qt.rgba(0, 0, 0, 0.10)
                }
            }

            RowLayout {
                spacing: 12

                C.MacSecondaryButton {
                    text: "清空 ROI"
                    onClicked: page.clear_roi_requested()
                }

                C.MacButton {
                    text: "重新计算"
                    onClicked: page.commit_roi_requested()
                }
            }
        }
    }
}
