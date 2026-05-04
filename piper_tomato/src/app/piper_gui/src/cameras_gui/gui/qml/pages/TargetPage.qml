import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "../components" as C

C.PagePanel {
    id: page

    property string target_text: "未选择目标"

    signal clear_roi_requested
    signal commit_roi_requested

    title: "目标框选"
    subtitle: "左键添加多边形顶点，双击闭合并计算目标点"

    ScrollView {
        anchors.fill: parent
        clip: true

        RowLayout {
            width: parent.width
            spacing: 14

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredWidth: 680
                Layout.fillHeight: true
                Layout.minimumHeight: 156
                radius: 16
                color: Qt.rgba(1, 1, 1, 0.52)
                border.color: Qt.rgba(0, 0, 0, 0.08)
                border.width: 1

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 8

                    RowLayout {
                        Layout.fillWidth: true

                        Text {
                            text: "当前目标"
                            color: C.Theme.text_primary
                            font.family: C.Theme.font_stack
                            font.pixelSize: 15
                            font.bold: true
                        }

                        Item {
                            Layout.fillWidth: true
                        }

                        Text {
                            text: "base_link / camera frame"
                            color: C.Theme.text_tertiary
                            font.family: C.Theme.font_stack
                            font.pixelSize: 12
                        }
                    }

                    TextArea {
                        text: page.target_text
                        readOnly: true
                        wrapMode: TextArea.Wrap
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        Layout.minimumHeight: 86

                        color: C.Theme.text_primary
                        font.family: C.Theme.mono_font_stack
                        font.pixelSize: 13

                        background: Rectangle {
                            radius: 14
                            color: Qt.rgba(1, 1, 1, 0.62)
                            border.color: Qt.rgba(0, 0, 0, 0.08)
                            border.width: 1
                        }
                    }
                }
            }

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredWidth: 360
                Layout.fillHeight: true
                Layout.minimumHeight: 156
                radius: 16
                color: Qt.rgba(1, 1, 1, 0.46)
                border.color: Qt.rgba(0, 0, 0, 0.08)
                border.width: 1

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 10

                    Text {
                        text: "操作"
                        color: C.Theme.text_primary
                        font.family: C.Theme.font_stack
                        font.pixelSize: 15
                        font.bold: true
                    }

                    Text {
                        Layout.fillWidth: true
                        text: "在上方图像区框选番茄区域。闭合后会自动计算目标点，也可以在调整 ROI 后手动重新计算。"
                        wrapMode: Text.WordWrap
                        color: C.Theme.text_secondary
                        font.family: C.Theme.font_stack
                        font.pixelSize: 13
                        lineHeight: 1.15
                    }

                    Item {
                        Layout.fillHeight: true
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 10

                        C.MacSecondaryButton {
                            text: "清空 ROI"
                            Layout.fillWidth: true
                            onClicked: page.clear_roi_requested()
                        }

                        C.MacButton {
                            text: "重新计算"
                            Layout.fillWidth: true
                            onClicked: page.commit_roi_requested()
                        }
                    }
                }
            }
        }
    }
}
