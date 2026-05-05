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
    subtitle: "左键添加 3 个多边形顶点，第三个点后自动闭合并计算目标点"

    ScrollView {
        id: page_scroll
        anchors.fill: parent
        clip: true
        contentWidth: availableWidth
        contentHeight: Math.max(availableHeight, content_row.implicitHeight + 2)
        ScrollBar.vertical.policy: ScrollBar.AsNeeded
        ScrollBar.horizontal.policy: ScrollBar.AlwaysOff

        RowLayout {
            id: content_row
            width: page_scroll.availableWidth
            height: Math.max(page_scroll.availableHeight, implicitHeight)
            spacing: 14

            Rectangle {
                id: target_card
                Layout.fillWidth: true
                Layout.preferredWidth: 680
                Layout.preferredHeight: Math.max(174, page_scroll.availableHeight, target_card_content.implicitHeight + 28)
                Layout.alignment: Qt.AlignTop
                radius: 16
                color: Qt.rgba(1, 1, 1, 0.52)
                border.color: Qt.rgba(0, 0, 0, 0.08)
                border.width: 1

                ColumnLayout {
                    id: target_card_content
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

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        Layout.minimumHeight: 96
                        radius: 14
                        color: Qt.rgba(1, 1, 1, 0.62)
                        border.color: Qt.rgba(0, 0, 0, 0.08)
                        border.width: 1
                        clip: true

                        Flickable {
                            id: target_text_flick
                            anchors.fill: parent
                            anchors.margins: 1
                            clip: true
                            boundsBehavior: Flickable.StopAtBounds
                            contentWidth: Math.max(width, target_text_edit.implicitWidth + 24)
                            contentHeight: Math.max(height, target_text_edit.implicitHeight + 24)

                            TextEdit {
                                id: target_text_edit
                                x: 12
                                y: 12
                                width: Math.max(1, target_text_flick.width - 24)
                                text: page.target_text
                                readOnly: true
                                selectByMouse: true
                                wrapMode: TextEdit.Wrap
                                color: C.Theme.text_primary
                                font.family: C.Theme.mono_font_stack
                                font.pixelSize: 13
                            }

                            ScrollBar.vertical: ScrollBar {
                                policy: target_text_flick.contentHeight > target_text_flick.height + 1 ? ScrollBar.AlwaysOn : ScrollBar.AsNeeded
                            }
                        }
                    }
                }
            }

            Rectangle {
                id: op_card
                Layout.fillWidth: true
                Layout.preferredWidth: 360
                Layout.preferredHeight: Math.max(174, page_scroll.availableHeight, op_card_content.implicitHeight + 28)
                Layout.alignment: Qt.AlignTop
                radius: 16
                color: Qt.rgba(1, 1, 1, 0.46)
                border.color: Qt.rgba(0, 0, 0, 0.08)
                border.width: 1

                ColumnLayout {
                    id: op_card_content
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
                        text: "在上方图像区左键依次点选 3 个顶点。第三个点后会自动闭合并计算目标点；右键可清空 ROI。视频区支持滚轮缩放，按住鼠标中键拖拽平移。"
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
