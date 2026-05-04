import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "../components" as C

C.PagePanel {
    id: page

    title: "任务参数"
    subtitle: "配置 PickTaskGoal 写入参数"

    function task_payload() {
        return {
            group_name: group_name.text,
            task_id: parseInt(task_id.text),
            description: task_desc.text,
            task_type: task_type.currentIndex,
            use_eef: use_eef.checked,
            use_place_pose: use_place.checked,
            retry_times: parseInt(retry_times.text),
            go_safe_after_cancel: go_safe_after_cancel.checked,
            go_home_after_finish: go_home_after_finish.checked,
            group_sort_type: group_sort.currentIndex,
            weight_orient: parseFloat(weight_orient.text)
        }
    }

    function place_payload() {
        return {
            target_type: place_type.currentText === "Pose" ? "pose" : "point",
            frame_id: "base_link",
            x: parseFloat(place_x.text),
            y: parseFloat(place_y.text),
            z: parseFloat(place_z.text),
            roll: parseFloat(place_roll.text),
            pitch: parseFloat(place_pitch.text),
            yaw: parseFloat(place_yaw.text)
        }
    }

    ScrollView {
        Layout.fillWidth: true
        Layout.fillHeight: true
        clip: true

        ColumnLayout {
            width: parent.width
            spacing: 12

            GridLayout {
                columns: 4
                columnSpacing: 12
                rowSpacing: 12
                Layout.fillWidth: true

                C.FormLabel { text: "任务组" }
                C.MacTextField {
                    id: group_name
                    text: "gui_pick"
                    Layout.fillWidth: true
                }

                C.FormLabel { text: "任务 ID" }
                C.MacTextField {
                    id: task_id
                    text: "1"
                    Layout.fillWidth: true
                }

                C.FormLabel { text: "描述" }
                C.MacTextField {
                    id: task_desc
                    text: "GUI采摘任务"
                    Layout.fillWidth: true
                }

                C.FormLabel { text: "重试次数" }
                C.MacTextField {
                    id: retry_times
                    text: "0"
                    Layout.fillWidth: true
                }

                C.FormLabel { text: "任务类型" }
                C.MacComboBox {
                    id: task_type
                    model: ["PICK", "MOVE_ONLY"]
                    Layout.fillWidth: true
                }

                C.FormLabel { text: "排序" }
                C.MacComboBox {
                    id: group_sort
                    model: ["ID", "DIST"]
                    Layout.fillWidth: true
                }

                C.FormLabel { text: "姿态权重" }
                C.MacTextField {
                    id: weight_orient
                    text: "0.30"
                    Layout.fillWidth: true
                }

                Item { Layout.fillWidth: true }
            }

            RowLayout {
                spacing: 16

                CheckBox {
                    id: use_eef
                    text: "执行夹爪"
                    checked: true
                }

                CheckBox {
                    id: use_place
                    text: "启用放置"
                    checked: true
                }

                CheckBox {
                    id: go_home_after_finish
                    text: "完成后回零"
                    checked: true
                }

                CheckBox {
                    id: go_safe_after_cancel
                    text: "取消后回安全位"
                    checked: true
                }
            }

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 1
                color: Qt.rgba(0, 0, 0, 0.08)
            }

            Text {
                text: "放置区"
                color: C.Theme.text_primary
                font.family: C.Theme.font_stack
                font.pixelSize: 16
                font.bold: true
            }

            GridLayout {
                columns: 4
                columnSpacing: 12
                rowSpacing: 12
                Layout.fillWidth: true

                C.FormLabel { text: "类型" }
                C.MacComboBox {
                    id: place_type
                    model: ["Point", "Pose"]
                    Layout.fillWidth: true
                }

                C.FormLabel { text: "X" }
                C.MacTextField {
                    id: place_x
                    text: "0.00"
                    Layout.fillWidth: true
                }

                C.FormLabel { text: "Y" }
                C.MacTextField {
                    id: place_y
                    text: "0.20"
                    Layout.fillWidth: true
                }

                C.FormLabel { text: "Z" }
                C.MacTextField {
                    id: place_z
                    text: "0.20"
                    Layout.fillWidth: true
                }

                C.FormLabel { text: "Roll" }
                C.MacTextField {
                    id: place_roll
                    text: "0.00"
                    Layout.fillWidth: true
                }

                C.FormLabel { text: "Pitch" }
                C.MacTextField {
                    id: place_pitch
                    text: "0.00"
                    Layout.fillWidth: true
                }

                C.FormLabel { text: "Yaw" }
                C.MacTextField {
                    id: place_yaw
                    text: "0.00"
                    Layout.fillWidth: true
                }

                Item { Layout.fillWidth: true }
            }
        }
    }
}
