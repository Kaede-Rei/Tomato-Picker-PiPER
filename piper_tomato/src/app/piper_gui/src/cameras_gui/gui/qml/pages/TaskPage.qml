import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "../components" as C

C.PagePanel {
    id: page

    signal upsert_task_requested
    signal execute_group_requested
    signal cancel_requested
    signal go_home_requested

    title: "任务"
    subtitle: "配置 PickTaskGoal 参数，写入/执行/取消任务或返回安全区"

    function task_payload() {
        return {
            groupName: group_name.text,
            group_name: group_name.text,
            taskId: parseInt(task_id.text),
            task_id: parseInt(task_id.text),
            description: task_desc.text,
            retryTimes: parseInt(retry_times.text),
            retry_times: parseInt(retry_times.text),
            taskType: task_type.currentText,
            task_type_name: task_type.currentText,
            task_type: task_type.currentText === "PICK" ? 1 : 0,
            groupSort: group_sort.currentText,
            group_sort: group_sort.currentText,
            group_sort_type: group_sort.currentText === "DIST" ? 1 : 0,
            weightOrient: parseFloat(weight_orient.text),
            weight_orient: parseFloat(weight_orient.text),
            useEef: use_eef.checked,
            use_eef: use_eef.checked,
            usePlace: use_place.checked,
            use_place: use_place.checked,
            goHomeAfterFinish: go_home_after_finish.checked,
            go_home_after_finish: go_home_after_finish.checked,
            goSafeAfterCancel: go_safe_after_cancel.checked,
            go_safe_after_cancel: go_safe_after_cancel.checked
        };
    }

    function place_payload() {
        return {
            type: place_type.currentText,
            target_type: place_type.currentText.toLowerCase(),
            frame: "base_link",
            frame_id: "base_link",
            x: parseFloat(place_x.text),
            y: parseFloat(place_y.text),
            z: parseFloat(place_z.text),
            roll: parseFloat(place_roll.text),
            pitch: parseFloat(place_pitch.text),
            yaw: parseFloat(place_yaw.text)
        };
    }

    function tcp_compensation_payload() {
        return {
            enabled: enable_tcp_comp.checked,
            dx: parseFloat(tcp_comp_x.text),
            dy: parseFloat(tcp_comp_y.text),
            dz: parseFloat(tcp_comp_z.text)
        };
    }

    function set_combo_text(combo, value) {
        var target = String(value);
        for (var i = 0; i < combo.model.length; ++i) {
            if (String(combo.model[i]) === target) {
                combo.currentIndex = i;
                return;
            }
        }
    }

    function load_config(task, place, tcp_compensation) {
        task = task || {};
        place = place || {};
        tcp_compensation = tcp_compensation || {};

        group_name.text = task.group_name || task.groupName || group_name.text;
        task_id.text = String(task.task_id !== undefined ? task.task_id : (task.taskId !== undefined ? task.taskId : task_id.text));
        task_desc.text = task.description || task_desc.text;
        retry_times.text = String(task.retry_times !== undefined ? task.retry_times : (task.retryTimes !== undefined ? task.retryTimes : retry_times.text));
        set_combo_text(task_type, (task.task_type === 0 || task.taskType === 0) ? "MOVE_ONLY" : (task.task_type_name || task.taskType || "PICK"));
        set_combo_text(group_sort, (task.group_sort_type === 1 || task.groupSort === 1) ? "DIST" : (task.group_sort || task.groupSort || "ID"));
        weight_orient.text = String(task.weight_orient !== undefined ? task.weight_orient : (task.weightOrient !== undefined ? task.weightOrient : weight_orient.text));
        use_eef.checked = task.use_eef !== undefined ? task.use_eef : (task.useEef !== undefined ? task.useEef : use_eef.checked);
        use_place.checked = task.use_place_pose !== undefined ? task.use_place_pose : (task.usePlace !== undefined ? task.usePlace : use_place.checked);
        go_home_after_finish.checked = task.go_home_after_finish !== undefined ? task.go_home_after_finish : (task.goHomeAfterFinish !== undefined ? task.goHomeAfterFinish : go_home_after_finish.checked);
        go_safe_after_cancel.checked = task.go_safe_after_cancel !== undefined ? task.go_safe_after_cancel : (task.goSafeAfterCancel !== undefined ? task.goSafeAfterCancel : go_safe_after_cancel.checked);

        set_combo_text(place_type, (place.target_type || place.type || "point").toString().toLowerCase() === "pose" ? "Pose" : "Point");
        place_x.text = String(place.x !== undefined ? place.x : place_x.text);
        place_y.text = String(place.y !== undefined ? place.y : place_y.text);
        place_z.text = String(place.z !== undefined ? place.z : place_z.text);
        place_roll.text = String(place.roll !== undefined ? place.roll : place_roll.text);
        place_pitch.text = String(place.pitch !== undefined ? place.pitch : place_pitch.text);
        place_yaw.text = String(place.yaw !== undefined ? place.yaw : place_yaw.text);

        enable_tcp_comp.checked = tcp_compensation.enabled === true;
        tcp_comp_x.text = Number(tcp_compensation.dx !== undefined ? tcp_compensation.dx : 0).toFixed(4);
        tcp_comp_y.text = Number(tcp_compensation.dy !== undefined ? tcp_compensation.dy : 0).toFixed(4);
        tcp_comp_z.text = Number(tcp_compensation.dz !== undefined ? tcp_compensation.dz : 0).toFixed(4);
    }

    ScrollView {
        id: task_scroll
        anchors.fill: parent
        clip: true
        contentWidth: availableWidth
        contentHeight: Math.max(availableHeight, task_content.implicitHeight + 2)
        ScrollBar.vertical.policy: ScrollBar.AsNeeded
        ScrollBar.horizontal.policy: ScrollBar.AlwaysOff

        ColumnLayout {
            id: task_content
            width: task_scroll.availableWidth
            spacing: 14

            RowLayout {
                id: task_cards
                Layout.fillWidth: true
                spacing: 14

                Rectangle {
                    id: basic_card
                    Layout.fillWidth: true
                    Layout.preferredWidth: 560
                    Layout.preferredHeight: Math.max(238, basic_content.implicitHeight + 28)
                    Layout.alignment: Qt.AlignTop
                    radius: 16
                    color: Qt.rgba(1, 1, 1, 0.52)
                    border.color: Qt.rgba(0, 0, 0, 0.08)
                    border.width: 1

                    ColumnLayout {
                        id: basic_content
                        anchors.fill: parent
                        anchors.margins: 14
                        spacing: 10

                        Text {
                            text: "基础任务"
                            color: C.Theme.text_primary
                            font.family: C.Theme.font_stack
                            font.pixelSize: 15
                            font.bold: true
                        }

                        GridLayout {
                            columns: 4
                            columnSpacing: 10
                            rowSpacing: 10
                            Layout.fillWidth: true

                            C.FormLabel {
                                text: "任务组"
                            }
                            C.MacTextField {
                                id: group_name
                                text: "gui_pick"
                                Layout.fillWidth: true
                            }

                            C.FormLabel {
                                text: "任务 ID"
                            }
                            C.MacTextField {
                                id: task_id
                                text: "1"
                                Layout.fillWidth: true
                            }

                            C.FormLabel {
                                text: "描述"
                            }
                            C.MacTextField {
                                id: task_desc
                                text: "GUI采摘任务"
                                Layout.fillWidth: true
                                Layout.columnSpan: 3
                            }

                            C.FormLabel {
                                text: "重试次数"
                            }
                            C.MacTextField {
                                id: retry_times
                                text: "0"
                                Layout.fillWidth: true
                            }

                            C.FormLabel {
                                text: "任务类型"
                            }
                            C.MacComboBox {
                                id: task_type
                                model: ["PICK", "MOVE_ONLY"]
                                Layout.fillWidth: true
                            }

                            C.FormLabel {
                                text: "排序"
                            }
                            C.MacComboBox {
                                id: group_sort
                                model: ["ID", "DIST"]
                                Layout.fillWidth: true
                            }

                            C.FormLabel {
                                text: "姿态权重"
                            }
                            C.MacTextField {
                                id: weight_orient
                                text: "0.30"
                                Layout.fillWidth: true
                            }
                        }

                        Rectangle {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 1
                            color: Qt.rgba(0, 0, 0, 0.07)
                        }

                        GridLayout {
                            columns: 2
                            columnSpacing: 14
                            rowSpacing: 8
                            Layout.fillWidth: true

                            CheckBox {
                                id: use_eef
                                text: "执行夹爪"
                                checked: true
                                Layout.fillWidth: true
                            }

                            CheckBox {
                                id: use_place
                                text: "启用放置"
                                checked: true
                                Layout.fillWidth: true
                            }

                            CheckBox {
                                id: go_home_after_finish
                                text: "完成后回零"
                                checked: true
                                Layout.fillWidth: true
                            }

                            CheckBox {
                                id: go_safe_after_cancel
                                text: "取消后回安全位"
                                checked: true
                                Layout.fillWidth: true
                            }
                        }
                    }
                }

                Rectangle {
                    id: place_card
                    Layout.fillWidth: true
                    Layout.preferredWidth: 520
                    Layout.preferredHeight: Math.max(238, place_content.implicitHeight + 28)
                    Layout.alignment: Qt.AlignTop
                    radius: 16
                    color: Qt.rgba(1, 1, 1, 0.46)
                    border.color: Qt.rgba(0, 0, 0, 0.08)
                    border.width: 1

                    ColumnLayout {
                        id: place_content
                        anchors.fill: parent
                        anchors.margins: 14
                        spacing: 10

                        RowLayout {
                            Layout.fillWidth: true

                            Text {
                                text: "放置区"
                                color: C.Theme.text_primary
                                font.family: C.Theme.font_stack
                                font.pixelSize: 15
                                font.bold: true
                            }

                            Item {
                                Layout.fillWidth: true
                            }

                            Text {
                                text: "frame: base_link"
                                color: C.Theme.text_tertiary
                                font.family: C.Theme.font_stack
                                font.pixelSize: 12
                            }
                        }

                        GridLayout {
                            columns: 4
                            columnSpacing: 10
                            rowSpacing: 10
                            Layout.fillWidth: true

                            C.FormLabel {
                                text: "类型"
                            }
                            C.MacComboBox {
                                id: place_type
                                model: ["Point", "Pose"]
                                Layout.fillWidth: true
                            }

                            C.FormLabel {
                                text: "X"
                            }
                            C.MacTextField {
                                id: place_x
                                text: "0.00"
                                Layout.fillWidth: true
                            }

                            C.FormLabel {
                                text: "Y"
                            }
                            C.MacTextField {
                                id: place_y
                                text: "0.20"
                                Layout.fillWidth: true
                            }

                            C.FormLabel {
                                text: "Z"
                            }
                            C.MacTextField {
                                id: place_z
                                text: "0.20"
                                Layout.fillWidth: true
                            }

                            C.FormLabel {
                                text: "Roll"
                            }
                            C.MacTextField {
                                id: place_roll
                                text: "0.00"
                                Layout.fillWidth: true
                            }

                            C.FormLabel {
                                text: "Pitch"
                            }
                            C.MacTextField {
                                id: place_pitch
                                text: "0.00"
                                Layout.fillWidth: true
                            }

                            C.FormLabel {
                                text: "Yaw"
                            }
                            C.MacTextField {
                                id: place_yaw
                                text: "0.00"
                                Layout.fillWidth: true
                            }
                        }

                        Text {
                            Layout.fillWidth: true
                            text: "Point 模式只使用 XYZ；Pose 模式会额外使用 Roll / Pitch / Yaw。"
                            wrapMode: Text.WordWrap
                            color: C.Theme.text_secondary
                            font.family: C.Theme.font_stack
                            font.pixelSize: 12
                        }
                    }
                }
            }

            Rectangle {
                id: tcp_comp_card
                Layout.fillWidth: true
                Layout.preferredHeight: Math.max(112, tcp_comp_content.implicitHeight + 26)
                radius: 16
                color: Qt.rgba(1, 1, 1, 0.48)
                border.color: Qt.rgba(0, 0, 0, 0.08)
                border.width: 1

                ColumnLayout {
                    id: tcp_comp_content
                    anchors.fill: parent
                    anchors.margins: 13
                    spacing: 10

                    RowLayout {
                        Layout.fillWidth: true

                        Text {
                            text: "TCP 平移补偿"
                            color: C.Theme.text_primary
                            font.family: C.Theme.font_stack
                            font.pixelSize: 15
                            font.bold: true
                        }

                        Item {
                            Layout.fillWidth: true
                        }

                        CheckBox {
                            id: enable_tcp_comp
                            text: "启用"
                            checked: false
                        }
                    }

                    GridLayout {
                        columns: 6
                        columnSpacing: 10
                        rowSpacing: 10
                        Layout.fillWidth: true

                        C.FormLabel {
                            text: "X"
                        }
                        C.MacTextField {
                            id: tcp_comp_x
                            text: "0.0200"
                            Layout.fillWidth: true
                        }

                        C.FormLabel {
                            text: "Y"
                        }
                        C.MacTextField {
                            id: tcp_comp_y
                            text: "0.0200"
                            Layout.fillWidth: true
                        }

                        C.FormLabel {
                            text: "Z"
                        }
                        C.MacTextField {
                            id: tcp_comp_z
                            text: "0.0100"
                            Layout.fillWidth: true
                        }
                    }

                    Text {
                        Layout.fillWidth: true
                        text: "补偿值按 TCP 坐标系米制平移，只在目标输出坐标系为 TCP frame 时应用。"
                        color: C.Theme.text_secondary
                        font.family: C.Theme.font_stack
                        font.pixelSize: 12
                        wrapMode: Text.WordWrap
                    }
                }
            }

            Rectangle {
                id: execute_card
                Layout.fillWidth: true
                Layout.preferredHeight: Math.max(82, execute_content.implicitHeight + 26)
                radius: 16
                color: Qt.rgba(1, 1, 1, 0.50)
                border.color: Qt.rgba(0, 0, 0, 0.08)
                border.width: 1

                RowLayout {
                    id: execute_content
                    anchors.fill: parent
                    anchors.margins: 13
                    spacing: 12

                    ColumnLayout {
                        Layout.fillWidth: true
                        spacing: 3

                        Text {
                            text: "执行控制"
                            color: C.Theme.text_primary
                            font.family: C.Theme.font_stack
                            font.pixelSize: 15
                            font.bold: true
                        }

                        Text {
                            Layout.fillWidth: true
                            text: "先写入 / 更新当前任务，再执行任务组；取消或返回安全区会立即发送指令。"
                            color: C.Theme.text_secondary
                            font.family: C.Theme.font_stack
                            font.pixelSize: 12
                            wrapMode: Text.WordWrap
                        }
                    }

                    C.MacButton {
                        text: "写入 / 更新任务"
                        implicitWidth: 170
                        onClicked: page.upsert_task_requested()
                    }

                    C.MacButton {
                        text: "执行任务组"
                        implicitWidth: 140
                        onClicked: page.execute_group_requested()
                    }

                    C.MacSecondaryButton {
                        text: "取消"
                        implicitWidth: 100
                        onClicked: page.cancel_requested()
                    }

                    C.MacSecondaryButton {
                        text: "返回安全区"
                        implicitWidth: 130
                        onClicked: page.go_home_requested()
                    }
                }
            }
        }
    }
}
