pragma ComponentBehavior: Bound

import QtQuick
import QtQuick.Window
import QtQuick.Controls
import QtQuick.Layouts

import "components" as C
import "pages" as P

ApplicationWindow {
    id: root

    width: 1480
    height: 920
    minimumWidth: 1180
    minimumHeight: 760
    visible: true

    // 不再 transparent。Linux/X11 下透明无边框窗口拖动时会让合成器重绘整窗，拖动会明显变卡。
    color: C.Theme.bg0
    title: "PiPER Multi-Camera GUI"
    flags: Qt.FramelessWindowHint | Qt.Window

    property var cameras: ({})
    property var camera_names: []
    property var camera_statuses: ({})

    property string preview_camera: "wrist"
    property string preview_source: "image://cameras/wrist?rev=0"

    property string target_text: "未选择目标"
    property string task_status: "未连接"
    property string logs: ""

    property int current_page: 0
    property real page_scale: 1.0
    property real page_offset: 0.0
    property bool is_maximized: false
    property var nav_items: ["相机", "框选", "任务", "执行", "日志"]

    Component.onCompleted: {
        root.load_state(JSON.parse(backend.state_init()));
        root.append_log("PiPER QML GUI 已启动");
    }

    Connections {
        target: backend

        function onImage_changed(url) {
            root.preview_source = url;
        }

        function onCamera_status_changed(raw) {
            root.camera_statuses = JSON.parse(raw);
        }

        function onTarget_changed(raw) {
            var obj = JSON.parse(raw);
            root.update_target_text(obj);
        }

        function onTask_status_changed(message) {
            root.task_status = message;
            root.append_log(message);
        }

        function onLog_changed(message) {
            root.append_log(message);
        }

        function onState_changed(raw_state) {
            root.load_state(JSON.parse(raw_state));
        }
    }

    function toggle_maximize() {
        if (root.is_maximized) {
            root.showNormal();
            root.is_maximized = false;
        } else {
            root.showMaximized();
            root.is_maximized = true;
        }
    }

    function load_state(next_state) {
        root.cameras = next_state.cameras || {};
        root.camera_names = next_state.camera_names || next_state.cameraNames || Object.keys(root.cameras);

        if (root.camera_names.length > 0) {
            root.preview_camera = next_state.preview_camera || next_state.previewCamera || root.camera_names[0];
            backend.set_preview_camera(root.preview_camera);
        }
    }

    function append_log(message) {
        var now = new Date();
        root.logs += "[" + now.toLocaleTimeString() + "] " + message + "\n";
    }

    function fmt3(arr) {
        if (!arr || arr.length < 3)
            return "--";
        return arr[0].toFixed(4) + ", " + arr[1].toFixed(4) + ", " + arr[2].toFixed(4);
    }

    function update_target_text(obj) {
        root.target_text = "相机: " + obj.camera + "\n" + "像素: (" + obj.pixel.u.toFixed(1) + ", " + obj.pixel.v.toFixed(1) + ")\n" + "深度: " + obj.depthM.toFixed(3) + " m\n" + "相机坐标: " + root.fmt3(obj.cameraXYZ) + "\n" + "目标坐标[" + obj.targetFrame + "]: " + root.fmt3(obj.targetXYZ);
    }

    function state_json() {
        return JSON.stringify({
            preview_camera: root.preview_camera,
            cameras: root.cameras,
            task: task_page.task_payload(),
            place: task_page.place_payload()
        });
    }

    function commit_roi(payload) {
        var result = JSON.parse(backend.commit_polygon_selection(JSON.stringify(payload)));

        if (!result.ok) {
            root.target_text = "目标选择失败：\n" + result.message;
            root.append_log("目标选择失败：" + result.message);
            return;
        }

        root.update_target_text(result);
        root.append_log("目标已锁定：" + result.camera + " -> " + result.targetFrame);
    }

    C.CandyBackground {
        anchors.fill: parent
        animated: false
        intensity: 0.92
    }

    RowLayout {
        anchors.fill: parent
        anchors.margins: 18
        spacing: 16

        C.GlassPanel {
            id: sidebar
            Layout.preferredWidth: 220
            Layout.fillHeight: true
            corner_radius: 26
            tint_color: Qt.rgba(1, 1, 1, 0.50)

            C.SoftShadow {
                corner_radius: sidebar.radius
                strength: 0.7
            }

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 16
                spacing: 12

                Item {
                    Layout.preferredHeight: 8
                    Layout.fillWidth: true
                }

                Text {
                    text: "PiPER"
                    color: C.Theme.text_primary
                    font.family: C.Theme.font_stack
                    font.pixelSize: 26
                    font.bold: true
                }

                Text {
                    text: "Multi-Camera Picker"
                    color: C.Theme.text_secondary
                    font.family: C.Theme.font_stack
                    font.pixelSize: 13
                }

                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 1
                    color: Qt.rgba(0, 0, 0, 0.08)
                }

                Repeater {
                    model: root.nav_items

                    delegate: C.SidebarButton {
                        required property string modelData
                        required property int index

                        Layout.fillWidth: true
                        label: modelData
                        page_index: index
                        active: root.current_page === index

                        onClicked: {
                            root.current_page = page_index;
                        }
                    }
                }

                Item {
                    Layout.fillHeight: true
                }

                C.StatusPill {
                    label: root.task_status
                    accent: root.task_status.indexOf("失败") >= 0 ? "#FF453A" : root.task_status.indexOf("未") >= 0 ? "#FF9F0A" : "#32D74B"
                }

                C.MacSecondaryButton {
                    text: "连接 ROS"
                    Layout.fillWidth: true
                    onClicked: backend.connect_ros(root.state_json())
                }
            }
        }

        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 16

            C.CameraPreviewPanel {
                id: preview_panel

                Layout.fillWidth: true
                Layout.fillHeight: true

                camera_names: root.camera_names
                cameras: root.cameras
                camera_statuses: root.camera_statuses
                preview_camera: root.preview_camera
                preview_source: root.preview_source
                task_status: root.task_status

                onPreview_camera_requested: function (camera_name) {
                    root.preview_camera = camera_name;
                    backend.set_preview_camera(camera_name);
                }

                onRoi_commit_requested: function (payload) {
                    root.commit_roi(payload);
                }
            }

            C.GlassPanel {
                id: page_panel

                Layout.fillWidth: true
                Layout.preferredHeight: Math.max(290, root.height * 0.30)
                corner_radius: 24
                tint_color: Qt.rgba(1, 1, 1, 0.58)

                C.SoftShadow {
                    corner_radius: page_panel.radius
                    strength: 0.65
                }

                Item {
                    anchors.fill: parent
                    anchors.margins: 16

                    StackLayout {
                        id: page_stack

                        anchors.fill: parent
                        currentIndex: root.current_page
                        scale: root.page_scale
                        y: root.page_offset

                        onCurrentIndexChanged: {
                            opacity = 0;
                            root.page_scale = 0.975;
                            root.page_offset = 16;
                            page_enter_anim.restart();
                        }

                        P.CameraStatusPage {
                            camera_names: root.camera_names
                            camera_statuses: root.camera_statuses
                        }

                        P.TargetPage {
                            target_text: root.target_text

                            onClear_roi_requested: preview_panel.clear_roi()
                            onCommit_roi_requested: preview_panel.commit_roi()
                        }

                        P.TaskPage {
                            id: task_page
                        }

                        P.ExecutePage {
                            onUpsert_task_requested: {
                                var r = JSON.parse(backend.upsert_current_task(root.state_json()));
                                root.append_log(r.message);
                            }

                            onExecute_group_requested: {
                                var r = JSON.parse(backend.execute_group(root.state_json()));
                                root.append_log(r.message);
                            }

                            onCancel_requested: backend.cancel_task()
                            onGo_home_requested: backend.go_home()
                        }

                        P.LogPage {
                            logs: root.logs
                        }
                    }
                }

                ParallelAnimation {
                    id: page_enter_anim

                    NumberAnimation {
                        target: page_stack
                        property: "opacity"
                        to: 1
                        duration: 180
                        easing.type: Easing.OutCubic
                    }

                    NumberAnimation {
                        target: root
                        property: "page_scale"
                        to: 1
                        duration: 230
                        easing.type: Easing.OutCubic
                    }

                    NumberAnimation {
                        target: root
                        property: "page_offset"
                        to: 0
                        duration: 230
                        easing.type: Easing.OutCubic
                    }
                }
            }
        }
    }

    C.WindowChrome {
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        target_window: root
        title: ""

        onClose_requested: root.close()
        onMinimize_requested: root.showMinimized()
        onMaximize_requested: root.toggle_maximize()
    }
}
