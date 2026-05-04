pragma ComponentBehavior: Bound

import QtQuick
import QtQuick.Window
import QtQuick.Controls
import QtQuick.Layouts

import "components" as C
import "pages" as P

ApplicationWindow {
    id: root

    width: 1500
    height: 930
    minimumWidth: 1220
    minimumHeight: 780
    visible: true

    color: "transparent"
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

    readonly property bool maximized_view: root.visibility === Window.Maximized || root.is_maximized
    readonly property real outer_margin: maximized_view ? 0 : 12
    readonly property real frame_radius: maximized_view ? 0 : 18
    readonly property real shell_radius: maximized_view ? 0 : 14

    property real content_gap: 20

    property real sidebar_width: 268
    property real min_sidebar_width: 190
    property real max_sidebar_width: Math.max(380, root.width * 0.36)

    property real bottom_panel_height: 286
    property real min_bottom_panel_height: 180
    property real max_bottom_panel_height: Math.max(470, root.height * 0.56)

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

    Rectangle {
        id: shell_shadow
        anchors.fill: parent
        anchors.margins: root.outer_margin + 3
        radius: root.frame_radius + 4
        color: root.maximized_view ? "transparent" : Qt.rgba(0, 0, 0, 0.08)
        y: root.maximized_view ? 0 : 5
    }

    Rectangle {
        id: window_frame
        anchors.fill: parent
        anchors.margins: root.outer_margin
        radius: root.frame_radius
        color: "#EDE6F2"
        border.color: root.maximized_view ? "transparent" : "#D7CFE2"
        border.width: root.maximized_view ? 0 : 1
        clip: true

        Rectangle {
            id: window_shell
            anchors.fill: parent
            anchors.margins: root.maximized_view ? 0 : 7
            radius: root.shell_radius
            color: "#F8F5FA"
            border.color: root.maximized_view ? "transparent" : Qt.rgba(1, 1, 1, 0.72)
            border.width: root.maximized_view ? 0 : 1
            clip: true

            C.CandyBackground {
                anchors.fill: parent
                animated: false
                intensity: 0.72
            }

            Item {
                id: content_area
                anchors.fill: parent
                anchors.leftMargin: root.maximized_view ? 10 : 18
                anchors.rightMargin: root.maximized_view ? 10 : 18
                anchors.topMargin: root.maximized_view ? 10 : 18
                anchors.bottomMargin: root.maximized_view ? 10 : 18

                C.GlassPanel {
                    id: sidebar
                    anchors.left: parent.left
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom
                    width: root.sidebar_width
                    corner_radius: 18
                    tint_color: Qt.rgba(1, 1, 1, 0.54)

                    C.SoftShadow {
                        corner_radius: sidebar.radius
                        strength: 0.36
                    }

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.leftMargin: 18
                        anchors.rightMargin: 18
                        anchors.topMargin: 58
                        anchors.bottomMargin: 18
                        spacing: 14

                        Text {
                            text: "PiPER"
                            color: C.Theme.text_primary
                            font.family: C.Theme.font_stack
                            font.pixelSize: 28
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

                        Item {
                            Layout.preferredHeight: 2
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

                Item {
                    id: right_area
                    anchors.left: sidebar.right
                    anchors.leftMargin: root.content_gap
                    anchors.right: parent.right
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom

                    C.CameraPreviewPanel {
                        id: preview_panel
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.top: parent.top
                        anchors.bottom: page_panel.top
                        anchors.bottomMargin: root.content_gap

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

                    Item {
                        id: preview_bottom_handle
                        anchors.horizontalCenter: preview_panel.horizontalCenter
                        anchors.bottom: preview_panel.bottom
                        anchors.bottomMargin: -6
                        width: 180
                        height: 18
                        z: 30

                        Rectangle {
                            anchors.centerIn: parent
                            width: 120
                            height: 8
                            radius: 4
                            color: h_handle_mouse.containsMouse || h_handle_mouse.pressed ? Qt.rgba(0.44, 0.58, 0.94, 0.95) : Qt.rgba(0.61, 0.65, 0.74, 0.40)
                        }

                        MouseArea {
                            id: h_handle_mouse
                            anchors.fill: parent
                            hoverEnabled: true
                            cursorShape: Qt.SizeVerCursor

                            property real press_y_global: 0
                            property real start_height: 0

                            onPressed: function (mouse) {
                                var p = mapToItem(content_area, mouse.x, mouse.y);
                                press_y_global = p.y;
                                start_height = root.bottom_panel_height;
                            }

                            onPositionChanged: function (mouse) {
                                if (!pressed)
                                    return;
                                var p = mapToItem(content_area, mouse.x, mouse.y);
                                var delta = p.y - press_y_global;
                                var next_h = start_height - delta;
                                next_h = Math.max(root.min_bottom_panel_height, Math.min(root.max_bottom_panel_height, next_h));
                                root.bottom_panel_height = next_h;
                            }
                        }
                    }

                    C.GlassPanel {
                        id: page_panel
                        anchors.left: parent.left
                        anchors.right: parent.right
                        anchors.bottom: parent.bottom
                        height: root.bottom_panel_height
                        corner_radius: 18
                        tint_color: Qt.rgba(1, 1, 1, 0.60)

                        C.SoftShadow {
                            corner_radius: page_panel.radius
                            strength: 0.36
                        }

                        Item {
                            id: page_host
                            anchors.fill: parent
                            anchors.margins: 18
                            clip: true

                            P.CameraStatusPage {
                                anchors.fill: parent
                                visible: root.current_page === 0
                                camera_names: root.camera_names
                                camera_statuses: root.camera_statuses
                            }

                            P.TargetPage {
                                anchors.fill: parent
                                visible: root.current_page === 1
                                target_text: root.target_text

                                onClear_roi_requested: preview_panel.clear_roi()
                                onCommit_roi_requested: preview_panel.commit_roi()
                            }

                            P.TaskPage {
                                id: task_page
                                anchors.fill: parent
                                visible: root.current_page === 2
                            }

                            P.ExecutePage {
                                anchors.fill: parent
                                visible: root.current_page === 3

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
                                anchors.fill: parent
                                visible: root.current_page === 4
                                logs: root.logs
                            }
                        }
                    }
                }

                Item {
                    id: preview_left_handle
                    anchors.left: right_area.left
                    anchors.leftMargin: -8
                    anchors.verticalCenter: preview_panel.verticalCenter
                    width: 18
                    height: 520
                    z: 30

                    Rectangle {
                        anchors.centerIn: parent
                        width: 8
                        height: 104
                        radius: 4
                        color: v_handle_mouse.containsMouse || v_handle_mouse.pressed ? Qt.rgba(0.44, 0.58, 0.94, 0.95) : Qt.rgba(0.61, 0.65, 0.74, 0.38)
                    }

                    MouseArea {
                        id: v_handle_mouse
                        anchors.fill: parent
                        hoverEnabled: true
                        cursorShape: Qt.SizeHorCursor

                        property real press_x_global: 0
                        property real start_width: 0

                        onPressed: function (mouse) {
                            var p = mapToItem(content_area, mouse.x, mouse.y);
                            press_x_global = p.x;
                            start_width = root.sidebar_width;
                        }

                        onPositionChanged: function (mouse) {
                            if (!pressed)
                                return;
                            var p = mapToItem(content_area, mouse.x, mouse.y);
                            var delta = p.x - press_x_global;
                            var next_w = start_width + delta;
                            next_w = Math.max(root.min_sidebar_width, Math.min(root.max_sidebar_width, next_w));
                            root.sidebar_width = next_w;
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
                z: 9999

                onClose_requested: root.close()
                onMinimize_requested: root.showMinimized()
                onMaximize_requested: root.toggle_maximize()
            }
        }
    }
}
