pragma ComponentBehavior: Bound

import QtQuick
import QtQuick.Layouts
import "." as C

C.GlassPanel {
    id: panel

    property var camera_names: []
    property var cameras: ({})
    property var camera_statuses: ({})

    property string preview_camera: "wrist"
    property string preview_source: "image://cameras/wrist?rev=0"
    property string task_status: "未连接"

    property var roi_points: []
    property bool roi_closed: false

    property bool image_frame_latched: false
    property real latched_painted_width: 0
    property real latched_painted_height: 0

    readonly property bool stream_ready: {
        var st = panel.camera_statuses[panel.preview_camera] || {};
        return st.color_received || st.colorReady || false;
    }

    readonly property bool source_has_real_frame: panel.preview_source.indexOf("rev=0") < 0
    readonly property bool image_geometry_ready: preview_image.paintedWidth > 4 && preview_image.paintedHeight > 4
    readonly property bool preview_ready: panel.stream_ready && panel.image_frame_latched && panel.latched_painted_width > 4 && panel.latched_painted_height > 4

    signal preview_camera_requested(string camera_name)
    signal roi_commit_requested(var payload)
    signal roi_cleared

    onPreview_cameraChanged: {
        panel.image_frame_latched = false;
        panel.latched_painted_width = 0;
        panel.latched_painted_height = 0;
    }

    onPreview_sourceChanged: {
        panel.try_latch_frame();
    }

    onStream_readyChanged: {
        if (panel.stream_ready) {
            stream_lost_clear_timer.stop();
        } else {
            stream_lost_clear_timer.restart();
        }
    }

    Timer {
        id: stream_lost_clear_timer
        interval: 600
        repeat: false

        onTriggered: {
            if (!panel.stream_ready) {
                panel.image_frame_latched = false;
                panel.latched_painted_width = 0;
                panel.latched_painted_height = 0;
                panel.clear_roi();
            }
        }
    }

    function try_latch_frame() {
        if (!panel.stream_ready || !panel.source_has_real_frame || preview_image.status !== Image.Ready)
            return;

        if (preview_image.paintedWidth > 4 && preview_image.paintedHeight > 4) {
            panel.latched_painted_width = preview_image.paintedWidth;
            panel.latched_painted_height = preview_image.paintedHeight;
            panel.image_frame_latched = true;
            roi_canvas.requestPaint();
        }
    }

    function clear_roi() {
        roi_points = [];
        roi_closed = false;
        roi_canvas.requestPaint();
        roi_cleared();
    }

    function color_size_for_camera(name) {
        var st = camera_statuses[name] || {};
        var size = st.color_size || st.colorSize || "1280x720";
        var parts = size.split("x");
        if (parts.length !== 2)
            return {
                w: 1280,
                h: 720
            };

        return {
            w: parseInt(parts[0]),
            h: parseInt(parts[1])
        };
    }

    function image_rect() {
        var iw = preview_image.paintedWidth > 4 ? preview_image.paintedWidth : panel.latched_painted_width;
        var ih = preview_image.paintedHeight > 4 ? preview_image.paintedHeight : panel.latched_painted_height;
        var ox = preview_image.x + (preview_image.width - iw) / 2;
        var oy = preview_image.y + (preview_image.height - ih) / 2;
        return {
            x: ox,
            y: oy,
            w: iw,
            h: ih
        };
    }

    function view_to_image(mx, my) {
        if (!panel.preview_ready)
            return {
                x: 0,
                y: 0
            };

        var r = image_rect();
        var s = color_size_for_camera(preview_camera);

        var u = (mx - r.x) / r.w * s.w;
        var v = (my - r.y) / r.h * s.h;

        u = Math.max(0, Math.min(s.w - 1, u));
        v = Math.max(0, Math.min(s.h - 1, v));

        return {
            x: u,
            y: v
        };
    }

    function image_to_view(p) {
        if (!panel.preview_ready)
            return {
                x: 0,
                y: 0
            };

        var r = image_rect();
        var s = color_size_for_camera(preview_camera);

        return {
            x: r.x + p.x / s.w * r.w,
            y: r.y + p.y / s.h * r.h
        };
    }

    function commit_roi() {
        if (!panel.preview_ready || roi_points.length < 3)
            return;
        var cam_cfg = cameras[preview_camera] || {};
        var output_frame = cam_cfg.default_output_frame || cam_cfg.target_frame || "base_link";

        var payload = {
            camera: preview_camera,
            points: roi_points,
            outputFrame: output_frame,
            output_frame: output_frame,
            pointMethod: "skeleton_centroid",
            point_method: "skeleton_centroid"
        };

        roi_commit_requested(payload);
    }

    corner_radius: 28
    tint_color: Qt.rgba(1, 1, 1, 0.57)

    C.SoftShadow {
        corner_radius: panel.radius
        strength: 0.50
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 14

        RowLayout {
            Layout.fillWidth: true
            spacing: 12

            Repeater {
                model: panel.camera_names

                delegate: C.CameraButton {
                    required property string modelData

                    Layout.preferredWidth: 168
                    camera_name: modelData
                    label: {
                        var cfg = panel.cameras[modelData] || {};
                        return cfg.label || modelData;
                    }
                    active: panel.preview_camera === modelData
                    camera_status: panel.camera_statuses[modelData] || ({})

                    onClicked: {
                        panel.preview_camera_requested(camera_name);
                        panel.clear_roi();
                    }
                }
            }

            Item {
                Layout.fillWidth: true
            }

            C.StatusPill {
                label: panel.task_status
                accent: panel.task_status.indexOf("失败") >= 0 ? "#FF453A" : "#32D74B"
            }
        }

        Rectangle {
            id: image_stage
            Layout.fillWidth: true
            Layout.fillHeight: true
            radius: 24
            color: "#050D1D"
            clip: true
            border.color: Qt.rgba(1, 1, 1, 0.08)
            border.width: 1

            Image {
                id: preview_image
                anchors.fill: parent
                anchors.margins: 1
                source: panel.preview_source
                fillMode: Image.PreserveAspectFit
                cache: false
                asynchronous: false

                opacity: panel.image_frame_latched ? 1.0 : 0.10

                onStatusChanged: panel.try_latch_frame()
                onPaintedWidthChanged: panel.try_latch_frame()
                onPaintedHeightChanged: panel.try_latch_frame()

                Behavior on opacity {
                    NumberAnimation {
                        duration: 160
                    }
                }
            }

            Canvas {
                id: roi_canvas
                anchors.fill: parent

                onPaint: {
                    var ctx = getContext("2d");
                    ctx.reset();

                    if (!panel.preview_ready || panel.roi_points.length === 0)
                        return;
                    ctx.lineWidth = 3;
                    ctx.strokeStyle = "#32D74B";
                    ctx.fillStyle = "rgba(50, 215, 75, 0.22)";

                    var p0 = panel.image_to_view(panel.roi_points[0]);
                    ctx.beginPath();
                    ctx.moveTo(p0.x, p0.y);

                    for (var i = 1; i < panel.roi_points.length; ++i) {
                        var p = panel.image_to_view(panel.roi_points[i]);
                        ctx.lineTo(p.x, p.y);
                    }

                    if (panel.roi_points.length >= 3)
                        ctx.closePath();

                    ctx.stroke();

                    if (panel.roi_points.length >= 3)
                        ctx.fill();

                    ctx.fillStyle = "#FF453A";
                    for (var j = 0; j < panel.roi_points.length; ++j) {
                        var q = panel.image_to_view(panel.roi_points[j]);
                        ctx.beginPath();
                        ctx.arc(q.x, q.y, 5, 0, Math.PI * 2);
                        ctx.fill();
                    }
                }
            }

            MouseArea {
                anchors.fill: parent
                enabled: panel.preview_ready
                acceptedButtons: Qt.LeftButton | Qt.RightButton

                onClicked: function (mouse) {
                    if (mouse.button === Qt.RightButton) {
                        panel.clear_roi();
                        return;
                    }

                    if (panel.roi_closed) {
                        panel.clear_roi();
                    }

                    var p = panel.view_to_image(mouse.x, mouse.y);
                    panel.roi_points.push(p);
                    panel.roi_points = panel.roi_points;
                    roi_canvas.requestPaint();
                }

                onDoubleClicked: function (mouse) {
                    if (panel.preview_ready && panel.roi_points.length >= 3) {
                        panel.roi_closed = true;
                        roi_canvas.requestPaint();
                        panel.commit_roi();
                    }
                }
            }

            Rectangle {
                anchors.left: parent.left
                anchors.top: parent.top
                anchors.margins: 14
                radius: 11
                color: "#B5000000"
                height: 38
                width: status_text.implicitWidth + 26

                Text {
                    id: status_text
                    anchors.centerIn: parent
                    text: panel.preview_camera + " | 左键框选，双击闭合，右键清空"
                    color: "white"
                    font.family: C.Theme.font_stack
                    font.pixelSize: 14
                }
            }

            Rectangle {
                anchors.centerIn: parent
                width: 180
                height: 56
                radius: 16
                color: Qt.rgba(0, 0, 0, 0.48)
                border.color: Qt.rgba(1, 1, 1, 0.10)
                border.width: 1
                visible: !panel.preview_ready

                Text {
                    anchors.centerIn: parent
                    text: "等待图像流"
                    color: "white"
                    font.family: C.Theme.font_stack
                    font.pixelSize: 22
                    font.bold: true
                }
            }
        }
    }
}
