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
    property color task_status_accent: "#32D74B"

    property var roi_points: []
    property bool roi_closed: false
    property var target_pixel: ({})

    property int auto_commit_min_points: 3
    property double suppress_click_until: 0
    property double last_click_time: 0
    property real last_click_x: -9999
    property real last_click_y: -9999

    property bool image_frame_latched: false
    property real latched_painted_width: 0
    property real latched_painted_height: 0

    property real image_zoom: 1.0
    property real min_image_zoom: 1.0
    property real max_image_zoom: 6.0
    property real image_pan_x: 0.0
    property real image_pan_y: 0.0
    property bool middle_panning: false
    property real last_pan_mouse_x: 0.0
    property real last_pan_mouse_y: 0.0

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
        panel.reset_view();
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

    onTarget_pixelChanged: roi_canvas.requestPaint()

    Timer {
        id: stream_lost_clear_timer
        interval: 600
        repeat: false

        onTriggered: {
            if (!panel.stream_ready) {
                panel.image_frame_latched = false;
                panel.latched_painted_width = 0;
                panel.latched_painted_height = 0;
                panel.reset_view();
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

    function reset_view() {
        panel.image_zoom = panel.min_image_zoom;
        panel.image_pan_x = 0;
        panel.image_pan_y = 0;
        panel.middle_panning = false;
        roi_canvas.requestPaint();
    }

    function base_image_rect() {
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

    function clamp_pan() {
        var b = panel.base_image_rect();
        if (b.w <= 4 || b.h <= 4)
            return;

        var scaled_w = b.w * panel.image_zoom;
        var scaled_h = b.h * panel.image_zoom;
        var margin_x = Math.max(40, image_stage.width * 0.35);
        var margin_y = Math.max(40, image_stage.height * 0.35);
        var limit_x = Math.max(0, (scaled_w - image_stage.width) / 2) + margin_x;
        var limit_y = Math.max(0, (scaled_h - image_stage.height) / 2) + margin_y;

        panel.image_pan_x = Math.max(-limit_x, Math.min(limit_x, panel.image_pan_x));
        panel.image_pan_y = Math.max(-limit_y, Math.min(limit_y, panel.image_pan_y));
    }

    function zoom_at(mx, my, factor) {
        if (!panel.preview_ready)
            return;

        var old_r = panel.image_rect();
        if (old_r.w <= 4 || old_r.h <= 4)
            return;

        var old_zoom = panel.image_zoom;
        var next_zoom = Math.max(panel.min_image_zoom, Math.min(panel.max_image_zoom, old_zoom * factor));
        if (Math.abs(next_zoom - old_zoom) < 0.001)
            return;

        var rel_x = (mx - old_r.x) / old_r.w;
        var rel_y = (my - old_r.y) / old_r.h;
        var b = panel.base_image_rect();
        var new_w = b.w * next_zoom;
        var new_h = b.h * next_zoom;
        var new_cx = mx - (rel_x - 0.5) * new_w;
        var new_cy = my - (rel_y - 0.5) * new_h;

        panel.image_zoom = next_zoom;
        panel.image_pan_x = new_cx - (b.x + b.w / 2);
        panel.image_pan_y = new_cy - (b.y + b.h / 2);
        panel.clamp_pan();
        roi_canvas.requestPaint();
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
        var b = panel.base_image_rect();
        var w = b.w * panel.image_zoom;
        var h = b.h * panel.image_zoom;
        var cx = b.x + b.w / 2 + panel.image_pan_x;
        var cy = b.y + b.h / 2 + panel.image_pan_y;
        return {
            x: cx - w / 2,
            y: cy - h / 2,
            w: w,
            h: h
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

    function point_inside_image(mx, my) {
        if (!panel.preview_ready)
            return false;

        var r = image_rect();
        return mx >= r.x && mx <= r.x + r.w && my >= r.y && my <= r.y + r.h;
    }

    function target_pixel_valid() {
        var px = panel.target_pixel || {};
        if (px.camera && px.camera !== panel.preview_camera)
            return false;

        return px.x !== undefined && px.y !== undefined && isFinite(px.x) && isFinite(px.y);
    }

    function add_roi_point(mx, my) {
        if (!panel.preview_ready || !panel.point_inside_image(mx, my))
            return;

        var now = Date.now();
        if (now < panel.suppress_click_until)
            return;

        var dx = mx - panel.last_click_x;
        var dy = my - panel.last_click_y;
        if (now - panel.last_click_time < 280 && Math.sqrt(dx * dx + dy * dy) < 10) {
            panel.suppress_click_until = now + 260;
            return;
        }

        panel.last_click_time = now;
        panel.last_click_x = mx;
        panel.last_click_y = my;

        if (panel.roi_closed)
            panel.clear_roi();

        var p = panel.view_to_image(mx, my);
        panel.roi_points = panel.roi_points.concat([
            {
                x: p.x,
                y: p.y
            }
        ]);
        roi_canvas.requestPaint();

        if (panel.roi_points.length >= panel.auto_commit_min_points) {
            panel.roi_closed = true;
            panel.suppress_click_until = now + 420;
            roi_canvas.requestPaint();
            panel.commit_roi();
        }
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
                max_width: Math.max(100, Math.min(280, panel.width * 0.24))
                Layout.preferredWidth: Math.min(max_width, implicitWidth)
                Layout.maximumWidth: max_width
                label: panel.task_status
                accent: panel.task_status_accent
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

            Item {
                id: image_surface
                x: panel.image_pan_x
                y: panel.image_pan_y
                width: image_stage.width
                height: image_stage.height
                scale: panel.image_zoom
                transformOrigin: Item.Center

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
                    onPaintedWidthChanged: {
                        panel.try_latch_frame();
                        panel.clamp_pan();
                    }
                    onPaintedHeightChanged: {
                        panel.try_latch_frame();
                        panel.clamp_pan();
                    }

                    Behavior on opacity {
                        NumberAnimation {
                            duration: 160
                        }
                    }
                }
            }

            Canvas {
                id: roi_canvas
                anchors.fill: parent

                onPaint: {
                    var ctx = getContext("2d");
                    ctx.reset();

                    if (!panel.preview_ready)
                        return;

                    if (panel.roi_points.length > 0) {
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

                        for (var j = 0; j < panel.roi_points.length; ++j) {
                            var q = panel.image_to_view(panel.roi_points[j]);
                            ctx.beginPath();
                            ctx.arc(q.x, q.y, 6, 0, Math.PI * 2);
                            ctx.fillStyle = "#FF453A";
                            ctx.fill();
                            ctx.lineWidth = 2;
                            ctx.strokeStyle = "white";
                            ctx.stroke();
                        }
                    }

                    if (panel.target_pixel_valid()) {
                        var tp = panel.image_to_view({
                            x: panel.target_pixel.x,
                            y: panel.target_pixel.y
                        });

                        ctx.fillStyle = "rgba(191, 90, 242, 0.20)";
                        ctx.beginPath();
                        ctx.arc(tp.x, tp.y, 13, 0, Math.PI * 2);
                        ctx.fill();

                        ctx.fillStyle = "#BF5AF2";
                        ctx.beginPath();
                        ctx.arc(tp.x, tp.y, 6, 0, Math.PI * 2);
                        ctx.fill();
                        ctx.lineWidth = 2;
                        ctx.strokeStyle = "#FFFFFF";
                        ctx.stroke();

                        var label = "目标点 (" + panel.target_pixel.x.toFixed(1) + ", " + panel.target_pixel.y.toFixed(1) + ")";
                        ctx.font = "bold 12px " + C.Theme.font_stack;
                        var label_w = ctx.measureText(label).width + 16;
                        var label_x = Math.min(roi_canvas.width - label_w - 12, Math.max(12, tp.x + 16));
                        var label_y = Math.max(32, tp.y - 18);

                        ctx.fillStyle = "rgba(0, 0, 0, 0.62)";
                        ctx.fillRect(label_x, label_y - 22, label_w, 26);
                        ctx.fillStyle = "#FFFFFF";
                        ctx.fillText(label, label_x + 8, label_y - 5);
                    }
                }
            }

            MouseArea {
                id: image_mouse
                anchors.fill: parent
                enabled: panel.preview_ready
                hoverEnabled: true
                acceptedButtons: Qt.LeftButton | Qt.RightButton | Qt.MiddleButton
                cursorShape: panel.middle_panning ? Qt.ClosedHandCursor : Qt.ArrowCursor

                onWheel: function (wheel) {
                    var factor = wheel.angleDelta.y > 0 ? 1.12 : 1.0 / 1.12;
                    panel.zoom_at(wheel.x, wheel.y, factor);
                    wheel.accepted = true;
                }

                onPressed: function (mouse) {
                    if (mouse.button === Qt.MiddleButton) {
                        panel.middle_panning = true;
                        panel.last_pan_mouse_x = mouse.x;
                        panel.last_pan_mouse_y = mouse.y;
                        mouse.accepted = true;
                    }
                }

                onPositionChanged: function (mouse) {
                    if (!panel.middle_panning)
                        return;

                    panel.image_pan_x += mouse.x - panel.last_pan_mouse_x;
                    panel.image_pan_y += mouse.y - panel.last_pan_mouse_y;
                    panel.last_pan_mouse_x = mouse.x;
                    panel.last_pan_mouse_y = mouse.y;
                    panel.clamp_pan();
                    roi_canvas.requestPaint();
                    mouse.accepted = true;
                }

                onReleased: function (mouse) {
                    if (mouse.button === Qt.MiddleButton) {
                        panel.middle_panning = false;
                        mouse.accepted = true;
                    }
                }

                onCanceled: {
                    panel.middle_panning = false;
                }

                onClicked: function (mouse) {
                    if (mouse.button === Qt.MiddleButton)
                        return;

                    if (mouse.button === Qt.RightButton) {
                        panel.clear_roi();
                        return;
                    }

                    if (mouse.button !== Qt.LeftButton)
                        return;

                    panel.add_roi_point(mouse.x, mouse.y);
                }

                onDoubleClicked: function (mouse) {
                    mouse.accepted = true;
                    panel.suppress_click_until = Date.now() + 320;

                    if (panel.preview_ready && panel.roi_points.length >= panel.auto_commit_min_points && !panel.roi_closed) {
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
                    text: panel.preview_camera + " | 左键添加点 " + Math.min(panel.roi_points.length, panel.auto_commit_min_points) + "/" + panel.auto_commit_min_points + "，满 3 点自动计算；右键清空；滚轮缩放 / 中键拖拽"
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
