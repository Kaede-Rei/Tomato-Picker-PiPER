pragma ComponentBehavior: Bound

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "../components" as C

C.PagePanel {
    id: page

    property var camera_names: []
    property var camera_statuses: ({})

    title: "相机状态"
    subtitle: "检查 RGB、Depth、Frame 与数据延迟"

    ScrollView {
        anchors.fill: parent
        clip: true

        ColumnLayout {
            width: parent.width
            spacing: 10

            Repeater {
                model: page.camera_names

                delegate: C.GlassPanel {
                    id: camera_card
                    required property string modelData

                    Layout.fillWidth: true
                    Layout.preferredHeight: 78
                    corner_radius: 18
                    tint_color: Qt.rgba(1, 1, 1, 0.56)

                    RowLayout {
                        anchors.fill: parent
                        anchors.margins: 14
                        spacing: 14

                        Text {
                            text: camera_card.modelData
                            color: C.Theme.text_primary
                            font.family: C.Theme.font_stack
                            font.pixelSize: 16
                            font.bold: true
                            Layout.preferredWidth: 80
                        }

                        Text {
                            Layout.fillWidth: true
                            color: C.Theme.text_secondary
                            font.family: C.Theme.font_stack
                            font.pixelSize: 13
                            elide: Text.ElideRight

                            text: {
                                var st = page.camera_statuses[camera_card.modelData] || {};
                                var color_ready = st.color_received || st.colorReady;
                                var depth_ready = st.depth_received || st.depthReady;
                                var color_size = st.color_size || st.colorSize || "--";
                                var depth_size = st.depth_size || st.depthSize || "--";
                                var depth_frame = st.depth_frame || st.depthFrame || "--";

                                return "RGB: " + (color_ready ? color_size : "--") + "    Depth: " + (depth_ready ? depth_size : "--") + "    Frame: " + depth_frame;
                            }
                        }

                        C.StatusPill {
                            label: {
                                var st = page.camera_statuses[camera_card.modelData] || {};
                                var ready = st.color_received || st.colorReady;
                                return ready ? "LIVE" : "WAIT";
                            }

                            accent: {
                                var st = page.camera_statuses[camera_card.modelData] || {};
                                var ready = st.color_received || st.colorReady;
                                return ready ? "#32D74B" : "#FF9F0A";
                            }
                        }
                    }
                }
            }
        }
    }
}
