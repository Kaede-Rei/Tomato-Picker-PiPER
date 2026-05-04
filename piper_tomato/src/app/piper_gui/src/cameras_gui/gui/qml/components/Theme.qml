pragma Singleton
import QtQuick

QtObject {
    readonly property color mac_blue: "#007AFF"
    readonly property color mac_blue_hover: "#0A84FF"
    readonly property color mac_blue_pressed: "#0057B8"

    readonly property color bg0: "#F5F1F8"
    readonly property color panel_bg: "#FFFFFF"
    readonly property color field_bg: "#F5F5F7"

    readonly property color text_primary: "#1D1D1F"
    readonly property color text_secondary: "#6E6E73"
    readonly property color text_tertiary: "#8E8E93"

    readonly property color glass_light: Qt.rgba(1.0, 1.0, 1.0, 0.58)
    readonly property color glass_strong: Qt.rgba(1.0, 1.0, 1.0, 0.78)
    readonly property color glass_stroke: Qt.rgba(1.0, 1.0, 1.0, 0.72)

    readonly property color dark_stage: "#050D1D"

    readonly property string font_stack:
        "San Francisco, SF Pro Text, PingFang SC, Helvetica Neue, Arial, sans-serif"

    readonly property string mono_font_stack:
        "JetBrains Mono, SF Mono, Menlo, Consolas, monospace"
}
