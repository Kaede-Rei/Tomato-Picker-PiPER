import QtQuick
import QtQuick.Layouts
import "." as C

Item {
    id: page

    property string title: ""
    property string subtitle: ""

    default property alias content: content_column.data

    ColumnLayout {
        anchors.fill: parent
        spacing: 12

        ColumnLayout {
            Layout.fillWidth: true
            spacing: 3

            Text {
                text: page.title
                color: C.Theme.text_primary
                font.family: C.Theme.font_stack
                font.pixelSize: 22
                font.bold: true
            }

            Text {
                text: page.subtitle
                color: C.Theme.text_secondary
                font.family: C.Theme.font_stack
                font.pixelSize: 13
                visible: page.subtitle.length > 0
            }
        }

        ColumnLayout {
            id: content_column
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 10
        }
    }
}
