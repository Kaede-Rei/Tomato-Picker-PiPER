import QtQuick
import "." as C

Item {
    id: page

    property string title: ""
    property string subtitle: ""

    default property alias content: body.data

    Text {
        id: title_text
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        text: page.title
        color: C.Theme.text_primary
        font.family: C.Theme.font_stack
        font.pixelSize: 22
        font.bold: true
        elide: Text.ElideRight
    }

    Text {
        id: subtitle_text
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: title_text.bottom
        anchors.topMargin: 4
        text: page.subtitle
        color: C.Theme.text_secondary
        font.family: C.Theme.font_stack
        font.pixelSize: 13
        visible: page.subtitle.length > 0
        elide: Text.ElideRight
    }

    Item {
        id: body
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: subtitle_text.visible ? subtitle_text.bottom : title_text.bottom
        anchors.topMargin: 14
        anchors.bottom: parent.bottom
        clip: true
    }
}
