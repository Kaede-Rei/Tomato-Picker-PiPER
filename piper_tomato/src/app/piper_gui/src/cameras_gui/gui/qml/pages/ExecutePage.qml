import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "../components" as C

C.PagePanel {
    id: page

    signal upsert_task_requested()
    signal execute_group_requested()
    signal cancel_requested()
    signal go_home_requested()

    title: "执行控制"
    subtitle: "写入任务、执行任务组、取消或回安全区"

    ScrollView {
        Layout.fillWidth: true
        Layout.fillHeight: true
        clip: true

        ColumnLayout {
            width: parent.width
            spacing: 12

            RowLayout {
                spacing: 12

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
