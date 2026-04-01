import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

Rectangle {
    id: gnbPower
    color: "#2b2b2b"
    Layout.minimumWidth: 400
    Layout.minimumHeight: 220
    Layout.fillWidth: true
    anchors.fill: parent

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 8

        // Title
        Label {
            text: "gNB Power Control"
            font.bold: true
            font.pixelSize: 16
            color: "#ffffff"
            Layout.alignment: Qt.AlignHCenter
        }

        Rectangle {
            Layout.fillWidth: true
            height: 1
            color: "#555555"
        }

        // Container Name
        RowLayout {
            spacing: 8
            Label {
                text: "Container:"
                color: "#cccccc"
                font.pixelSize: 13
                Layout.preferredWidth: 80
            }
            TextField {
                id: containerField
                text: gNBPowerPlugin.containerName
                placeholderText: "e.g. oai-gNB1"
                Layout.fillWidth: true
                onTextChanged: gNBPowerPlugin.containerName = text
                enabled: !gNBPowerPlugin.connected
                font.pixelSize: 13
                background: Rectangle {
                    color: containerField.enabled ? "#3c3c3c" : "#2a2a2a"
                    border.color: "#666666"
                    radius: 3
                }
                color: "#ffffff"
            }
        }

        // Version Selector
        RowLayout {
            spacing: 8
            Label {
                text: "Version:"
                color: "#cccccc"
                font.pixelSize: 13
                Layout.preferredWidth: 80
            }
            ComboBox {
                id: versionCombo
                model: ["v24 (.conf)", "v26 (.yaml)"]
                currentIndex: gNBPowerPlugin.versionIndex
                onCurrentIndexChanged: gNBPowerPlugin.versionIndex = currentIndex
                enabled: !gNBPowerPlugin.connected
                Layout.fillWidth: true
                font.pixelSize: 13

                background: Rectangle {
                    color: versionCombo.enabled ? "#3c3c3c" : "#2a2a2a"
                    border.color: "#666666"
                    radius: 3
                }
                contentItem: Text {
                    text: versionCombo.displayText
                    color: "#ffffff"
                    font: versionCombo.font
                    verticalAlignment: Text.AlignVCenter
                    leftPadding: 8
                }
            }
        }

        Rectangle {
            Layout.fillWidth: true
            height: 1
            color: "#555555"
        }

        // Power Toggle Button
        Button {
            id: powerButton
            text: !gNBPowerPlugin.connected ? "Check Status" :
                  (gNBPowerPlugin.processRunning ? "Turn OFF" : "Turn ON")
            Layout.fillWidth: true
            Layout.preferredHeight: 48
            font.pixelSize: 16
            font.bold: true
            onClicked: gNBPowerPlugin.toggleProcess()

            background: Rectangle {
                color: !gNBPowerPlugin.connected ? "#666666" :
                       (gNBPowerPlugin.processRunning ? "#44aa44" : "#cc4444")
                radius: 6
            }
            contentItem: Text {
                text: powerButton.text
                font: powerButton.font
                color: "#ffffff"
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
        }

        Item {
            Layout.fillHeight: true
        }
    }
}
