import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

Rectangle {
    id: uePower
    color: "#2b2b2b"
    Layout.minimumWidth: 400
    Layout.minimumHeight: 320
    Layout.fillWidth: true
    anchors.fill: parent

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 8

        // Title
        Label {
            text: "UE Power Control"
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
                text: UePowerPlugin.containerName
                placeholderText: "e.g. ue_turtlebot"
                Layout.fillWidth: true
                onTextChanged: UePowerPlugin.containerName = text
                enabled: !UePowerPlugin.connected
                font.pixelSize: 13
                background: Rectangle {
                    color: containerField.enabled ? "#3c3c3c" : "#2a2a2a"
                    border.color: "#666666"
                    radius: 3
                }
                color: "#ffffff"
            }
        }

        // gNB IP Input
        RowLayout {
            spacing: 8
            Label {
                text: "gNB IP:"
                color: "#cccccc"
                font.pixelSize: 13
                Layout.preferredWidth: 80
            }
            TextField {
                id: gnbIpField
                text: UePowerPlugin.gnbIp
                placeholderText: "e.g. 192.168.70.160"
                Layout.fillWidth: true
                onTextChanged: UePowerPlugin.gnbIp = text
                enabled: !UePowerPlugin.connected
                font.pixelSize: 13
                background: Rectangle {
                    color: gnbIpField.enabled ? "#3c3c3c" : "#2a2a2a"
                    border.color: "#666666"
                    radius: 3
                }
                color: "#ffffff"
            }
        }

        // Carrier Frequency Input
        RowLayout {
            spacing: 8
            Label {
                text: "Carrier Freq:"
                color: "#cccccc"
                font.pixelSize: 13
                Layout.preferredWidth: 80
            }
            TextField {
                id: carrierFreqField
                text: UePowerPlugin.carrierFreq
                placeholderText: "e.g. 3619200000"
                Layout.fillWidth: true
                onTextChanged: UePowerPlugin.carrierFreq = text
                enabled: !UePowerPlugin.connected
                font.pixelSize: 13
                background: Rectangle {
                    color: carrierFreqField.enabled ? "#3c3c3c" : "#2a2a2a"
                    border.color: "#666666"
                    radius: 3
                }
                color: "#ffffff"
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
            text: !UePowerPlugin.connected ? "Check Status" :
                  (UePowerPlugin.processRunning ? "Turn OFF" : "Turn ON")
            Layout.fillWidth: true
            Layout.preferredHeight: 48
            font.pixelSize: 16
            font.bold: true
            onClicked: UePowerPlugin.toggleProcess()

            background: Rectangle {
                color: !UePowerPlugin.connected ? "#666666" :
                       (UePowerPlugin.processRunning ? "#44aa44" : "#cc4444")
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
