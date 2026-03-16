import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

Rectangle {
    id: networkMonitor
    color: "#2b2b2b"
    Layout.minimumWidth: 320
    Layout.minimumHeight: 420

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 8

        // Title
        Label {
            text: "Network Performance Monitor"
            font.bold: true
            font.pixelSize: 16
            color: "#ffffff"
            Layout.alignment: Qt.AlignHCenter
        }

        // Separator
        Rectangle {
            Layout.fillWidth: true
            height: 1
            color: "#555555"
        }

        // UE IP Input
        RowLayout {
            spacing: 8
            Label {
                text: "UE IP:"
                color: "#cccccc"
                font.pixelSize: 13
                Layout.preferredWidth: 50
            }
            TextField {
                id: ueIpField
                text: NetworkMonitor.ueIp
                placeholderText: "e.g. 10.0.0.1"
                Layout.fillWidth: true
                onTextChanged: NetworkMonitor.ueIp = text
                enabled: !NetworkMonitor.running
                font.pixelSize: 13
                background: Rectangle {
                    color: ueIpField.enabled ? "#3c3c3c" : "#2a2a2a"
                    border.color: "#666666"
                    radius: 3
                }
                color: "#ffffff"
            }
        }

        // DN IP Input
        RowLayout {
            spacing: 8
            Label {
                text: "DN IP:"
                color: "#cccccc"
                font.pixelSize: 13
                Layout.preferredWidth: 50
            }
            TextField {
                id: dnIpField
                text: NetworkMonitor.dnIp
                placeholderText: "e.g. 192.168.70.135"
                Layout.fillWidth: true
                onTextChanged: NetworkMonitor.dnIp = text
                enabled: !NetworkMonitor.running
                font.pixelSize: 13
                background: Rectangle {
                    color: dnIpField.enabled ? "#3c3c3c" : "#2a2a2a"
                    border.color: "#666666"
                    radius: 3
                }
                color: "#ffffff"
            }
        }

        // Start/Stop Button
        Button {
            id: testButton
            text: NetworkMonitor.running ? "Stop Test" : "Start Test"
            Layout.fillWidth: true
            Layout.preferredHeight: 40
            font.pixelSize: 14
            font.bold: true
            onClicked: {
                if (NetworkMonitor.running)
                    NetworkMonitor.stopTest()
                else
                    NetworkMonitor.startTest()
            }
            background: Rectangle {
                color: NetworkMonitor.running ? "#cc4444" : "#44aa44"
                radius: 5
            }
            contentItem: Text {
                text: testButton.text
                font: testButton.font
                color: "#ffffff"
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
        }

        // Separator
        Rectangle {
            Layout.fillWidth: true
            height: 1
            color: "#555555"
        }

        // Results Header
        Label {
            text: "Results"
            font.bold: true
            font.pixelSize: 14
            color: "#ffffff"
        }

        // Results Grid
        GridLayout {
            columns: 2
            columnSpacing: 12
            rowSpacing: 8
            Layout.fillWidth: true

            Label {
                text: "Uplink Latency:"
                color: "#aaaaaa"
                font.pixelSize: 13
            }
            Label {
                text: NetworkMonitor.uplinkLatency
                color: "#66ff66"
                font.pixelSize: 13
                font.bold: true
            }

            Label {
                text: "Downlink Latency:"
                color: "#aaaaaa"
                font.pixelSize: 13
            }
            Label {
                text: NetworkMonitor.downlinkLatency
                color: "#66ff66"
                font.pixelSize: 13
                font.bold: true
            }

            Label {
                text: "Uplink Bandwidth:"
                color: "#aaaaaa"
                font.pixelSize: 13
            }
            Label {
                text: NetworkMonitor.uplinkBandwidth
                color: "#6699ff"
                font.pixelSize: 13
                font.bold: true
            }

            Label {
                text: "Downlink Bandwidth:"
                color: "#aaaaaa"
                font.pixelSize: 13
            }
            Label {
                text: NetworkMonitor.downlinkBandwidth
                color: "#6699ff"
                font.pixelSize: 13
                font.bold: true
            }
        }

        // Spacer
        Item {
            Layout.fillHeight: true
        }
    }
}
