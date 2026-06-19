/* Copyright 2025 phine.tech GmbH

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.*/

#ifndef UE_POWER_PLUGIN_HH_
#define UE_POWER_PLUGIN_HH_

#include <QString>
#include <QTimer>
#include <fstream>
#include <ignition/gui/Plugin.hh>
#include <string>
#include <vector>

namespace phine_plugins {

class UePowerPlugin : public ignition::gui::Plugin {
    Q_OBJECT

    Q_PROPERTY(QString containerName READ getContainerName WRITE
		   setContainerName NOTIFY containerNameChanged)
    Q_PROPERTY(QString gnbIp READ getGnbIp WRITE setGnbIp NOTIFY gnbIpChanged)
    Q_PROPERTY(QString carrierFreq READ getCarrierFreq WRITE setCarrierFreq
		   NOTIFY carrierFreqChanged)
    Q_PROPERTY(
	bool processRunning READ isProcessRunning NOTIFY processRunningChanged)
    Q_PROPERTY(bool connected READ isConnected NOTIFY connectedChanged)

  public:
    UePowerPlugin();
    ~UePowerPlugin() override;
    void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    QString getContainerName() const;
    void setContainerName(const QString &name);
    QString getGnbIp() const;
    void setGnbIp(const QString &ip);
    QString getCarrierFreq() const;
    void setCarrierFreq(const QString &freq);
    bool isProcessRunning() const;
    bool isConnected() const;

  public slots:
    void toggleProcess();
    void checkAndUpdateStatus();

  signals:
    void containerNameChanged();
    void gnbIpChanged();
    void carrierFreqChanged();
    void processRunningChanged();
    void connectedChanged();

  private:
    bool checkProcessRunning();
    std::vector<int> getProcessPids() const;
    std::string buildStartCommand() const;
    static bool isValidContainerName(const std::string &name);
    static bool isValidIpAddress(const std::string &ip);
    static bool isValidFrequency(const std::string &freq);

    QString container_name{"ue_turtlebot"};
    QString gnb_ip{"192.168.70.160"};
    QString carrier_freq{"3619200000"};
    bool process_running{false};
    bool connected{false};
    QTimer *status_timer{nullptr};
    std::vector<int> tracked_pids;

    // Anti-Flicker trackers
    int skip_checks_count{0};
    int failed_checks{0};
};

} // namespace phine_plugins

#endif // UE_POWER_PLUGIN_HH_