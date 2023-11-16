/*
 * Copyright (c) 2023 Particle Industries, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "dct.h"
#include "EdgePlatform.h"
#include "edge.h"
#include "edge_cellular.h"
#include "mcp_can.h"
#include "edge_location_publish.h"
#include "edge_fuelgauge.h"
#include "monitor_one_config.h"

void ctrl_request_custom_handler(ctrl_request* req)
{
    auto result = SYSTEM_ERROR_NOT_SUPPORTED;
    if (Edge::instance().isUsbCommandEnabled())
    {
        String command(req->request_data, req->request_size);
        if (CloudService::instance().dispatchCommand(command))
        {
            result = SYSTEM_ERROR_NONE;
        }
        else
        {
            result = SYSTEM_ERROR_INVALID_ARGUMENT;
        }
    }

    system_ctrl_set_result(req, result, nullptr, nullptr, nullptr);
}

void memfault_metrics_heartbeat_collect_data(void)
{
    Edge::instance().collectMemfaultHeartbeatMetrics();
}

int __attribute__((weak)) user_init()
{
    return SYSTEM_ERROR_NONE;
}

int __attribute__((weak)) user_loop()
{
    return SYSTEM_ERROR_NONE;
}

Edge *Edge::_instance = nullptr;

Edge::Edge() :
    cloudService(CloudService::instance()),
    configService(ConfigService::instance()),
    sleep(EdgeSleep::instance()),
    locationService(EdgeGnssAbstraction::instance()),
    motionService(EdgeMotion::instance()),
    location(EdgeLocation::instance()),
    motion(EdgeMotionConfiguration::instance()),
    shipping(EdgeShipping::instance()),
    _model(TRACKER_MODEL_BARE_SOM),
    _variant(0),
    _lastLoopSec(0),
    _canPowerEnabled(false),
    _pastWarnLimit(false),
    _evalTick(0),
    _lastBatteryCharging(false),
    _delayedBatteryCheck(true),
    _delayedBatteryCheckTick(0),
    _pendingChargeStatus{.uptime = 0, .state = EdgeChargeState::CHARGE_INIT},
    _chargeStatus(EdgeChargeState::CHARGE_INIT),
    _lowBatteryEvent(0),
    _evalChargingTick(0),
    _batterySafeToCharge(true),
    _forceDisableCharging(false)
{
    _cloudConfig =
    {
        .UsbCommandEnable = true,
    };
}

#ifdef EDGE_USE_MEMFAULT
void Edge::collectMemfaultHeartbeatMetrics() {
    memfault_metrics_heartbeat_set_unsigned(
        MEMFAULT_METRICS_KEY(Bat_Soc), (uint32_t)(System.batteryCharge() * _commonCfgData.memfaultBatteryScaling));

    if (_model == TRACKER_MODEL_TRACKERONE) {
        auto temperature = get_temperature();

        memfault_metrics_heartbeat_set_signed(
            MEMFAULT_METRICS_KEY(Tracker_TempC), (int32_t)(temperature * _commonCfgData.memfaultTemperatureScaling));
    } else {
        memfault_metrics_heartbeat_set_signed(
            MEMFAULT_METRICS_KEY(Tracker_TempC), (int32_t)(_commonCfgData.memfaultTemperatureInvalid * _commonCfgData.memfaultTemperatureScaling));
    }
}
#endif // EDGE_USE_MEMFAULT

int Edge::registerConfig()
{
    static ConfigObject tracker_config("tracker", {
        ConfigBool("usb_cmd", &_cloudConfig.UsbCommandEnable),
    });
    configService.registerModule(tracker_config);

    return 0;
}

int Edge::getPowerManagementConfig(hal_power_config& conf) {
    conf.size = sizeof(conf); // Size is provided for backwards compatibility
    auto err = dct_read_app_data_copy(DCT_POWER_CONFIG_OFFSET, &conf, conf.size);
    if (err) {
        return SYSTEM_ERROR_IO; // Read error
    }

    // Check if table contents are valid
    if ((conf.version == 0xff) || (conf.size == 0x00) || (conf.size == 0xff)) {
        return SYSTEM_ERROR_INVALID_STATE;
    }

    // Invert first byte of the flags to keep compatibility for HAL_POWER_PMIC_DETECTION flag
    uint32_t inverted = (~conf.flags) & 0x000000ff;
    conf.flags &= 0xffffff00;
    conf.flags |= inverted;

    return SYSTEM_ERROR_NONE;
}

int Edge::setPowerManagementConfig(const hal_power_config& conf) {
    return system_power_management_set_config(&conf, nullptr);
}

int Edge::enablePowerManagement() {
    // Gather power management configuration data that has been saved away to the DCT
    // and ensure the power managemen disable flag is clear without affecting any
    // other settings.  This will be performed inside of the device OS on later
    // versions.

    hal_power_config conf = {};
    auto err = getPowerManagementConfig(conf);

    if (err) {
        return err;
    }

    // Clear the disable flag if set but keep everything else
    if ((conf.flags & HAL_POWER_MANAGEMENT_DISABLE) == 0) {
        return SYSTEM_ERROR_NONE;
    }

    conf.flags &= ~HAL_POWER_MANAGEMENT_DISABLE;

    err = setPowerManagementConfig(conf);

    return err;
}

void Edge::enableIoCanPower(bool enable)
{
    digitalWrite(MCP_CAN_PWR_EN_PIN, (_canPowerEnabled = enable) ? HIGH : LOW);
}

int Edge::initEsp32()
{
    // ESP32 related GPIO
#if (PLATFORM_ID == PLATFORM_TRACKER)
    pinMode(ESP32_BOOT_MODE_PIN, OUTPUT);
    digitalWrite(ESP32_BOOT_MODE_PIN, HIGH);
    pinMode(ESP32_PWR_EN_PIN, OUTPUT);
    digitalWrite(ESP32_PWR_EN_PIN, LOW); // power off device, first power off for ESP32 workaround for low power
    delay(50); // ESP32 workaround for low power
    digitalWrite(ESP32_PWR_EN_PIN, HIGH); // power on device, ESP32 workaround for low power
    delay(50); // ESP32 workaround for low power
    digitalWrite(ESP32_PWR_EN_PIN, LOW); // power off device
    pinMode(ESP32_CS_PIN, OUTPUT);
    digitalWrite(ESP32_CS_PIN, HIGH);
#endif
    return SYSTEM_ERROR_NONE;
}

int Edge::initCan()
{
    // CAN related GPIO
    pinMode(MCP_CAN_STBY_PIN, OUTPUT);
    digitalWrite(MCP_CAN_STBY_PIN, LOW);
    pinMode(MCP_CAN_PWR_EN_PIN, OUTPUT);
    // Do not power the CAN interface on yet
    pinMode(MCP_CAN_RESETN_PIN, OUTPUT);
    digitalWrite(MCP_CAN_RESETN_PIN, HIGH);
    pinMode(MCP_CAN_INT_PIN, INPUT_PULLUP);
    pinMode(MCP_CAN_CS_PIN, OUTPUT);
    digitalWrite(MCP_CAN_CS_PIN, HIGH);

    // Reset CAN transceiver
    digitalWrite(MCP_CAN_RESETN_PIN, LOW);
    delay(50);
    digitalWrite(MCP_CAN_RESETN_PIN, HIGH);
    delay(50);

    digitalWrite(MCP_CAN_STBY_PIN, HIGH);

    // Initialize CAN device driver
    MCP_CAN can(MCP_CAN_CS_PIN, MCP_CAN_SPI_INTERFACE);
    if (can.minimalInit() != CAN_OK)
    {
        Log.error("CAN init failed");
    }

    if (_deviceConfig.enableIoCanPower()) {
        enableIoCanPower(true);
    }

    return SYSTEM_ERROR_NONE;
}

int Edge::initIo()
{
    // Initialize basic Tracker GPIO to known inactive values until they are needed later
    (void)initEsp32();
    (void)initCan();

    return SYSTEM_ERROR_NONE;
}

void Edge::enableWatchdog(bool enable) {
#ifndef RTC_WDT_DISABLE
    if (enable) {
        // watchdog at 1 minute
        hal_exrtc_enable_watchdog(_commonCfgData.watchdogExpireTime, nullptr);
        hal_exrtc_feed_watchdog(nullptr);
    }
    else {
        hal_exrtc_disable_watchdog(nullptr);
    }
#else
    (void)enable;
#endif // RTC_WDT_DISABLE
}

void Edge::startShippingMode() {
    // Always let the sleep framework manage dependencies on power state changes
    sleep.forceShutdown();
}

int Edge::prepareSleep() {
    if (_deviceConfig.enableIoCanPowerSleep()) {
        enableIoCanPower(false);
    }
    return SYSTEM_ERROR_NONE;
}

int Edge::prepareWake() {
    if (_deviceConfig.enableIoCanPowerSleep() && _deviceConfig.enableIoCanPower()) {
        enableIoCanPower(true);
    }
    return SYSTEM_ERROR_NONE;
}

void Edge::startLowBatteryShippingMode() {
    if (sleep.isForcedShutdownPending()) {
        return;
    }

    // Publish then shutdown
    sleep.forcePublishVitals();
    location.triggerLocPub(Trigger::IMMEDIATE,"batt_low");
    startShippingMode();
}

void Edge::lowBatteryHandler(system_event_t event, int data) {
    Edge::instance()._lowBatteryEvent = System.uptime();
}

EdgeChargeState Edge::batteryDecode(battery_state_t state) {
    auto chargeStatus = EdgeChargeState::CHARGE_INIT;

    switch (state) {
        case BATTERY_STATE_UNKNOWN:
        // Fall through
        case BATTERY_STATE_FAULT:
        // Fall through
        case BATTERY_STATE_NOT_CHARGING:
        // Fall through
        case BATTERY_STATE_DISCHARGING: {
            chargeStatus = EdgeChargeState::CHARGE_CARE;
            break;
        }

        case BATTERY_STATE_CHARGING:
        // Fall through
        case BATTERY_STATE_CHARGED:
        // Fall through
        case BATTERY_STATE_DISCONNECTED: {
            chargeStatus = EdgeChargeState::CHARGE_DONT_CARE;
            break;
        }
    }

    return chargeStatus;
}

void Edge::setPendingChargeStatus(unsigned int uptime, EdgeChargeState state) {
    const std::lock_guard<Mutex> lock(_pendingLock);
    _pendingChargeStatus.uptime = uptime;
    _pendingChargeStatus.state = state;
}

EdgeChargeStatus Edge::getPendingChargeStatus() {
    const std::lock_guard<Mutex> lock(_pendingLock);
    return _pendingChargeStatus;
}

void Edge::batteryStateHandler(system_event_t event, int data) {
    auto currentChargeStatus = Edge::instance().batteryDecode(static_cast<battery_state_t>(data));

    Edge::instance().setPendingChargeStatus(System.uptime(), currentChargeStatus);
}


void Edge::initBatteryMonitor() {
    auto powerConfig = System.getPowerConfiguration();
    // Start battery charging at low current state from boot then increase if necessary
    if ((powerConfig.batteryChargeCurrent() != _commonCfgData.chargeCurrentHigh) ||
        (powerConfig.powerSourceMaxCurrent() != _commonCfgData.inputCurrent)) {

        powerConfig.batteryChargeCurrent(_commonCfgData.chargeCurrentHigh);
        powerConfig.powerSourceMaxCurrent(_commonCfgData.inputCurrent);
        System.setPowerConfiguration(powerConfig);
    }

    // Keep a handy variable to check on battery charge enablement
    _batterySafeToCharge = !powerConfig.isFeatureSet(SystemPowerFeature::DISABLE_CHARGING);

    // To initialize the fuel gauge so that it provides semi-accurate readings we
    // want to ensure that the charging circuit is off when providing the
    // fuel gauge quick start command.
    // In order to disable charging safely we want to enable the PMIC watchdog so that
    // if anything happens during the procedure that the circuit can return to
    // normal operation in the event the MCU doesn't complete.

    EdgeFuelGauge::instance().init();
    {
        PMIC pmic(true); // Acquire lock
        FuelGauge fuelGauge;

        pmic.setWatchdog(0x1); // 40 seconds
        pmic.disableCharging();
        // Delay so that the bulk capacitance and battery can equalize
        delay(_commonCfgData.postChargeSettleTime);

        fuelGauge.quickStart();
        // Must delay at least 175ms after quickstart, before calling
        // getSoC(), or reading will not have updated yet.
        delay(200);

        _forceDisableCharging = _deviceConfig.disableCharging();
        if (_batterySafeToCharge && !_forceDisableCharging) {
            pmic.enableCharging();
        }
        pmic.disableWatchdog();
    }
}

bool Edge::getChargeEnabled() {
    return !System.getPowerConfiguration().isFeatureSet(SystemPowerFeature::DISABLE_CHARGING);
}

void Edge::evaluateBatteryCharge() {
    // This is delayed intialization for the fuel gauge threshold since power on
    // events may glitch between battery states easily.
    if (_delayedBatteryCheck) {
        if (System.uptime() >= _commonCfgData.lowBatteryStartTime) {
            _delayedBatteryCheck = false;
            FuelGauge fuelGauge;

            // Set the alert level for <SET VALUE> - 1%.  This value will not be normalized but rather the raw
            // threshold value provided by the fuel gauge.
            // The fuel gauge will only give an alert when passing through this limit with decreasing
            // succesive charge amounts.  It is important to check whether we are already below this limit
            fuelGauge.setAlertThreshold((uint8_t)(_commonCfgData.lowBatteryCutoff - _commonCfgData.lowBatteryCutoffCorrection));
            fuelGauge.clearAlert();
            delay(100);

            // NOTE: This is a workaround in case the fuel gauge interrupt is not configured as an input
            pinMode(LOW_BAT_UC, INPUT_PULLUP);

            System.on(low_battery, lowBatteryHandler);
            System.on(battery_state, batteryStateHandler);
            if (_chargeStatus == EdgeChargeState::CHARGE_INIT) {
                setPendingChargeStatus(System.uptime(), batteryDecode(static_cast<battery_state_t>(System.batteryState())));
            }
        }
    }

    // Debounce the charge status here by looking at data collected by the interrupt handler and making sure that the
    // last state is present over a qualified amount of time.
    auto status = getPendingChargeStatus();
    if (status.uptime && ((System.uptime() - status.uptime) >= _commonCfgData.lowBatteryDebounceTime)) {
        _chargeStatus = status.state;
        setPendingChargeStatus(0, _chargeStatus);
        _evalTick = System.uptime();
    }

    // No further work necessary if we are still in the delayed battery check interval or not on a evaluation interval
    unsigned int evalLoopInterval = sleep.isSleepDisabled() ? _commonCfgData.lowBatteryAwakeEvalInterval : _commonCfgData.lowBatterySleepEvalInterval;
    if (_delayedBatteryCheck ||
        (System.uptime() - _evalTick < evalLoopInterval)) {

        return;
    }

    _evalTick = System.uptime();

    auto stateOfCharge = System.batteryCharge();

    // Skip errors
    if (stateOfCharge < 0.0) {
        Log.info("Battery charge reporting error");
        return;
    }

    switch (_chargeStatus) {
        case EdgeChargeState::CHARGE_CARE: {
            if (_lowBatteryEvent || (stateOfCharge <= (float)_commonCfgData.lowBatteryCutoff)) {
                // Publish then shutdown
                Log.error("Battery charge of %0.1f%% is less than limit of %0.1f%%.  Entering shipping mode", stateOfCharge, (float)_commonCfgData.lowBatteryCutoff);
                startLowBatteryShippingMode();
            }
            else if (!_pastWarnLimit && (stateOfCharge <= (float)_commonCfgData.lowBatteryWarning)) {
                _pastWarnLimit = true;
                // Publish once when falling through this value
                Particle.publishVitals();
                location.triggerLocPub(Trigger::IMMEDIATE,"batt_warn");
                Log.warn("Battery charge of %0.1f%% is less than limit of %0.1f%%.  Publishing warning", stateOfCharge, (float)_commonCfgData.lowBatteryWarning);
            }
            break;
        }

        case EdgeChargeState::CHARGE_DONT_CARE: {
            // There may be instances where the device is being charged but the battery is still being discharged
            if (_lowBatteryEvent) {
                // Publish then shutdown
                Log.error("Battery charge of %0.1f%% is less than limit of %0.1f%%.  Entering shipping mode", stateOfCharge, (float)_commonCfgData.lowBatteryCutoff);
                startLowBatteryShippingMode();
            }
            else if (_pastWarnLimit && (stateOfCharge >= (float)(_commonCfgData.lowBatteryWarning + _commonCfgData.lowBatteryWarningHyst))) {
                _pastWarnLimit = false;
                // Publish again to announce that we are out of low battery warning
                Particle.publishVitals();
            }
        }
    }
}

void Edge::onSleepPrepare(EdgeSleepContext context)
{
    configService.flush();
    switch (_model) {
        case TRACKER_MODEL_TRACKERONE:
        case TRACKER_MODEL_EVAL:
        // Fall through
        case TRACKER_MODEL_MONITORONE: {
            EdgeSleep::instance().wakeAtSeconds(System.uptime() + _commonCfgData.lowBatterySleepWakeInterval);
        }
        break;
    }
}

void Edge::onSleep(EdgeSleepContext context)
{
    switch (_model) {
        case TRACKER_MODEL_TRACKERONE:
        case TRACKER_MODEL_MONITORONE: {
            GnssLedEnable(false);
        }
        break;
    }
}

void Edge::onWake(EdgeSleepContext context)
{
    switch (_model) {
        case TRACKER_MODEL_TRACKERONE:
        case TRACKER_MODEL_MONITORONE: {
            GnssLedEnable(true);
            // Ensure battery evaluation starts immediately after waking
            _evalTick = 0;
        }
        break;
    }
}

void Edge::onSleepStateChange(EdgeSleepContext context)
{
    if (context.reason == EdgeSleepReason::STATE_TO_SHUTDOWN) {
        // Consider any device shutdown here
    }
}

void Edge::otaHandler(system_event_t event, int param) {
    switch ((unsigned int)param) {
        case SystemEventsParam::firmware_update_complete: {
            // There will be an imminent system reset so disable the watchdog
            enableWatchdog(false);
        }
        break;

        case SystemEventsParam::firmware_update_begin: {
            if (!sleep.isSleepDisabled()) {
                // Don't allow the device to go asleep if an OTA has begun
                sleep.pauseSleep();
            }
        }
        break;

        case SystemEventsParam::firmware_update_failed: {
            if (!sleep.isSleepDisabled()) {
                // Allow the device to go asleep after a chance for the cloud to restart a failed OTA
                sleep.extendExecutionFromNow(_commonCfgData.failedOtaKeepAwake);
                sleep.resumeSleep();
            }
        }
        break;

        default:
            break;
    }
}

void Edge::startup()
{
    // Correct power manager states in the DCT
    enablePowerManagement();
}

int Edge::init()
{
    int ret = 0;

    _lastLoopSec = System.uptime();

    // Disable OTA updates until after the system handler has been registered
    System.disableUpdates();

#ifdef EDGE_USE_MEMFAULT
    if (nullptr == _memfault) {
        _memfault = new Memfault(EDGE_PRODUCT_VERSION);
    }
#endif // EDGE_USE_MEMFAULT

#ifndef TRACKER_MODEL_NUMBER
    ret = hal_get_device_hw_model(&_model, &_variant, nullptr);
    if (ret)
    {
        Log.error("Failed to read device model and variant");
    }
    else
    {
        Log.info("Tracker model = %04lX, variant = %04lX", _model, _variant);
    }
#else
    _model = TRACKER_MODEL_NUMBER;
#ifdef TRACKER_MODEL_VARIANT
    _variant = TRACKER_MODEL_VARIANT;
#else
    _variant = 0;
#endif // TRACKER_MODEL_VARIANT
#endif // TRACKER_MODEL_NUMBER

    EdgePlatform::instance().init();
    switch (EdgePlatform::instance().getModel()) {
        case EdgePlatform::TrackerModel::eMONITOR_ONE:
            _platformConfig = new MonitorOneConfiguration();
            _commonCfgData = _platformConfig->get_common_config_data();
            break;
    }

    // Initialize unused interfaces and pins
    (void)initIo();

    // Perform IO setup specific to Tracker One.  Reset the fuel gauge state-of-charge, check if under thresholds.
    BLE.selectAntenna(BleAntennaType::EXTERNAL);
    initBatteryMonitor();

    cloudService.init();

    configService.init();

    // Setup device monitoring configuration here
    static ConfigObject deviceMonitoringDesc
    (
        "monitoring",
        {
            ConfigBool("device_monitor", &_deviceMonitoring)
        }
    );

    configService.registerModule(deviceMonitoringDesc);

    sleep.init([this](bool enable){ this->enableWatchdog(enable); });
    sleep.registerSleepPrepare([this](EdgeSleepContext context){ this->onSleepPrepare(context); });
    sleep.registerSleep([this](EdgeSleepContext context){ this->onSleep(context); });
    sleep.registerWake([this](EdgeSleepContext context){ this->onWake(context); });
    sleep.registerStateChange([this](EdgeSleepContext context){ this->onSleepStateChange(context); });

    // Register our own configuration settings
    registerConfig();

    ret = locationService.begin(_deviceConfig.locationServiceConfig());
    if (ret)
    {
        Log.error("Failed to begin location service");
    }

    // Check for Monitor One hardware
    (void)GnssLedInit(_commonCfgData.pGnssLed);
    GnssLedEnable(true);
    switch (_model) {
        case TRACKER_MODEL_TRACKERONE: {
            temperature_init(TRACKER_THERMISTOR,
                [this](TemperatureChargeEvent event){ return chargeCallback(event); }
            );
        }
        break;
        case TRACKER_MODEL_MONITORONE: {
            temperature_init(MONITORONE_THERMISTOR,
                [this](TemperatureChargeEvent event){ return chargeCallback(event); }
            );
        }
        break;
    }

    motionService.start();

    location.init(_deviceConfig.gnssRetryCount());

    motion.init();

    shipping.init();
    shipping.regShutdownBeginCallback(std::bind(&Edge::stop, this));
    shipping.regShutdownIoCallback(std::bind(&Edge::end, this));
    shipping.regShutdownFinalCallback(
        [this](){
            enableWatchdog(false);
            return 0;
        });

    enableWatchdog(true);

    EdgeLocationPublish::instance().init();

    // Associate handler to OTAs and pending resets to disable the watchdog
    System.on(reset_pending,
        [this](system_event_t event, int param){
            // Stop everything
            stop();
            end();
        }
    );
    System.on(firmware_update, [this](system_event_t event, int param){ otaHandler(event, param); });

    // Allow OTAs now that the firmware update handlers are registered
    System.enableUpdates();

    location.regLocGenCallback(loc_gen_cb);

    _platformConfig->load_specific_platform_config();

    // User code can be initialized here
    user_init();

    return SYSTEM_ERROR_NONE;
}

void Edge::loop()
{
    if (_platformConfig == nullptr)
    {
        return;
    }

    uint32_t cur_sec = System.uptime();

    // slow operations for once a second
    if(_lastLoopSec != cur_sec)
    {
        _lastLoopSec = cur_sec;

#ifndef RTC_WDT_DISABLE
        hal_exrtc_feed_watchdog(nullptr);
#endif
    }

    EdgeFuelGauge::instance().loop();

    // Evaluate low battery conditions
    switch (_model) {
        case TRACKER_MODEL_TRACKERONE:
        // Fall through
        case TRACKER_MODEL_MONITORONE:{
            evaluateBatteryCharge();
        }
        // Fall through
        case TRACKER_MODEL_EVAL:
        break;
    }

    // fast operations for every loop
    sleep.loop();
    motion.loop();

    // Check for temperature enabled hardware
    switch (_model) {
        case TRACKER_MODEL_TRACKERONE:
        // Fall through
        case TRACKER_MODEL_MONITORONE: {
            temperature_tick();

            if (temperature_high_events())
            {
                location.triggerLocPub(Trigger::NORMAL,"temp_h");
            }

            if (temperature_low_events())
            {
                location.triggerLocPub(Trigger::NORMAL,"temp_l");
            }
        }
        break;
    }


    // fast operations for every loop
    cloudService.tick();
    configService.tick();
 #ifdef EDGE_USE_MEMFAULT
    if (_deviceMonitoring && (nullptr != _memfault)) {
        _memfault->process();
    }
#endif // EDGE_USE_MEMFAULT
    location.loop();

    // Execute a user defined loop here
    user_loop();
}

int Edge::stop() {
    locationService.stop();
    motionService.stop();

    return SYSTEM_ERROR_NONE;
}

int Edge::end() {
    enableIoCanPower(false);
    GnssLedEnable(false);
    enableWatchdog(false);
    return SYSTEM_ERROR_NONE;
}

int Edge::reset() {
    stop();
    end();
    System.reset();

    return SYSTEM_ERROR_NONE;
}

int Edge::pmicEnableCharging() {
    auto powerConfig = System.getPowerConfiguration();
    if (powerConfig.isFeatureSet(SystemPowerFeature::DISABLE_CHARGING)) {
        powerConfig.clearFeature(SystemPowerFeature::DISABLE_CHARGING);
        return System.setPowerConfiguration(powerConfig);
    }

    return SYSTEM_ERROR_NONE;
}

int Edge::pmicDisableCharging() {
    auto powerConfig = System.getPowerConfiguration();
    if (!powerConfig.isFeatureSet(SystemPowerFeature::DISABLE_CHARGING)) {
        powerConfig.feature(SystemPowerFeature::DISABLE_CHARGING);
        return System.setPowerConfiguration(powerConfig);
    }

    return SYSTEM_ERROR_NONE;
}

void Edge::forceDisableCharging(bool value) {
    _forceDisableCharging = value;

    if (_forceDisableCharging) {
        pmicDisableCharging();
    }
    else {
        if (_batterySafeToCharge) {
            pmicEnableCharging();
        }
    }
}

int Edge::setChargeCurrent(uint16_t current) {
    int ret = SYSTEM_ERROR_NONE;
    auto powerConfig = System.getPowerConfiguration();
    if (powerConfig.batteryChargeCurrent() != current) {
        powerConfig.batteryChargeCurrent(current);
        ret = System.setPowerConfiguration(powerConfig);
    }
    return ret;
}

int Edge::chargeCallback(TemperatureChargeEvent event) {
    auto shouldCharge = true;

    switch (event) {
        case TemperatureChargeEvent::NORMAL: {
            setChargeCurrent(_commonCfgData.chargeCurrentHigh);
            shouldCharge = true;
            break;
        }

        case TemperatureChargeEvent::OVER_CHARGE_REDUCTION: {
            setChargeCurrent(_commonCfgData.chargeCurrentLow);
            shouldCharge = true;
            break;
        }

        case TemperatureChargeEvent::OVER_TEMPERATURE: {
            setChargeCurrent(_commonCfgData.chargeCurrentLow);
            shouldCharge = false;
            break;
        }

        case TemperatureChargeEvent::UNDER_TEMPERATURE: {
            setChargeCurrent(_commonCfgData.chargeCurrentLow);
            shouldCharge = false;
            break;
        }
    }

    // Check if anything needs to be changed for charging
    if (!shouldCharge && _batterySafeToCharge) {
        _batterySafeToCharge = false;
        pmicDisableCharging();
    }
    else if (shouldCharge && !_batterySafeToCharge) {
        _batterySafeToCharge = true;
        if (!_forceDisableCharging) {
            pmicEnableCharging();
        }
    }

    return SYSTEM_ERROR_NONE;
}

void Edge::loc_gen_cb(JSONWriter& writer, LocationPoint &loc, const void *context)
{

    if(EdgeLocation::instance().getMinPublish())
    {
        // only add additional fields when not on minimal publish
        return;
    }

    // add cellular signal strength if available
    CellularSignal signal;
    if(!EdgeCellular::instance().getSignal(signal))
    {
        writer.name("cell").value(signal.getStrength(), 1);
    }

    // add lipo battery charge if available
    int bat_state = System.batteryState();
    if(bat_state == BATTERY_STATE_NOT_CHARGING ||
        bat_state == BATTERY_STATE_CHARGING ||
        bat_state == BATTERY_STATE_DISCHARGING ||
        bat_state == BATTERY_STATE_CHARGED)
    {
        float bat = System.batteryCharge();
        if(bat >= 0 && bat <= 100)
        {
            writer.name("batt").value(bat, 1);
        }
    }

    // Check for Tracker One hardware
    switch (Edge::instance().getModel()) {
        case TRACKER_MODEL_TRACKERONE:
        case TRACKER_MODEL_TRACKERM:
        // Fall through
        case TRACKER_MODEL_MONITORONE: {
            writer.name("temp").value(get_temperature(), 1);
        }
        break;
    }
}
