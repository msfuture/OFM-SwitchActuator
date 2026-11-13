#pragma once
#include "OpenKNX.h"
#include "StatusOutput.h"
#ifdef OPENKNX_SWA_BL0942_SPI
  #include "BL0942.h"

  #define OPENKNX_SWA_BL0942_INIT_DELAY 5000 // wait 5s to let capacitors charge first
  #define OPENKNX_SWA_BL0942_LOOP_DELAY 100

  // below this apparent power the power factor is meaningless (no load / noise) and reported as 0
  #define OPENKNX_SWA_POWER_FACTOR_MIN_VA 1.0f
  // plausibility limit for the energy increase between two BL0942 readings (in kWh); larger jumps
  // (e.g. after a chip reset or a corrupt frame) are ignored and the counter baseline is re-taken
  #define OPENKNX_SWA_ENERGY_MAX_STEP_KWH 0.05f
#endif

#define SWITCH_DEBOUNCE 250

class SwitchActuatorChannel : public OpenKNX::Channel
{
  private:
    const std::string name() override;
    bool statusDuringLock;
    uint32_t statusCyclicSendTimer = 0;
    uint32_t relayBistableImpulsTimer = 0;
    uint32_t turnOnDelayTimer = 0;
    uint32_t turnOffDelayTimer = 0;
    uint32_t switchLastTrigger = 0;

    void doSwitchInternal(bool active, bool syncSwitch = true);
    void processSwitchInput(bool newActive);
    void processLockInput(bool newActive);
    void processScene(uint8_t sceneNumber, bool learn);
    void relaisOff();

#ifdef OPENKNX_SWA_BL0942_SPI
    bl0942::BL0942 bl0942 = bl0942::BL0942(OPENKNX_SWA_BL0942_SPI);
    bl0942::SensorData lastDataReceived;

    bool bl0942Initialized = false;
    uint32_t bl0942StartupDelay = 0;
    uint32_t bl0942UpdateTimer = 0;
    SwaStatus::ValueState _statusPower;
    SwaStatus::ValueState _statusCurrent;
    SwaStatus::ValueState _statusVoltage;
    SwaStatus::ValueState _statusApparentPower;
    SwaStatus::ValueState _statusPowerFactor;
    SwaStatus::ValueState _statusEnergy;

    float _apparentPower = 0.0f; // VA
    float _powerFactor = 0.0f;   // -1..1, sign follows active power direction

    double _energyWh = 0.0;          // persistent energy counter (Wh)
    float _energyChipLastKWh = 0.0f; // last raw BL0942 energy reading (kWh since chip start)
    bool _energyChipValid = false;   // _energyChipLastKWh holds a usable baseline

    uint32_t _debugTimer = 0;

    void dataReceivedBl0942(bl0942::SensorData &data);
    void setChannelSelectorBl0942(bool active);
    void initBl0942();
    void updateEnergy(float chipEnergyKWh);
#endif

  protected:

  public:
    SwitchActuatorChannel(uint8_t iChannelNumber);
    ~SwitchActuatorChannel();

    void processInputKo(GroupObject &iKo) override;
    void setup(bool configured) override;
    void loop() override;

    void doSwitch(bool active, bool syncSwitch = true);
    bool isRelayActive();
    void savePower();
    bool restorePower();

#ifdef OPENKNX_SWA_BL0942_SPI
    float getPower();
    float getCurrent();
    double getEnergyWh();
    void setEnergyWh(double energyWh);
    void resetEnergy();
#endif
};