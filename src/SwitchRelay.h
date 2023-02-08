#ifndef SWITCH_RELAY_H
#define SWITCH_RELAY_H

#include <Arduino.h>

class SwitchRelay {
  public:
    enum SwitchState { 
      Off,
      On
    };

    void setOn() {
      setState(On);
    }

    void setOff() {
      setState(Off);
    }

    virtual SwitchState getState();
    virtual void setState(SwitchState targetState);

    void set(bool targetState) {
      this->setState(targetState ? SwitchState::On : SwitchState::Off);
    }
};

class SwitchRelayPin : public SwitchRelay {
  public:
    SwitchRelayPin(uint8_t pin) : SwitchRelayPin(pin, 1)
    { }

    SwitchRelayPin(uint8_t pin, uint8_t onValue, uint8_t pinModeType = OUTPUT)
      : pin(pin), onValue(onValue), offValue(onValue ? 0 : 1)
    { 
      pinMode(pin, pinModeType);
      setState(state);
    }

    virtual SwitchState getState() {
      return state;
    }

    virtual void setState(SwitchState targetState) {
      digitalWrite(pin, targetState == On ? onValue : offValue);
      state = targetState;
    }
  
  private:
    const uint8_t pin, onValue, offValue;
    SwitchState state = Off;
};

class SwitchRelayMock : public SwitchRelay {
  public:
    SwitchRelayMock(SwitchState state = Off) : state(state) 
    { }

    virtual SwitchState getState() {
      return state;
    }

    virtual void setState(SwitchState targetState)
    { }

  private:
    SwitchState state = Off;
};

#endif