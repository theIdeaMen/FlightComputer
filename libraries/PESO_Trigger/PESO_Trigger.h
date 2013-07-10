#ifndef __PESO_TRIGGER__
#define __PESO_TRIGGER__

#include <arduino.h>

class Trigger
{
  private:
    int _pin;
    bool _state;
    double _on;
    double _off;
    bool _disabled;
    
    int _onDir;
    int _offDir;

    void(*onCB)();
    void(*offCB)();
  
  public:

    enum { ABOVE, BELOW };

    Trigger();
    Trigger(int pin, double onVal=0, int onDir=0, double offVal=0, int offDir=0, bool state=false);
    void enable();
    void disable();
    void initialize(int pin, double onVal=0, int onDir=0, double offVal=0, int offDir=0, bool state=false);
    void on();
    void off();
    bool getState();
    int getPin();
    void update(double val);
    void update(double onVal, double offVal);
    void onCallBack(void(*func)());
    void offCallBack(void(*func)());

};


#endif __PESO_TRIGGER__

