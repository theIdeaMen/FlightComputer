#include <PESO_Trigger.h>

Trigger::Trigger()
{
  initialize(13);
}

Trigger::Trigger(int pin, double onVal, int onDir, double offVal, int offDir, bool state)
{
  initialize(pin, onVal, onDir, offVal, offDir, state);
}

void Trigger::initialize(int pin, double onVal, int onDir, double offVal, int offDir, bool state)
{
  _pin = pin;
  _state = state;
  _on = onVal;
  _off = offVal;
  _onDir = onDir;
  _offDir = offDir;
  _disabled = false;

  pinMode(_pin, OUTPUT); 
  _state ? on() : off();

  onCB = NULL;
  offCB = NULL;

}

void Trigger::on()
{
  digitalWrite(_pin, HIGH);
  _state = true;
  if (onCB) onCB();
}

void Trigger::off()
{
  digitalWrite(_pin, LOW);
  _state = false;
  if (offCB) offCB();
}

bool Trigger::getState()
{
  return _state;
}

int Trigger::getPin()
{
  return _pin;
}

void Trigger::enable()
{
  _disabled = false;
}

void Trigger::disable()
{
  _disabled = true;
}

void Trigger::update(double val)
{
  if (_disabled) return;

  if (!_state && _onDir == ABOVE && val > _on) { on(); return; }
  if (!_state && _onDir == BELOW && val < _on) { on(); return; }

  if (_state && _offDir == ABOVE && val > _off) { off(); return; }
  if (_state && _offDir == BELOW && val < _off) { off(); return; }
}

void Trigger::update(double onVal, double offVal)
{
  if (!_state && _onDir == ABOVE && onVal > _on) { on(); return; }
  if (!_state && _onDir == BELOW && onVal < _on) { on(); return; }

  if (_state && _offDir == ABOVE && offVal > _off) { off(); return; }
  if (_state && _offDir == BELOW && offVal < _off) { off(); return; }
}

void Trigger::onCallBack(void(*func)())
{
  onCB = func;
}

void Trigger::offCallBack(void(*func)())
{
  offCB = func;
}
