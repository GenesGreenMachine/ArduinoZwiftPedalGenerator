# ArduinoZwiftPedalGenerator
Zwift DPSxxxx Arduino BLE FMTS Pedal Generator Integration

This code uses the ModBus serial interface of a DPSxxxx charge controller and an Arduino Nano 33 IoT
to talk to Zwift, reporting actual watts to Zwift and accepting requests from Zwift to change the
resistance to different wattages.  This code is used in combination with a pedal generator connected to 
a suitable battery has been tested to 300 watts of input, and can potentially handle 1000 watts if generator and batteries are capable.

See GenesGreenMachine.com for build details
