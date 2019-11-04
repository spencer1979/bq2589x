//basic example 
// by Spencer Chen 
// Created @date 2019-11-04

#include <bq2589x.h>
#include <Wire.h>
bq2589x mycharger;
unsigned long last_change = 0;
unsigned long now = 0;


void setup()
{
   Serial.begin(9600);
   Wire.begin();
   mycharger.begin(&Wire);

}

void loop()
{
  now=millis();

  if (now - last_change > 5000)
  {

    last_change = now;

    mycharger.reset_watchdog_timer();                                             //reset watch dog
    Serial.printf("Battery voltage :%d \r\n", mycharger.adc_read_battery_volt()); // Read Battery voltage
    Serial.printf("System Voltage  :%d \r\n", mycharger.adc_read_sys_volt());     // ReadSystem Voltage
    Serial.printf("Bus Volatge :%d \r\n", mycharger.adc_read_vbus_volt());        // Read Volatge
    Serial.printf("Temperature :%d  \r\n", mycharger.adc_read_temperature());     // Read Temperature
    switch (mycharger.get_charging_status())                                      // charger status
    {
    case 0:
      Serial.print(" Charging status : Not Charging \r\n");
      break;
    case 1:
      Serial.print(" Charging status : Pre-charge \r\n");
      break;
    case 2:
      Serial.print(" Charging status : Fast Charging \r\n");
      break;
    case 3:
      Serial.print(" Charging status : Charge Termination Done \r\n");
      break;
    }
    Serial.printf("charger Current : %dA\r\n", mycharger.adc_read_charge_current()); // read charge current .
    Serial.printf("Idmp limit : %dA\r\n ", mycharger.read_idpm_limit());             //Dynamic Power Management current limit
   
  }
 
}

