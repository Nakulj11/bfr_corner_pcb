
#include "constants.h"
#include <CAN.h>
#include <Arduino.h>


#include <Wire.h>

#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640

#define TA_SHIFT 8 //Default shift for MLX90640 in open air

static float mlx90640To[768];
paramsMLX90640 mlx90640;

uint32_t str_gag_micro = 0xFFFFFFFF;
uint32_t shock_pot_micro = 0xFFFFFFFF;
uint32_t wheel_speed_micro = 0xFFFFFFFF;
uint32_t brake_temp_micro = 0xFFFFFFFF;
uint32_t wheel_temp_micro = 0xFFFFFFFF;


void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("----------");
  Serial.print(F("Starting Corner PCB "));
  Serial.println(BOARD_INDEX);

  CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);

  if (!CAN.begin (500E3)) {
    Serial.println ("Starting CAN failed!");
    while (1);
  }
  else {
    Serial.println ("CAN Initialized");
  }

  RS485.begin(9600);
  RS485.receive();

  pinMode(WHEEL_SPEED_PIN, INPUT);


  //temp sensor stuff

  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  Serial.println("MLX90640 IR Array Example");

  if (isConnected() == false)
  {
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
    while (1);
  }
  Serial.println("MLX90640 online!");

  //Get device parameters - We only have to do this once
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0)
    Serial.println("Failed to load system parameters");

  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0)
    Serial.println("Parameter extraction failed");

}

boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}

void readWriteWheelTemp(){
  if (micros() < wheel_temp_micro) return;

  for (byte x = 0 ; x < 2 ; x++) //Read both subpages
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0)
    {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
    }

    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
  }

  for (int x = start_pixel ; x < end_pixel ; x++)
  {
    
    uint32_t val = mlx90640To[x];

     uint8_t msg[8] = {
      (uint8_t)((val & 0xFF000000) >> 24), 
      (uint8_t)((val & 0x00FF0000) >> 16), 
      (uint8_t)((val & 0x0000FF00) >> 8), 
      (uint8_t)((val & 0x000000FF) >> 0), 
      0, 0, 0, 0};

      sendCanMessage(WHEEL_TEMP_ID_START + (x-start_pixel), msg);

  }

  
  

  
  wheel_temp_micro = micros() + WHEEL_TEMP_READ_INT;
}

void readWriteBrakeTemp(){
  if (micros() < brake_temp_micro) return;

  uint32_t val;
  if (RS485.available()) {
    val = RS485.read();
  }

  uint8_t msg[8] = {
      (uint8_t)((val & 0xFF000000) >> 24), 
      (uint8_t)((val & 0x00FF0000) >> 16), 
      (uint8_t)((val & 0x0000FF00) >> 8), 
      (uint8_t)((val & 0x000000FF) >> 0), 
      0, 0, 0, 0};

  sendCanMessage(BRAKE_TEMP_ID, msg);

  brake_temp_micro = micros() + BRAKE_TEMP_READ_INT;
}



void readWriteWheelSpeed(){
  if (micros() < wheel_speed_micro) return;
  uinst8_t val = digitalRead(WHEEL_SPEED_PIN);

  uint8_t msg[8] = {(uint8_t)(val), 0, 0, 0, 0, 0, 0, 0};

  sendCanMessage(WHEEL_SPEED_ID, msg);

  wheel_speed_micro = micros() + WHEEL_SPEED_READ_INT;
}


void readWriteStrainGauges(){
  if (micros() < str_gag_micro) return;

  uint16_t strgagOut1 = analogRead(STR_GAG_PIN_1);
  uint16_t strgagOut2 = analogRead(STR_GAG_PIN_2);
  uint16_t strgagOut3 = analogRead(STR_GAG_PIN_3);

  strgagOut1 = ((strgagOut1*(str_gag_range/1023.0))+str_gag_zero)*1023.0;
  strgagOut2 = ((strgagOut2*(str_gag_range/1023.0))+str_gag_zero)*1023.0;
  strgagOut3 = ((strgagOut3*(str_gag_range/1023.0))+str_gag_zero)*1023.0;

  uint8_t msg1[8] = {(uint8_t)(strgagOut1>> 8), (uint8_t)(strgagOut1), 0, 0, 0, 0, 0, 0};
  uint8_t msg2[8] = {(uint8_t)(strgagOut2 >> 8), (uint8_t)(strgagOut2), 0, 0, 0, 0, 0, 0};
  uint8_t msg3[8] = {(uint8_t)(strgagOut3 >> 8), (uint8_t)(strgagOut3), 0, 0, 0, 0, 0, 0};

  sendCanMessage(CAN_STR_GAG_ID_1, msg1);
  sendCanMessage(CAN_STR_GAG_ID_2, msg2);
  sendCanMessage(CAN_STR_GAG_ID_3, msg3);

  str_gag_micro = micros() + STR_GAG_READ_INT;
}

void readWriteShockPot(){
  if (micros() < shock_pot_micro) return;

  uint16_t shockPotOut = analogRead(SHOCK_POT_PIN);

  shockPotOut = ((shockPotOut*(shock_pot_range/1023.0))+shock_pot_zero)*1023.0;

  uint8_t msg[8] = {(uint8_t)(shockPotOut>> 8), (uint8_t)(shockPotOut), 0, 0, 0, 0, 0, 0};

  sendCanMessage(CAN_SHOCK_POT_ID, msg);

  str_gag_micro = micros() + STR_POT_READ_INT;
}



void loop() {
  readWriteStrainGauges();
  readWriteShockPot();
  readWriteWheelSpeed();
}



void sendCanMessage(uint16_t id, uint8_t msg[]){
  CAN.beginPacket(id);
  
  for(int i=0;i<8;i++){
    CAN.write(msg[i]);
  }

  CAN.endPacket();
}

