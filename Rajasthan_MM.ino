#include <VirtualWire.h>
#include <SoftwareSerial.h>
SoftwareSerial t(6, 7);
int transPin[] = {8,9,2,3,4,5};
int recPin[] = {A0, A1, A2, A3, A4, A5};
int usedPin[] = {0, 0, 0, 0, 0, 0};
int pinButtonState[] = {0,0,0,0,0,0};
int fanRegulatorPins[] = {10, 11};
void setup()
{
 // vw_setup(6400);
  //vw_rx_stop();
  //vw_set_tx_pin(11);
  t.begin(9600);
  Serial.begin(9600);
  for(int i = 0;i<6;++i){
    pinMode(transPin[i], OUTPUT);
    pinMode(recPin[i],INPUT);
    digitalWrite(transPin[i], LOW);
  }
 /* for(int i = 0;i<2;++i){
    pinMode(fanRegulatorPins[i], OUTPUT);
    analogWrite(fanRegulatorPins[i], 0);
  }*/
}

 //0:key, 1:-x 2:+x 3:-y 4:+y 5,6:rightClick 7,8:LeftClick
 //9:initializeNew(starting index 0)
 //10:typeLight 11:fanType 12:speakerType 13:ProjectorTypr 14:doorType
 //15:clearAllData
 //16:getAppIndexTango
 
 
 int BluetoothData;
 char *msg;
 int appType[10];
 int pinIndex[10];
 int pinFanIndex[10];
 int countApp=0;
  int countFan=0;
 int h = 1, l = 0;
void loop() 
{
  for(int i = 0;i<countApp;++i){
    if(pinButtonState[i] != (digitalRead(recPin[pinIndex[i]])==HIGH?h:l) && usedPin[pinIndex[i]] == 1){
      t.write(i);
      pinButtonState[i] = (digitalRead(recPin[pinIndex[i]]) == HIGH?h:l);
      if(pinButtonState[i] == 1)
        digitalWrite(transPin[pinIndex[i]], HIGH);
      else
        digitalWrite(transPin[pinIndex[i]], LOW);
      delay(50);
    }
  }

  if(Serial.available()){
    if(Serial.read() == 16){
        while (!Serial.available());
        int Data = (char)Serial.read();
        int op, mode;
        switch(appType[Data]){
          case 0: while (!Serial.available());
                  mode = (char)Serial.read();
                  digitalWrite(transPin[pinIndex[Data]], mode);
                  pinButtonState[Data] = mode;
                  break;
           case 4: while (!Serial.available());
                  mode = (char)Serial.read();
                  digitalWrite(transPin[pinIndex[Data]], mode);
                  pinButtonState[Data] = mode;
                  break;
                  
        }
        
    }
    delay(50);
  }
  if (t.available()){
      BluetoothData = (char)t.read();
      if(BluetoothData == 0)
      {//int Data ;
        Serial.print("Key ");
         while (!t.available());
          int Data = (char)t.read();
         Serial.println((char)Data);
      }
      else if(BluetoothData == 1)
      {
        Serial.print("Move x -");
         while (!t.available());
          int Data = (char)t.read();
          Serial.print(Data);
      }
      else if(BluetoothData == 2)
      {
        Serial.print("Move x +");
         while (!t.available());
          int Data = (char)t.read();
          Serial.print(Data);
      }
      else if(BluetoothData == 3)
      {
        Serial.print(" y -");
         while (!t.available());
          int Data = (char)t.read();
          Serial.println(Data);
      }
      else if(BluetoothData == 4)
      {
        Serial.print(" y +");
         while (!t.available());
          int Data = (char)t.read();
          Serial.println(Data);
      }
      else if(BluetoothData == 5)
      {
        Serial.println("LeftPress");
      }
      else if(BluetoothData == 6)
      {
        Serial.println("LeftRelease");
      }
      else if(BluetoothData == 7)
      {
        Serial.println("RightPress");
      }
      else if(BluetoothData == 8)
      {
        Serial.println("RightRelease");
      }
      else if(BluetoothData == 9){
        while (!t.available());
        int Data = (char)t.read();
        appType[countApp] = Data;
        int pin;
        while(1){
          pin = -1;
          for(int j = 0;j < 6;++j){
            if(digitalRead(recPin[j]) == HIGH && usedPin[j] == 0){
              pin = j;
              break;
            }
          }
          if(pin != -1)break;
        }
        digitalWrite(transPin[pin], HIGH);
        usedPin[pin] = 1;
        pinButtonState[pin] = 0;
        pinIndex[countApp++] = pin;
        t.write(1);
        delay(1100);
      }
      else if(BluetoothData == 16){
        while (!t.available());
        int Data = (char)t.read();
        int op, mode;
        switch(appType[Data]){
          case 0: while (!t.available());
                  mode = (char)t.read();
                  digitalWrite(transPin[pinIndex[Data]], mode);
                  pinButtonState[Data] = mode;
                  break;
          case 1: while (!t.available());
                  op = (char)t.read();
                  while (!t.available());
                  mode = (char)t.read();
                  if(op == 0){
                    digitalWrite(transPin[pinIndex[Data]], mode);
                    pinButtonState[Data] = mode;
                  }
                  else{
                    for(int k = 0;k<countFan;++k){
                        
                      }  
                  }
                  break;
           case 4: while (!t.available());
                  mode = (char)t.read();
                  digitalWrite(transPin[pinIndex[Data]], mode);
                  pinButtonState[Data] = mode;
                  break;
                  
        }
        delay(50);
      }
      //Serial.println(BluetoothData);
      //send(msg);
    }
}


//void send (char *message)
//{
//  vw_send((uint8_t *)message, strlen(message));
//    vw_wait_tx();
//}

