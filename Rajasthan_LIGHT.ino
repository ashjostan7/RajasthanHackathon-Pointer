int incomingByte = 0;   // for incoming serial data

void setup() {
    pinMode(8,OUTPUT);
    pinMode(9,OUTPUT);
    pinMode(10,OUTPUT);
    digitalWrite(10, HIGH);
        Serial.begin(74880);     // opens serial port, sets data rate to 9600 bps
}

void loop() {

        // send data only when you receive data:
        /*if (Serial.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial.parseInt();

                // say what you got:
                Serial.print("I received: ");
                Serial.println(incomingByte, DEC);
        }*/
        while(Serial.available()){
            incomingByte = Serial.read();
           // Serial.print("I received: ");
           //     Serial.println(incomingByte, DEC);
                if(incomingByte == '1'){
                  digitalWrite(8,HIGH);
                  }
                  else if(incomingByte == '2')
                  digitalWrite(9,HIGH);
                  else if(incomingByte == '3')
                  digitalWrite(9,LOW);
                  else
                  digitalWrite(8,LOW);
                  
          }
}
