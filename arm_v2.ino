#include <Servo.h>

Servo servo1,servo2,servo3,servo4,servo5,servo6,servo7,servo8;  // create servo object to control servos  Arm:1~6 node:7 shake:8

typedef struct {
  unsigned char TxBuf[200];
  unsigned char RxBuf[200];
  unsigned int RxCnt;
  unsigned char RxState;
}UART_Str;

UART_Str Uart1;
unsigned char servoCmd[8] = {90,90,90,20,90,40,90,90};

void setup() {
  servo1.attach(4);  // attaches the servo on pin 9 to the servo object
  servo2.attach(5);  
  servo3.attach(6);  
  servo4.attach(7);  
  servo5.attach(8);  
  servo6.attach(9);
  
  servo7.attach(10); // head DOF  
  servo8.attach(11);   
  
  servo1.write(90); // initialize
  servo2.write(90);
  servo3.write(90);
  servo4.write(20);
  servo5.write(90);
  servo6.write(40);
  //servo7.write(120);
  //servo8.write(120);
  Serial.begin(115200);
  Serial.println("Arm_arduino start");
}

void serialEvent()
{
  unsigned char Byte,CheckSum;

  switch(Uart1.RxState)
  {
    case 0:
    Byte = Serial.read();
    if(Byte==0xc9)   //0xc9
    {
      Uart1.RxBuf[Uart1.RxCnt++] = Byte;
      Uart1.RxState=1;
    }
    break;   // start process
    case 1:
      Byte = Serial.read();
      Uart1.RxBuf[Uart1.RxCnt++]=Byte;   // continuous read incoming byte
      if(Byte==0xca)     //Receive end byte and start checking the checksum  
      {
        CheckSum = 0;
            for(int i=1;i<9;i++)
            {
              CheckSum ^= Uart1.RxBuf[i];
            }
            if(CheckSum == Uart1.RxBuf[9])
            {
              Serial.println("OK");
              for(int i=0;i<8;i++)    // get data from RxBuffer
              {
                servoCmd[i] = Uart1.RxBuf[i+1];
              }
              Uart1.RxState = 0;    // Back to idle state and wait for start byte
              Uart1.RxCnt = 0;
              memset(( &Uart1.RxBuf), 0,sizeof(Uart1.RxBuf));  // flush memory 
            }
      }
      if(Uart1.RxCnt>12)
      {
        Serial.println("Fail");
        memset(( &Uart1.RxBuf), 0,sizeof(Uart1.RxBuf));
          Uart1.RxState =0;
          Uart1.RxCnt =0;
      }
       break;
    default:
      Uart1.RxState =0;
      Uart1.RxCnt =0;
            memset(( &Uart1.RxBuf), 0,sizeof(Uart1.RxBuf));
    break;

  }

}



void loop() {
  
serialEvent();  
servo1.write(servoCmd[0]);
servo2.write(servoCmd[1]);
servo3.write(servoCmd[2]);
servo4.write(servoCmd[3]);
servo5.write(servoCmd[4]);
servo6.write(servoCmd[5]);
servo7.write(servoCmd[6]);
servo8.write(servoCmd[7]);
delay(5);

  
}
