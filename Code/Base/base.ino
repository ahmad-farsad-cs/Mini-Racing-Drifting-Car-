#include <avr/io.h>
#include <math.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>
#define BIT(a) (1 << (a))

RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"1GR04", "2GR04"};

//=================== external interrupt ================================
void setupExtInt();														//
unsigned int chIn[9];													//
//=======================================================================


void setup()
{
	setupExtInt();
 	Serial.begin(115200);
	radio.begin();
	radio.openWritingPipe(addresses[0]); // 00001
	radio.openReadingPipe(1, addresses[1]); // 00002//can receive up to 6 input channels
	radio.setPALevel(RF24_PA_MAX);
	radio.setChannel(103);
	
}



void loop()
{
	
    const int NMAX = 64;
    static char buffer[NMAX];
    int i = 0, len;
    int j,k;
    char ch;
    
   	radio.stopListening();
	
	radio.write(&chIn, sizeof(chIn));
		 
	radio.startListening();
	/* for(i=0;i<9;i++){
		Serial.print(chIn[i]);
		Serial.print("\t");	 
	}*/  
	//Serial.println(); 
    delay(9);
	
	
	int bufferR [9] = {}; 
	int ii;
	if(radio.available()!=0)
	{
		radio.read(&bufferR, sizeof(bufferR));
		
		for(i=0;i<9;i++){
			Serial.print(bufferR[i]);
			Serial.print("\t");
		}
		  
		Serial.print("\n");
	}
  
	
  //delay(10);
}
  

  
void setupExtInt(){
	EICRA |= BIT(ISC11) | BIT(ISC10) | BIT(ISC01) | BIT(ISC00); // both interrupts on rising
	EIMSK |= BIT(INT1) | BIT(INT0); // enabling interrupts function 
}

ISR(INT0_vect){ //hall effect sensor
	static int i=0;
	static unsigned long t=0, t_old=0;
	static unsigned long t1=0, t1_old=0;
	t=micros();
	chIn[i]=(int)(t-t_old);
	//if(i==5) chIn[i]=0;
	if(chIn[i]>3000) i=0;  // move the long free time to chIn[0] and then chIn[1] chIn[2] ...
	i++;
	if(i==9) i=0;
	t_old=t;	
}

ISR(INT1_vect){ // PPM reading
	/* static int i=0;
	static unsigned long t=0, t_old=0;
	static unsigned long t1=0, t1_old=0;
	t=micros();
	chIn[i]=t-t_old;
	if(chIn[i]>3000) i=0;  // move the long free time to chIn[0] and then chIn[1] chIn[2] ...
	i++;
	if(i==9) i=0;
	t_old=t; */
}



