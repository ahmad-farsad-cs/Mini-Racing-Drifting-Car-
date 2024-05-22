//#include <Servo.h>
#include <avr/io.h>
#include <math.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
// bit a
#define BIT(a) (1 << (a))

//=====================================IMU =============================//
void IMUsetup();														//
float doIMU();															//
LSM6 imu;																//
LIS3MDL mag;															//
float heading ;															//
#define MAG_X_MIN -2618													//
#define MAG_X_MAX  3848													//
																		//
#define MAG_Y_MIN -4401													//
#define MAG_Y_MAX 2022													//
																		//
#define MAG_Z_MIN 422													//
#define MAG_Z_MAX 7432	 												//
//======================================================================//


//==========================RF transmitter ==============================//
RF24 radio(7, 8); // CE, CSN											 //
const byte addresses[][6] = {"1GR04", "2GR04"};							 //
//=======================================================================//


//====================== myMicros and myMillis setup ====================
unsigned long myMillis();												//
unsigned long myMicros();												//
void setupTimer2();														//
unsigned long t2OVF=0;													//
//=======================================================================


//=================== external interrupt ================================
void setupExtInt();														//
unsigned long odometer=0;												//
//=======================================================================


//================ pin change (ultrasonic) ==============================
void setupPinChange();													//
void trigger(); // to send high-low pulse to trigger pin of ultrasonic	//
long t_high=0, t_low=0; // for pin change								//
long t_high2=0, t_low2=0;												//
int dist=0;																//
int dist_back=0;														//
//=======================================================================


//========================= Servo setting with timer1 ===================
void setupTimer1();														//
void setServo(int amount, int servoNo);									//
int ch=1;																//
volatile int chOut[2]={1500,1500};										//
#define motor 1															//
#define steering 2														//
//=======================================================================


//======================== mode selecting - different driving modes ====//
void modeSelect();														//
int mode;																//
#define simple 1;														//
#define nomotor 2;														//
#define radar 3;														//
#define rpmControl 4;													//
#define strControl 5; //the faster you go the harder the steering		//
#define headingControl 6; // holds the heading							//
//======================================================================//

void getRPMfront();
void demoServo(long time);
float getRPM();

float t=0.0, t_old, dt=1.0e8 ,rpm=0;
unsigned int chIn[9];
unsigned long failsafe=0;
int anLeft=0,anRight=0;
float rpm_left=0,rpm_right=0;
float str_left=735,str_right=735;
float lft_offst=745.0,rght_offst=745.0;
float servo_amnt=1500; //for pid

void setup() {
	
	setupTimer2(); // for myMicros and myMillis
	setupExtInt();
	setupPinChange();
	setupTimer1();
	IMUsetup();
	heading=doIMU();
	radio.begin();
	radio.openWritingPipe(addresses[1]); // 00001
	radio.openReadingPipe(1, addresses[0]); // 00002
	radio.setPALevel(RF24_PA_MAX);
	
	DDRD|= BIT(5) | BIT(6);// 5 for motor - 6 for servo
	DDRB |= BIT(2) ; // D10-set pin 10 to output (trigger for ultrasonic)
	
	Serial.begin(115200);
	Serial.println("Turn on ESC");
	//delay(3000);
	//demoServo(5000);
	radio.setChannel(103);
	//radio.setAutoAck(true);
	radio.startListening();
	radio.setRetries(2,3);
}


void loop() {
	static unsigned long mil=myMicros();
	static unsigned long v100hz=0;
	
	float error=0, desiredRPM=0,error_cmnd=0, derror=0;
	static float error_old=0;
	
	getRPMfront();
	
	//===================PID Speed Controller===========================//
	desiredRPM=((int)chIn[2]-1500)*20;									//
	if(abs(desiredRPM<200)) desiredRPM=0;								//
	rpm=rpm*0.8+0.2*getRPM(); // smoothing rpm							//
	//desiredRPM=constrain(desiredRPM,0,1500);							//
	error=desiredRPM-rpm;												//
	derror=error-error_old;												//
	error_old=error;													//
	error_cmnd=error/750.0;+derror/200.0;   // this is 1/k_p and 1/k_d	//
	servo_amnt+=error_cmnd;												//	
	if(rpm<10){															//
		if(desiredRPM==0) servo_amnt=1480; // reset value if we stop	//
	}																	//
	servo_amnt=constrain(servo_amnt,1460,1600);							//
	//==================================================================//
	
	
	//==================================radio communication ============//	
	int bufferR [9] = {}; 												//
	int bufferW [9] = {};												//
																		//
	if(radio.available()!=0){											//
	    radio.read(&bufferR, sizeof(bufferR));							//
																		//
		for(int r=0;r<9;r++){											//
			chIn[r] = bufferR[r];										//
		}																//
		bufferW[0]=odometer;											//	
		bufferW[1]=dist;												//
		bufferW[2]=dist_back;											//
		bufferW[3]=rpm;													//
		bufferW[4]=-rpm_left;											//
		bufferW[5]=-rpm_right;											//
		bufferW[6]=desiredRPM;											//
		bufferW[7]=servo_amnt;											//
		bufferW[8]=(int)(heading*10);									//
		radio.stopListening();											//
		radio.write(&bufferW,sizeof(bufferW));							//
		radio.startListening();											//
	}																	//
	//==================================================================//
	
	
	failsafe++;   // if by anyhow the main loop stops working then the motor and 
				  // steering will go to neutral -- 
	
	modeSelect(); //select different driving mode
	heading=doIMU();
	
	if(v100hz%10==0)  //works on 10hz cycles
	{
		
		trigger(); // trig ultrasonic on 10Hz
		anLeft=analogRead(1);
		anRight=analogRead(2);
		
		
				
	}
	else if(v100hz%10==5)// another loop of 10 hz
	{
		Serial.print(chIn[2]); 
		Serial.print("\t");
		Serial.print(chIn[4]); 
		Serial.print("\t"); 
		Serial.print(odometer); 
		Serial.print("\t"); 
		Serial.print(getRPM(),0); 
		Serial.print("\t");
		Serial.print(-rpm_right,0);
		Serial.print("\t");
		Serial.print(-rpm_left,0);
		Serial.print("\t");
		Serial.print(desiredRPM,0);
		Serial.print("\t");
		Serial.print(servo_amnt);
		Serial.print("\t");
		Serial.println(heading,2);
		
				
	}
	
		
	if(myMicros()<mil+10000)   //in order to work 100Hz
	{
		//t1=myMillis();
		delayMicroseconds(mil+10000-myMicros());
	}
	v100hz++;
	mil+=10000;
		
}


void demoServo(long time)
{
	long t=myMillis();
	int i=1500;
	int dir=10;
	do
	{
		
		setServo(i,steering);
		delay(20);
		i=i+dir;
		if(i>1800) dir=-10;
		if(i<1200) dir=10;
			
	}while((myMillis()-t)<time);
	setServo(1500,steering);
	
}

float getRPM()
{
	static float rpm1;
	static long odometer_old=0;
	/* if(odometer_old==odometer) return 0;
	odometer_old=odometer; */
	if(dt<100)dt=100.0;
	if(rpm1==1.0e6*15/dt) //suspect rpm is not true (rpm is same as previous)
	{
		
		if(abs(t-myMicros())>1.0e5) return 0;  // suspect it lasts for more than .1 sec (10 cycles)
	}
	
	rpm1=1.0e6*15/dt;  //15=60/4		60 is min/sec - 4 is number of interrups in a loop
	//if(chIn[2]<1470) rpm1*=-1;
	
	
	return rpm1;
}


void setupTimer2() {
	cli();
	TCCR2A=0;
	TCCR2B=0;
	TCCR2B |= BIT(CS22) ; // prescale 64; each tich 4 microseconds
	TCCR2A |= BIT(WGM21); // CTC mode
	OCR2A= 249; //250*4= 1 millisecond
	TIMSK2 |= BIT(OCIE2A); // compare match A interrupt enable
	sei();
}

ISR(TIMER2_COMPA_vect){
	t2OVF++;
}

unsigned long  myMillis(){
	unsigned long m;
	uint8_t oldSREG=SREG;
	cli();
	m= t2OVF;
	SREG=oldSREG;
	return m;
	
}

unsigned long  myMicros(){
	unsigned long n,m;
	uint8_t old1SREG=SREG;
	cli();
	n= TCNT2;
	m= t2OVF;
	SREG=old1SREG;
	return m*1000+n*4;
}

void setupExtInt(){
	EICRA |=  BIT(ISC10) | BIT(ISC01) | BIT(ISC00); // interrupt0 on rising 1 any change --BIT(ISC11) |
	EIMSK |= BIT(INT1) | BIT(INT0); // enabling interrupts function 
}

ISR(INT0_vect){ //hall effect sensor
	t=micros();
	dt=t-t_old;
	t_old=t;
	odometer+=2;
}

ISR(INT1_vect){ // PPM reading
	
	if((PIND & BIT(3))==(BIT(3))) {		
		t_high2=micros();
	}else{
		t_low2=micros();
		dist_back= int((t_low2-t_high2)/58); //ultrasonic distance
		if(dist_back>400) dist_back=400;
	}
/* 	static int i=0;
	static unsigned long t=0, t_old=0;
	static unsigned long t1=0, t1_old=0;
	t=micros();
	chIn[i]=t-t_old;
	if(chIn[i]>3000) i=0;  // move the long free time to chIn[0] and then chIn[1] chIn[2] ...
	i++;
	if(i==9) i=0;
	t_old=t; */
}

void setupPinChange(){
	PCICR|= BIT(PCIE0); //enable interrupt function (pin change)
	PCMSK0 |= BIT(PCINT1); // pin echo 
}

void trigger(){
	//digitalWrite(trigPin, LOW); 
	PORTB &= ~BIT(2);
	
	delayMicroseconds(2); 
	
	//digitalWrite(trigPin, HIGH);
	PORTB |= BIT(2);
	
	delayMicroseconds(10);
	
	//digitalWrite(trigPin, LOW);
	PORTB &= ~BIT(2);
}

ISR(PCINT0_vect)
{
	
	if((PINB & BIT(1))==(BIT(1))) {		
		t_high=micros();
	}else{
		t_low=micros();
		dist= int((t_low-t_high)/58); //ultrasonic distance
		if(dist>400) dist=400;
	}
}

void setupTimer1() {
	TCCR1A=0;
	TCCR1B=0;
	TCCR1B |= BIT(WGM12) | BIT(CS11) ; // prescale 8; each tick 0.5 microsecond and CTC mode
	OCR1A= 3999; // compare matchA every 2ms
	TIMSK1 |= BIT(OCIE1A) | BIT(OCIE1B); // compare match A and B interrupt enable
	OCR1B=2999;	
}

ISR(TIMER1_COMPA_vect){
	switch(ch){
		case 1: //servo 1
		{
			PORTD|= BIT(5);	
			OCR1B=chOut[0]*2-1;
			
		}break;
		case 2:		// check failsafe of program
		{
			static unsigned long failsafe_pre;
			if(failsafe_pre== failsafe){
			setServo(1470,motor);
			setServo(1500,steering);
			}else{
			failsafe_pre=failsafe;
	}
			
		}break;
		
		case 3:	//servo 2
		{
			PORTD|= BIT(6);
			OCR1B=chOut[1]*2-1;
		}break;
	}
}

ISR(TIMER1_COMPB_vect){
	switch(ch){
		case 1:
		{
			PORTD&= ~BIT(5);
			ch++;
		}break;
		case 3:
		{
			PORTD&= ~BIT(6);
			ch++;
		}break;
		case 10:
		{
			ch=1;
		}break;
		default: ch++;break;
		
	}
}
void setServo(int amount, int servoNo){
	if(servoNo==motor){
	//	if(abs(amount-1500)>500) amount=1500;
	}
	if(amount<2000 && amount>1000){ // check if we have correct value
		if(servoNo==motor){
			chOut[0]=amount;
		}else if(servoNo==steering){
			chOut[1]=amount;
		}
	}
}


void modeSelect(){
	static float desiredHeading;
	static int check1=0;
	float error;
	static float  servo_str;
	
	mode=simple;
	if(chIn[7]>1800) mode = 6; //SWA down == nomotor - for debugging
	if(chIn[8]>1800) mode = rpmControl;   // SWB down == radar
	
		if(mode== 1){ //simple mode - sends to the servos exactly what it recieves 
		setServo((int)chIn[2],motor);
		setServo((int)chIn[4],steering);
		check1=0;
	}else if(mode==2){ // no motor - for debugging
		setServo(1460,motor);
		setServo((int)chIn[4],steering);
	}else if(mode==3){ // use distance for safety
		if(dist<100){
			if(chIn[2]>1460){	// id does not move forward but can move reverse
				setServo(1460,motor);
				setServo((int)chIn[4],steering);
			}else{
				setServo((int)chIn[2],motor);
				setServo((int)chIn[4],steering);
			}
		}else{
			setServo((int)chIn[2],motor);
			setServo((int)chIn[4],steering);
		}
	}else if(mode ==4){ //rpmControl --pid
		setServo((int)servo_amnt,motor);
		setServo((int)chIn[4],steering);
	
	}else if(mode==5){ //str control
		setServo((int)chIn[2],motor);
		setServo((int)constrain(chIn[4]*(1-rpm/30000.0*9),0.1,1),steering);
		
	}else if(mode ==6) { 	// heading control
		if(check1==0){
			desiredHeading=heading;
			check1=1;
			servo_str=chIn[4];
		}
		setServo((int)chIn[2],motor);
		error=(desiredHeading-heading)*10;
		servo_str=chIn[4]+error;
		setServo(constrain((int)servo_str,1200,1800),steering);
		
	}
}

void getRPMfront(){
	str_left=str_left*0.9+0.1*anLeft;	
	str_right=str_right*0.9+0.1*anRight;
	rpm_left=(str_left-lft_offst)*83.4;
	rpm_right=(str_right-rght_offst)*70.5;
	if(abs(getRPM())<10.0)	// change the offset when we stop
		{
			rght_offst=rght_offst*0.99+anRight*0.01;
			lft_offst=lft_offst*0.99+anLeft*0.01;
		}	
	
}

void IMUsetup(){
	
	Wire.begin();

	if (!imu.init()) {
		Serial.println("Failed to detect and initialize IMU!");
		while (1);
	}
	imu.enableDefault();
  
 	imu.writeReg(LSM6::CTRL1_XL, 0x30); // 52 Hz, +/- 2 g full scale
	
	imu.writeReg(LSM6::CTRL2_G, 0x30); // 52 Hz, 245 dps 
   
	if (!mag.init())
	{
		Serial.println("Failed to detect and initialize magnetometer!");
		while (1);
	}
  
	mag.enableDefault(); 
	delay(50);
}

float doIMU(){
	static float tp = 0.0; // previous time
	static float w_filt = 0.0, yaw_filt = 0.0;
	float t, dt, wz;
	static int init = 0;
	// static float N = 0.0;
	
	// accelerometer sensor output
	float f_x, f_y, f_z;

	// gravity vector (m/s^2)
	float g_x, g_y, g_z;
  
	// magnitude of gravitational constant (m/s^2)
	const float g_const = 9.8; 

	// multiply sensor output by conversion factor to get m/s^2
	const float A_CONV = 0.061 * 1.0e-3 * g_const; // for +/- 2g range (see included pdf)
  
	float yaw, pitch, roll;
  
 	// scaled magnetic field vector, -0.5 to 0.5 for each component
	float B_x, B_y, B_z, G_z;
	int cycle=1;
	// loop for average
	for(int i=0;i<cycle;i++){
		imu.read();
		mag.read();

		f_x += imu.a.x * A_CONV;
		f_y += imu.a.y * A_CONV;
		f_z += imu.a.z * A_CONV;

		g_x += -f_x;
		g_y += -f_y;  
		g_z += -f_z;

		B_x += (float)(mag.m.x - MAG_X_MIN) / (MAG_X_MAX - MAG_X_MIN) - 0.5;
		B_y += (float)(mag.m.y - MAG_Y_MIN) / (MAG_Y_MAX - MAG_Y_MIN) - 0.5;
		B_z += (float)(mag.m.z - MAG_Z_MIN) / (MAG_Z_MAX - MAG_Z_MIN) - 0.5;

		G_z += imu.g.z;
	}

	f_x /= cycle;
	f_y /=cycle;
	f_z /=cycle;

	g_x /= cycle;
	g_y /= cycle;
	g_z /= cycle;
  
	B_x /= cycle;
	B_y /= cycle;
	B_z /= cycle;

	G_z /= cycle; 
  
	yaw = atan2(-B_y,B_x);

  
	t = micros()*1.0e-6;
  
	dt = t - tp; // time step
  
	wz += (G_z * 8.75e-3); // deg / s

   // set ICs for filter
	if(!init) {
		yaw_filt = yaw/3.14159*180;
		//    yaw_filt = 0.0;
		init = 1;
	}
  
  // get wz offset wz0 from average of wz in gyro calibrate program
// float wz0 = -4.1631; //prof.
	float wz0 = -2.2;


	yaw_filt += (wz-wz0)*dt;

	tp = t; // record previous time  
  return yaw_filt;
}