#include <Ultrasonic.h>
#include <QTRSensors.h>
#include <FastLED.h>

const PROGMEM uint8_t ENA=7;
const PROGMEM uint8_t ENB=2;
const PROGMEM uint8_t IN1=6;
const PROGMEM uint8_t IN2=5;
const PROGMEM uint8_t IN3=4;
const PROGMEM uint8_t IN4=3;
Ultrasonic ultrasonicR(A9, A8);
Ultrasonic ultrasonicF(A7, A6);
Ultrasonic ultrasonicL(A5, A4);
int distanceL,distanceR,distanceF;
int prev_i=0,prev_error=0;
QTRSensors qtr;

const PROGMEM uint8_t LED_PIN=10;
const PROGMEM uint8_t NUM_LEDS=5;
static uint8_t PathColorSettings[5]={0,0,0,0,0};
CRGB leds[NUM_LEDS];
uint8_t hue = 0;
long ledactiontime=0;




 uint8_t SensorCount  = 5;
uint16_t sensors[5];
char Dsensors[5];// Dsensors : W B , IntDsensors 1 line 0 background que ce soit B or W mode
static int IntDsensors[5],lastIntDsensors[5]={0,0,0,0,0};
double lastIntDsensorstimes[5];
static const uint8_t analog_pins[] = {A10,A11,A12,A14,A15};

uint8_t otherconditionsCounter=0;
char Prevc=0,mode='S',currentLineColor;
long pos=0,sv=0;
uint16_t position;
int Taction=0;

// ********************************************* IMPORTANT PATH STRING : *************************************************************************
int pathSteps=0;
//const char path[] ="BlLWBls"; // stope with Ss
const char path[] ="BRWBRrs"; // stope with Ss
// String lezm tabda b B or W !!!!!
 // path turns of 90 degrees si 90 degre safya : mahech T or X : ekteb r el l bech idourha bel pid 
// R= RIGHT    ;     L left : B  mode black  ;    W mode WHITE
// F forward tawwalii ;     S stop  ; C other conditions like distance based ones
// EXEMPLE "BRLWLRLRFBCLFCWRRRs"
// ********************************************* IMPORTANT PATH STRING : *************************************************************************


void printreadings(int x=1000){
  delay( x);
Serial.print("analog reading:  ");
Serial.print(analogRead(A10));
Serial.print(" ||| ");
Serial.print(analogRead(A11));
Serial.print(" ||| ");
Serial.print(analogRead(A12));
Serial.print(" ||| ");
Serial.print(analogRead(A14));
Serial.print(" ||| ");
Serial.print(analogRead(A15));
Serial.print(" |||     ");
Serial.print(" L: ");
Serial.print(ultrasonicL.read());
Serial.print(" F: ");
Serial.print(ultrasonicF.read());
Serial.print(" R: ");
Serial.print(ultrasonicR.read());
Serial.println("      **");
}
boolean compare(int a[5],char b[5],int L=5)  {
  for(int i=0;i<L;i++){
    if (b[i]=='x'){} // jump
    else if(a[i]!=(((int)b[i])-48)) // -48 HOWA ASCII DE 0 POUR TRANFORMER CHARACTER TO INT just 0 and 1
      return false;
  }
  return true;
}
void forward(int r = 120, int l =120) {
    analogWrite(ENA, l); 
    analogWrite(ENB, r); 
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    //Serial.print("forward");
}
void right(int r = 120, int l = 120)
{  // THE SPEED OF FORWARD OF RIGHT THEN LEFT WHEEL 
    analogWrite(ENA, l); 
    analogWrite(ENB, r); 
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
   //Serial.print("right");
}

void left(int r =120, int l = 120)
{// THE SPEED OF FORWARD OF RIGHT THEN LEFT WHEEL 
    analogWrite(ENA, l); 
    analogWrite(ENB, r); 
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    //Serial.print(" left ");
}

void back(int R = 110, int L = 110) {// BACK SPEED OF RIGHT THEN LEFT WHEEL 
    analogWrite(ENA, L); 
    analogWrite(ENB, R); 
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW); 
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, HIGH);
   Serial.print(" back ");
}


void stope(int br=0) {
    if (br==0) {
		analogWrite(ENA, 0); 
		analogWrite(ENB, 0); 
		digitalWrite(IN1, LOW); 
		digitalWrite(IN2, LOW);
		digitalWrite(IN3, LOW);
		digitalWrite(IN4, LOW); 
		Serial.println("stop without BRAKE");
    }
    else{
		digitalWrite(ENA, 1); 
		digitalWrite(ENB, 1); 
		digitalWrite(IN1, 1);
		digitalWrite(IN2, 1);
		digitalWrite(IN3, 1);
		digitalWrite(IN4, 1);
		Serial.println("BRAAAKE !!!");
    }
}


int derivative = 0;
int Tp = 200;
int Kp2 = 12;
int Kd2 = 0;
int error=0,lasterror,Turn,powerR,powerL;


void rwfollow(int offset = 10) // offset : distance to the wall
{
  //delay(200);
   // distanceL = ultrasonicL.read(); //Serial.print("distance left : "); Serial.println(distanceL);Serial.println("*************");
    //distanceR = ultrasonicR.read();Serial.print("distance right : "); Serial.println(distanceR);Serial.println("*******************************************");
    //distanceL=10;
    if (distanceR <= 2)// KI YOD5LL F 7IT IMIN MEL 50° JSUQA 30°// CBN
    {
        while (1)// idour ISAR jusqa YARJA3 PARALLEL LEL 7IT LIMIN
        {
            distanceR = ultrasonicR.read();Serial.print("distance right : "); Serial.println(distanceR);Serial.println("*******************************************");
            delay(10);
            back(50, 150);
            if (distanceR >= 5) { break; };
        }
        delay(10);
        right(80, 80); // bech y7abes eddoran bel inertie 
        delay(40);// bech y7abes eddoran bel inertie
        stope();// bech y7abes

    }
    else if (distanceL <=2) // KI YOD5L F 7IT lisar // CBN
    {

        delay(10);
        back(150, 50);
        delay(250);
        left(80, 80); // bech y7abes eddoran bel inertie 
        delay(40);// bech y7abes eddoran bel inertie
        stope();// bech y7abes
    }

    else { // PID FOLLOW
        error = distanceR - offset;    
        derivative = error - lasterror;   
        Turn = Kp2 * error + Kd2 * derivative;
       int r = Tp - Turn;             
       int l = Tp + Turn;
       r=constrain(r,-160,160);
        r = map(r, -160, 160, 40, 130);  // 40 OUTMIN ET 110 OUT MAX
       l = map(l, -2*Tp, 2*Tp, 0 , 170); //BESTTTT + KP 12 0 0
      //l = map(l, -255, 255, 0 , 140);
        
        if (r < 0 && l < 0) { back(abs(r), abs(l)); }
         else if (r < 0 && l >0) { right(0, abs(l)); }
         else if (r > 0 && l < 0) { left(r, 0); }
         else { forward(r, l); };
        lasterror = error;   // lel integrale       
    }
}
 
void updatesensorsblack(){
   position = qtr.readLineBlack(sensors);
   currentLineColor='B';
}
void updatesensorswhite(){
   position = qtr.readLineWhite(sensors);
   currentLineColor='W';
}
void myledwhiteon(){
	ledactiontime=millis();
	for(int i=0;i<NUM_LEDS;i++){
		leds[i]=CHSV(0,0,255);
	}
	FastLED.show();
}
void updatesensors(char S='B'){ // updatesensors( BLACK B mode or WHite W mode)
    distanceL = ultrasonicL.read(); //Serial.print("distance left : "); Serial.println(distanceL);Serial.println("*************");
    distanceR = ultrasonicR.read();//Serial.print("distance right : "); Serial.println(distanceR);Serial.println("*******************************************");
    distanceF = ultrasonicF.read();
       for(int i=0;i<SensorCount;i++){lastIntDsensors[i]=IntDsensors[i];}
       if (S=='B') { position= qtr.readLineBlack(sensors); currentLineColor='B';}
       else {position= qtr.readLineWhite(sensors);currentLineColor='W';}
    //   Serial.print("cuurent color ");Serial.print (currentLineColor); Serial.print("// Dsensors :");
		if((millis()-ledactiontime)>100) // >TEMP LEDS WHITE ON WHEN FLASHING AFTER DOING AN ACTION
		{
		  for(int i=0;i<NUM_LEDS;i++){
				   int x=i*1000;
				   float z=map(constrain(abs(position-x),0,1100),0,1100,255,0); // FEL LED LI 7ATJTNA BEHA TKOUN 0 FEL BA9I 255
				   //leds[i]=CHSV(0,255,255-bri); // LINE FOLLOWING LEDS
			   int bri=constrain(z+PathColorSettings[i],0,255);
			   int c=constrain(z,0,1); // BECH KI NOTHRBO BRI*255 IJI YA 0 YA 255 = COLOR STABLE OF THE LINE FOLLOWINGLEDS
			   int colorr=c*95;
				   leds[i]=CHSV(colorr,255,bri); 
			   
		   }
			FastLED.show();
		}
    // creating Dsensors : W and B readings 
	  //Serial.print("Dsensors :");
       for(int i=0;i<SensorCount;i++){
            if(sensors[i]>(qtr.calibrationOn.maximum[i]+qtr.calibrationOn.minimum[i])/2) Dsensors[i]='B';
            else Dsensors[i]='W';
            //Serial.print(Dsensors[i]);Serial.print("  "); 
       } 
     //  Serial.print("  "); 
	   
    // creating intDsensors : 0 and 1 readings 1=line que ce soit white or black line
	  if (S=='W'){ 
        for(int i=0;i<SensorCount;i++){
        if(Dsensors[i]=='W') IntDsensors[i]=1;
        else IntDsensors[i]=0;
        }
     }
     else{
       for(int i=0;i<SensorCount;i++){
          if(Dsensors[i]=='W') IntDsensors[i]=0;
          else IntDsensors[i]=1;
       }
    }
	    // printing values
    // THIS ONE Serial.print("VALUES Dsensors : "); 
    // THIS ONE for(int i=0;i<SensorCount;i++){
    // THIS ONE Serial.print(IntDsensors[i]); }
    //Serial.print (" LAST: "); 
     // for(int i=0;i<SensorCount;i++){
    //Serial.print((int)lastIntDsensors[i]); }
    
	// saving changing times 
    for(int i=0;i<SensorCount;i++){ if(IntDsensors[i]!=lastIntDsensors[i]) lastIntDsensorstimes[i]=millis();}

	// printing times if u want to check
    // Serial.print("  TIMES CHANGES DIFFERENCE : "); 
    // for(int i=0;i<SensorCount;i++){Serial.print(millis()-lastIntDsensorstimes[i]);Serial.print("  ");  }     // print les temps de changement de capteurs  
     

     // THIS ONE Serial.print(" position : ");Serial.println(position);
     // THIS ONE Serial.println(' ');
}

const PROGMEM float TURNFACTOR=1.1;// multiplier=1 GADCH YON9ES VITESS FEL DORA
const PROGMEM float Kp=0.06;     // 255: 0.1     110: 0.2   
const PROGMEM float Ki=0.05 ;   // 255: 0.05    110: 0.05
const PROGMEM float Kd=0.001;   // 255: 0.003   110: 0.004
const PROGMEM uint8_t rightMaxSpeed=150 ; // 255  50
const PROGMEM uint8_t leftMaxSpeed=150;  // 255  50

//const PROGMEM float TURNFACTOR=1.1;//  multiplier=1  SPEED 150   int SetPoint=2100;// BEST 
//const PROGMEM float Kp=0.06;     // 
//const PROGMEM float Ki=0.05 ;   // 
//const PROGMEM float Kd=0.03;   // 
//const PROGMEM uint8_t rightMaxSpeed=150 ; // 
//const PROGMEM uint8_t leftMaxSpeed=150;  // 







 void pidfollow(int C=1){  

  int SetPoint=2100;//2000
  int med_Speed_R;
  int med_Speed_L;
  if (C!=0 ) updatesensors('B');
  int  p = position - SetPoint;
  int error=p;
  int  i = i + prev_i;
  int  d = error - (prev_error);
  float  pid = (Kp*p)+(Ki*i)+(Kd*d);
  prev_i = i;
  prev_error = error;

  med_Speed_L = leftMaxSpeed - TURNFACTOR*abs(pid);
  med_Speed_R = rightMaxSpeed - TURNFACTOR*abs(pid);
  int leftMotorSpeed = med_Speed_L + pid;
  int rightMotorSpeed = med_Speed_R - pid;
  
  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);
  float multiplier=1;// hetha lel 3jal leli idoro lteli , 0 y3ni matdor 7ata 3ejla lteli
                  // 1 y3ni fi right turn ,  vitesses des pid 3ejla avant w 3ejla arriere
  
  // bech ibaddel les vitess negatives into positive speeds that forward right and left fn can undestand
 if (rightMotorSpeed < 0 && leftMotorSpeed < 0) { back(abs(rightMotorSpeed), abs(leftMotorSpeed)); }
 else if (rightMotorSpeed < 0 && leftMotorSpeed >0) { right(abs(multiplier*rightMotorSpeed), abs(leftMotorSpeed)); }
 else if (rightMotorSpeed > 0 && leftMotorSpeed < 0) { left(rightMotorSpeed, multiplier*abs(leftMotorSpeed)); }
 else { forward(rightMotorSpeed, leftMotorSpeed); };
 // THIS ONE Serial.print(" SPEEDS R,L ");Serial.print(rightMotorSpeed);Serial.print(" , ");Serial.print(leftMotorSpeed);
 // forward(rightMotorSpeed,leftMotorSpeed);
 // delayMicroseconds(140);
}

void ELSE(){

        pidfollow(0); // 0 to disable internal readsensors fn of PID cuz we already updated in Run_Robot
 }

boolean otherconditionsCheck(){
	
	 switch(otherconditionsCounter)
    {
        case 0:
           // Serial.println(" Condition 1 check ");
            //if(Rdistance<100){return true;} example
			return false; 
        case 1:
            Serial.println(" Condition 2 check ");
            
			
			
			break;

        case 2:
            Serial.println(" Condition 3 check ");
            
			
			
			break;

        case 3:
			 Serial.println(" Condition 4 check ");
            
			
			
			break;

        // ADD CASES IF U HAVE MORE 
        default:
            Serial.println("Error! NOMBER OF C IN PATHSTRING IS MORE THAN THE CONDITIONS IN otherconditionsCheck FUNCTION");
			return true;// to continue following and does get stuck 
    }
	return false; // if anything missing in the conditions bech lcode may7bsch

} 
 void otherconditionsDO(){
	switch(otherconditionsCounter)
    {
        case 0:
            Serial.println(" Condition 1 found ");
            
			otherconditionsCounter++;break;
        case 1:
            Serial.println(" Condition 2 found ");
            
			
			
			otherconditionsCounter++;break;

        case 2:
            Serial.println(" Condition 3 found ");
            
			
			
			otherconditionsCounter++;break;

        case 3:
			 Serial.println(" Condition 4 found ");
            
			
			
			otherconditionsCounter++;break;

        // ADD CASES IF U HAVE MORE 
        default:
            Serial.println("Error! NOMBER OF C IN PATHSTRING IS MORE THAN THE CONDITIONS IN otherconditionsDO FUNCTION");
    }
   otherconditionsCounter+=1;
    return 0;
}
	 
	


void Run_Robot(char C='A'){ // C color : A AUTO COLOR FROM String path B black W white
    // be carefull , if COLOR GIVEN , IT WILL BE IGNORED FROM THE PATH READINGS!!!!!
    // if AUTO A IS given , lezm Tabta l path b COLOR !!
    int tempsinterval=5000; // TEMPS DE DIFFERENCE ENTRE CHANGEMENT DE VALEURS DE CAPTEURS , utilise dans changement de mode
    int r;
    if (C=='A') {// mode partie black et partie white bel map ye3rf 
		if(pathSteps==0) 
		{  
			char c=path[pathSteps++];// Serial.println(pathSteps);
			Serial.print("PATH[0] LINE COLOR INITIALISED : "); Serial.println(c);
			Prevc=c;
			updatesensors(Prevc); //r=pathSteps;
			//strncpy(lastIntDsensors, IntDsensors, 18);//
			for(int i=0;i<SensorCount;i++){lastIntDsensors[i]=IntDsensors[i];}
			//pathSteps=r;
			
			//Serial.println(pathSteps);
		  }
		  else if(path[pathSteps]=='B') { 
			if(Prevc=='W'){
			  updatesensors('W'); // TW Y3ML UPDATE BEL W AMA MODE B Y3NI YSTANA FI CIONDITION BECH YBADDEL LEL LINE BLACK
			  mode='B'; // mode S start ; W switch from BtoW LINE; B WtoB ;
			}
		  }
		  else if(path[pathSteps]=='W') { 
			if(Prevc=='B'){
			  updatesensors('B');
			  mode='W'; // mode S start ; W switch from BtoW LINE; B WtoB ;
			}
		  }
		  else {updatesensors(Prevc);  //Serial.println(" debugging 1 ");
		}

		  if((path[pathSteps]=='R')||(path[pathSteps]=='r'))
		  {
			 PathColorSettings[NUM_LEDS-1]=255;
			 for(int i=0;i<NUM_LEDS-1;i++){
			 PathColorSettings[i]=0;
				}
		  }
		  else if((path[pathSteps]=='L')||(path[pathSteps]=='l'))
		  {
			 PathColorSettings[0]=255;
			 for(int i=1;i<NUM_LEDS;i++){
			 PathColorSettings[i]=0;
				}
		  }
		  else if(path[pathSteps]=='F') 
		  {
			 for(int i=1;i<NUM_LEDS;i++){
			 PathColorSettings[i]=0;
				}
			PathColorSettings[2]=255; // 5 led : 2 wastanya 
			//PathColorSettings[3]=255; // 8 led : 3 ET 4 MIDDLE
			//PathColorSettings[4]=255; // 8 led : 3 ET 4 MIDDLE
		  }
		  else if((path[pathSteps]=='W')||(path[pathSteps]=='B'))
		  {
			 PathColorSettings[0]=255;
			 PathColorSettings[NUM_LEDS-1]=255;
			 for(int i=1;i<NUM_LEDS-1;i++){
			 PathColorSettings[i]=0;
				}
		  }	
		  else if(path[pathSteps]=='s') {
			 for(int i=0;i<NUM_LEDS;i++){
			 PathColorSettings[i]=255;
				}
		  }
      
    }
    else updatesensors(C);

   
  /*5 POSIBILITIES The Robo Will encounter 
   *  1 1 1 1 1     =>  START: F OR "CROSS + " or " T " SUIVANT la String path ye3ref 
            => possible : F , R , L , S : STOP 
   *  1 1 x 0 0    =>  -| T intersection left turn OR F 
   *  0 0 x 1 1    =>  |- T intersection right turn OR F 
   *  0 0 0 0 0    =>  OUT OF THE LINE : STOP  S or 
   *  1 x 0 x 1    => changement de mode
   *  ELSE  == The Robo is on the line,
   */
   // Serial.print("sensors  ");Serial.println(IntDsensors);
   
   if(path[pathSteps]=='C') {
	   if(otherconditionsCheck()){
		myledwhiteon();
	   otherconditionsDO();
	   pathSteps++; 
	   }
	   else ELSE();
	   } // capteurs couleur , capteur distance
  
   else if(millis()-Taction>150){
		
		if(compare(IntDsensors,"111xx")||compare(IntDsensors,"xx111")) {// compare (readings , string elli feha les X lezm tkoun 2eme parameter)
					if(path[pathSteps]=='F') {Serial.println("FORWARD : PATH F : ");myledwhiteon();pathSteps++; forward();delay(500);Taction=millis();}
					else if(path[pathSteps]=='R') {
						Serial.println("90° RIGHT : PATH R : ");
						pathSteps++; myledwhiteon();
						right(100,200);
						delay(300);
						while(1){
							right(100,200);//Serial.println("right");
							updatesensors(currentLineColor);
							if(IntDsensors[4]==1) break;
						}
						Taction=millis();
					}
					else if(path[pathSteps]=='L') {
						Serial.println("90° LEFT : PATH L : ");
						pathSteps++; myledwhiteon();
						left(200,100);
						delay(300);
						while(1){

							left(200,100);//Serial.println("left");
							updatesensors(currentLineColor);
							if(IntDsensors[0]==1) break;
						}
						Taction=millis();           
					}
					else if(path[pathSteps]=='l') {
						Serial.print("left 90 safya with pid");
						int t=millis();myledwhiteon();
						while((millis()-t)<800){
							  updatesensors(currentLineColor);
							  ELSE();
						}
					   pathSteps++;
					 }
					 else if(path[pathSteps]=='r') {
						Serial.print("right 90 safya with pid");
						int t=millis();myledwhiteon();
						while((millis()-t)<800){
							  updatesensors(currentLineColor);
							  ELSE();
						}
					   pathSteps++;
					 }
					else if(path[pathSteps]=='s') {Serial.println("DEAD STOP , PATHSTRING : s DONE");stope();delay(100000);}
					else { // ERREUR DANS PATHSTRING GO FOR PID SAFER
							// Y3NI LGUEE DOURA OR INTERSECTION W MALGUECH 7AJA S7I7A FEL PATHSTRING EXEMPLE 00X11 W YALGA LEFT L 
						//pathSteps++;
					   ELSE();Serial.println("SENT TO PID FROM 11111 CONDITION");
					   //goto jump; // INAGGEZ L ADD PATH , if needed remove the comment above
					}
					//jump:;
 			   }
/* 		else if(compare(IntDsensors,"111x0")){ //  left or f
					//Taction=millis();
					if(path[pathSteps]=='F') {Serial.println("FORWARD : PATH F : ");pathSteps++; forward();delay(100);Taction=millis();}
					else if(path[pathSteps]=='L') {
						Serial.println("90° LEFT : PATH L : ");
						pathSteps++; 
						delay(50);
						while(1){
					  
							left(200,100);//Serial.println("left");
							updatesensors(currentLineColor);
							if(IntDsensors[0]==1) break;
						}
						Taction=millis();
					}
					else if(path[pathSteps]=='s') {Serial.println("DEAD STOP , PATHSTRING : s DONE");stope();delay(100000);}
					else if(path[pathSteps]=='l') {
						Serial.print("left 90 safya with pid");
						int t=millis();
						while((millis()-t)<2000){
              updatesensors(currentLineColor);
							ELSE();
						}
           pathSteps++;
					}
					else { // ERREUR DANS PATHSTRING GO FOR PID SAFER
						//pathSteps++; ZA3MA KEN Y8LAT AMA 5IR N5ALLOH IZID PATH WALE LE ???
						ELSE();  Serial.print("SENT TO PID FROM 11x00 CONDITION");
					}
				
				
			  }
		else if(compare(IntDsensors,"0x111")){//  right or f // 
					Serial.print("debugging ENTER LOOP  ");
					Serial.println(path[pathSteps]);
					if(path[pathSteps]=='F') {Taction=millis();Serial.println("FORWARD : PATH F : "); pathSteps++;forward();delay(100);Taction=millis();}
					else if(path[pathSteps]=='R') {
						Serial.println("90° RIGHT : PATH R : ");
						pathSteps++; 
						delay(50);
						while(1){
							right(100,200);//Serial.println("right");
							updatesensors(currentLineColor);
							if(IntDsensors[4]==1) break;
						}
						Taction=millis();
					}    
					else if(path[pathSteps]=='s') {Serial.println("DEAD STOP , PATHSTRING : s DONE");stope();delay(100000);}
					else { // ERREUR DANS PATHSTRING GO FOR PID SAFER
					  //pathSteps++; ZA3MA KEN Y8LAT AMA 5IR N5ALLOH IZID PATH WALE LE ???
					  ELSE();Serial.println("SENT TO PID FROM 00x11 CONDITION");
					}
				
			  }  */
		else if(compare(IntDsensors,"110x1")||compare(IntDsensors,"1x011")){ // CHANGEMENT DE mode : point of changing found so stop using the mode variable
					//Taction=millis();
					// time checking first !
					boolean check=true;
				   
					if ((millis()-lastIntDsensorstimes[0])>tempsinterval) check=false;
					if ((millis()-lastIntDsensorstimes[2])>tempsinterval) check=false;
					if ((millis()-lastIntDsensorstimes[4])>tempsinterval) check=false;
					Serial.print("CHECK = ");Serial.print(check);
					
					if (check==true){
						  Taction=millis();
						  if(mode=='W'){
							Prevc='W';Serial.println("SWITCHED TO WHITE LINE : PATH W: ");
							pathSteps++;myledwhiteon();
							mode=='N';
							}
						  else if(mode=='B'){
							Prevc='B';Serial.println("SWITCHED TO BLACK LINE : PATH W: ");
							pathSteps++;myledwhiteon();
							mode=='N'; // normal mode ,cad : NO SEARCHING for linecolor switching ,  now Normal line following
							}
						  else { // ERREUR DANS PATHSTRING GO FOR PID SAFER
							// enable either A or B mech EZZOUZ
						  
						  
							//pathSteps++;   solution A
							//goto ELSE;    A
							
							/* or may we try to correct it automatically here ? if no disable it !! SOLUTION B
							the code will change linefollowing color without looking to the PATHSTRING
							 IT MAYBE JUST A FRACTION OF SECOND READING ERROR , so this correction  CAN CAUSE A HUGE PROBLEM*/
							 
							 
							 //solution B
							if (currentLineColor=='W') Prevc='B';
							else Prevc='W';
							mode=='N'; // normal mode ,cad : NO SEARCHING for linecolor switching ,  now Normal line following
								  // fin solution B 
						  }
					}
					else {
					  ELSE();Serial.print("SENT TO PID FROM 1x0x1 CONDITION");
					}
		}
		else if(compare(IntDsensors,"00000")&&(millis()-Taction>1000)){
			 if(path[pathSteps]=='s') {Serial.println("DEAD STOP , PATHSTRING : s DONE");stope();delay(100000);}
			 else { // ERREUR DANS PATHSTRING GO FOR PID SAFER
					// Y3NI LGUEE DOURA OR INTERSECTION W MALGUECH 7AJA S7I7A FEL PATHSTRING EXEMPLE 00X11 W YALGA LEFT L 
				//pathSteps++;
				ELSE();Serial.println("SENT TO PID FROM 00000 CONDITION");
			}

			
		}
		else{ // pid follow 
			// also fel else hethi ken fama change of line color from W to B or inverse
			// robot will not turn any 90 except for these who are specified in path
			// if the robot find in pathSTRING B or W he will PID FOLLOW and read times until it find
			// switching point of the two lines ,
			//Serial.print("here1");
			ELSE(); // THIS ONE Serial.print(" ELSE PID WORKING ");
    }
   }
   else{ // pid follow 
		// also fel else hethi ken fama change of line color from W to B or inverse
		// if the robot find in pathSTRING B or W he will PID FOLLOW and read times until it find
		// switching point of the two lines ,
		//Serial.print("here1");
		ELSE();  Serial.println(" HE DIDNT READ A PATH STRRING DUE TO TIME RESTRICTION , PID WORKING ");
    }
     
  
// THIS ONE Serial.print("path(searching for:) :");Serial.print(path[pathSteps]);Serial.print(" step: ");Serial.print(pathSteps);Serial.print(" "); 

} 
void calibratesensors(){
        for(int i=0;i<NUM_LEDS;i++){
          leds[i]=CHSV(0,0,0);
          FastLED.show();
        }
        for(int i=0;i<(int)NUM_LEDS/2;i++){
          leds[i]=CHSV(160,255,255);
          leds[NUM_LEDS-i-1]=CHSV(160,255,150);
          FastLED.show();
          delay(500);
        }
        leds[2]=CHSV(160,255,150);
        FastLED.show();
        delay(700);
        for(int i=0;i<NUM_LEDS;i++){
          leds[i]=CHSV(0,0,0);
          FastLED.show();
        }
        Serial.println("starting calibration");
        int hue=1;
        for (uint8_t i = 0; i <NUM_LEDS; i++)
        { 
          for (uint8_t j = 0; j <30; j++)// 30 PRESQUE TEMP BECH Y3ML CALIBRATE
               {
                    qtr.calibrate();
                    delay(20);
               }
          leds[i]=CHSV(100,255,190);
          FastLED.show();
        }
        Serial.println("calibration DONE");
          for (uint8_t i = 0; i < SensorCount; i++)
        {
          Serial.print(qtr.calibrationOn.minimum[i]);
          Serial.print(' ');
        }
        Serial.println();
      
        // print the calibration maximum values measured when emitters were on
        for (uint8_t i = 0; i < SensorCount; i++)
        {
          Serial.print(qtr.calibrationOn.maximum[i]);
          Serial.print(' ');
        }
        Serial.println(" ***********************************************************");
        for(int j=0;j<3;j++){
          for(int i=0;i<NUM_LEDS;i++){
            leds[i]=CHSV(100,255,255);
          }
          FastLED.show();
          delay(200);
          for(int i=0;i<NUM_LEDS;i++){
            leds[i]=CHSV(100,0,0);
          }
          FastLED.show();
          delay(200);
        }
       
}
  void calibratE(){
      
		  Serial.println("starting calibration");
		  for (uint8_t i = 0; i <150; i++)
		  {
			qtr.calibrate();
			delay(10);
		  }

		  Serial.println("calibration DONE");
		// print the calibration minimum values measured when emitters were on	
			for (uint8_t i = 0; i < SensorCount; i++)
		  {
			Serial.print(qtr.calibrationOn.minimum[i]);
			Serial.print(' ');
		  }
		  Serial.println();
		  // print the calibration maximum values measured when emitters were on
		  for (uint8_t i = 0; i < SensorCount; i++)
		  {
			Serial.print(qtr.calibrationOn.maximum[i]);
			Serial.print(' ');
		  }
		  Serial.println(" ***********************************************************");
	}
void setup() {
	pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    FastLED.addLeds<WS2811, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(80);
    stope();     
	//Serial.begin(9600);
	qtr.emittersOff();
	qtr.setTypeAnalog(); // or setTypeAnalog()
  qtr.setSensorPins(analog_pins, SensorCount);
//	calibratE();
	//calibratesensors();
	//PathColorSettings[]={255,255,0,0,0};
  }
void loop() {
//	forward(200,200);
// delay(1300);
//left(200,100);
//delay(300);

updatesensors();
rwfollow(20);
//Serial.println(millis());
//Run_Robot();
//pidfollow();
//Serial.println(millis());
//pidfollow();
//
// if(compare(IntDsensors,"111x0")){ //  left or f
//        Serial.println("90° LEFT : PATH L : ");
//                                    pathSteps++; 
//                                      delay(50);
//                                      while(1){
//                                      
//                                        left(200,100);Serial.println("right");
//                                        updatesensors(currentLineColor);
//                                        if(IntDsensors[0]==1) break;
//                                        }
//     }  
  //stope();
//printreadings(300);

 // updatesensors();
 //delay(300);
 // delay(100);
// for(int i=0;i<SensorCount;i++){
//      Serial.print(Dsensors[i]);
//  }
//      Serial.print("   ||   "); 
// for(int i=0;i<SensorCount;i++){
//      Serial.print(IntDsensors[i]);
//  }   
//  Serial.print("   ||   ");  
//Serial.print("SENSORS : "); 
//   for(int i=0;i<SensorCount;i++){
//     Serial.print("   ");  Serial.print( sensors[i]); Serial.print("    "); 
//  }   
 } // END OF THE LOOP
