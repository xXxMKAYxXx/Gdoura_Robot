
#include "Wire.h"
#include <MPU6050_light.h>

//---------------------------Seuil Colors--------------------
int seuilGG;
int seuilG;
int seuilC;
int seuilD;
//int seuilDD;

//----------------------------memory---------------------------
int past;
int counterpast = 0;

int counterwhite = 0;

int vitessemax = 200;
// ---------------------------pid-----------------------------
float kp = 3;

//---------------------------geo----------------------------
MPU6050 mpu(Wire);
unsigned long timer = 0;
float new_angl = 0;

// ---------------------------ultrason--------------------------

int trig = 9;
int echo = 8;

// ---------------------------fin de course--------------------------

int finCourseR = 3;
int finCourseL = 4;

float duration_us, distance_cm;

//---------------------------motors------------------------------
//left motor
int in1 = 5;
int in2 = 6;
//right motor
int in3 = 10;
int in4 = 11;

//---------------------------infrarouge-------------------------

int if_gg= A0;
int if_g = A1;
int if_c = A2;
int if_d = A3;
int if_dd= 12;

//---------------------------led---------------------------
int led = 13;


//---------------------------calcul val black----------------------

float seuilblack(int pin_sensor){
  float seuilblack = 0;
  for (int i=0; i < 50; i++)
  {
    seuilblack += analogRead(pin_sensor);
  }
  return seuilblack = seuilblack / 50;
}

//------------------------calcul val white-------------------------

float seuilwhite(int pin_sensor){
  float seuilwhite = 0;
  for (int i=0; i < 50; i++)
  {
    seuilwhite += analogRead(pin_sensor);
  }
  return seuilwhite = seuilwhite / 50;
}




//------------------------setup------------------------
void setup() {
  Serial.begin (9600);

  // set up the imu---------------------------

  Serial.print(F("MPU6050 status: "));
  Wire.begin();
  byte status = mpu.begin();
 
  Serial.println(status);
  while(status!=0){ }
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");  

  // configure the fin de course pins----------
  pinMode(finCourseR, INPUT_PULLUP);
  pinMode(finCourseL, INPUT_PULLUP);   

  // configure the ultrason pins----------
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  // configure the motor pins-------------
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT); 


  pinMode(led, OUTPUT);
  
  // configure infrarouge pins---------------
  pinMode(if_gg,INPUT);
  pinMode(if_g, INPUT);
  pinMode(if_c, INPUT);
  pinMode(if_d, INPUT);
  pinMode(if_dd,INPUT);

  //calibrage colors-------------------------

  //seuilBlack-------------------------------
  
  digitalWrite(led, HIGH);
  delay(3000);
  float SBgg =seuilblack(if_gg);
  float SBg = seuilblack(if_g);
  float SBc = seuilblack(if_c);
  float SBd = seuilblack(if_d);
  //float SBdd= seuilblack(if_dd);
  digitalWrite(led, LOW);
  delay(2000);

  //seuilWhite--------------------------------
  
  digitalWrite(led, HIGH);
  delay(3000);
  float SWgg = seuilwhite(if_gg);
  float SWg = seuilwhite(if_g);
  float SWc = seuilwhite(if_c);
  float SWd = seuilwhite(if_d);
  //float SWdd = seuilwhite(if_dd);
  digitalWrite(led, LOW);
  
  //calcul seuil--------------------------------

  seuilGG= (SBgg + SWgg)/2;
  seuilG = (SBg  + SWg )/2;
  seuilC = (SBc  + SWc )/2;
  seuilD = (SBd  + SWd )/2;  
  //seuilDD= (SBdd + SWdd)/2;

  /*test
  Serial.println("seuilGG start");
  Serial.println(seuilGG);
  Serial.println(seuilG);
  Serial.println(seuilC);
  Serial.println(seuilD);
  //Serial.println(seuilDD);
  Serial.println("seuilGG end");
  /*test*/
  
  /*test
  //Serial.println(seuilblack(if_dd));
  /*test*/

  while( digitalRead(finCourseR) == 0){
    // mata3mel chey
    Serial.println("olala");}

   
}

//----------------------maze-------------------------------------------

//Calcul distance-------------------------
float distance(){
    digitalWrite (trig, HIGH);
    delay(20);
    digitalWrite (trig, LOW);
    float duration=pulseIn(echo,HIGH);
    float distance=(duration/2)/29.1;
    return distance;
  }

//right turn in maze----------------
void right_turn() {
    //leftmotor
    analogWrite(in1,0);
    analogWrite(in2,70);

   //rightmotor
    analogWrite(in3,(70));
    analogWrite(in4,0);
 }

// left turn in maze-----------------------
void left_turn() {
  //leftmotor
   analogWrite(in1,(70));
   analogWrite(in2,0);

  //rightmotor
   analogWrite(in3,0);
   analogWrite(in4,70);
 }

// forward maze---------------------------------
void forward_maze(){
  //leftmotor
   analogWrite(in1,0);
   analogWrite(in2,70);

   //rightmotor
   analogWrite(in3,0);
   analogWrite(in4,70);
}

//forward asservie-------------------------------
void forward_pid(){
  
   //condition init
   mpu.update();
  float angl = (mpu.getAngleZ());

  while(true){
    mpu.update();
    float err = mpu.getAngleZ() - angl;
    float v_right = vitessemax+err*kp;
    float v_left  = vitessemax-err*kp;
    
    if (v_right < 0 ){
      v_right = 0;
    } else if 
      (v_right > 255 ){
      v_right = 255;
    }

    if (v_left < 0 ){
        v_left = 0;
    } else if 
      (v_left > 255 ){
       v_left = 255;
    }

    analogWrite(in1,0);
    analogWrite(in2,v_left);

    analogWrite(in3,0);
    analogWrite(in4,v_right);

    if ( digitalRead(finCourseR) && digitalRead(finCourseL) )
    {break;}
  }
}

//----------------inverse chaine-----------------------
String inverse_chaine(String ch){
  int lengthch = ch.length();
  for (int i = 0; i < lengthch; i++){
    if (ch[i] == '0'){
      ch[i] = '1';
    }
    else {ch[i] = '0';}
  }
  return ch;
}

void arrete(){
  analogWrite(in1,0);
  analogWrite(in2,0);

  analogWrite(in3,0);
  analogWrite(in4,0);
}

void wakher(){

  analogWrite(in1,100);
  analogWrite(in2,0);

  analogWrite(in3,100);
  analogWrite(in4,0);

   delay(500);
  
  while (distance < 10){
  analogWrite(in1,100);
  analogWrite(in2,0);

  analogWrite(in3,100);
  analogWrite(in4,0);
  }
  arrete();
  
}

void dour_pid(float target_angl){
  //condition init
   ;
   float angl = target_angl + new_angl;

  while(true){
    mpu.update();
    float err = mpu.getAngleZ() - angl;

    if (err > 1){
      left_turn();}
      else if(err < -1){
        right_turn();
      }
      else {
        arrete();

      break;}
      
  }
}
//--------------------maze-----------
void maze(float target_angl){
  
  forward_pid();
  arrete();
  forward_maze();
  delay(500);
  mpu.update();
  new_angl = mpu.getAngleZ();
  wakher();
  
  dour_pid(target_angl);
  
}

//------------------------------------------------
void forward() {
  //
  analogWrite(in1,0);
  analogWrite(in2,vitessemax);

  analogWrite(in3,0);
  analogWrite(in4,vitessemax);
}

void backward(){
  analogWrite(in1,vitessemax);
  analogWrite(in2,0);

  analogWrite(in3,vitessemax);
  analogWrite(in4,0);
}

void right() {
  //leftmotor
  analogWrite(in1,0);
  analogWrite(in2,vitessemax);

  //rightmotor
  analogWrite(in3,vitessemax*0.5);
  analogWrite(in4,0);
 }

void left() {
  //leftmotor
  analogWrite(in1,vitessemax*0.5);
  analogWrite(in2,0);

  //rightmotor
  analogWrite(in3,0);
  analogWrite(in4,vitessemax);
 }
 
String readSensors()
 {
  String ch="";
  
  //IR GAUCHE EXTREME------------------------------------------
    if ( analogRead(if_gg)<seuilGG){
      ch=ch+'0';}
      else {
        ch=ch+'1';}
    
  //IR GAUCHE-------------------------------------------------
    if ( analogRead(if_g)<seuilG)
    {
      ch=ch+'0';}
        else {
    ch=ch+'1';}
    
  //IR CENTER-------------------------------------------------------
     if ( analogRead(if_c)<seuilC){
      ch=ch+'0';}
      else {
        ch=ch+'1';}
    
  //IR DROITE-------------------------------------------------------
     if ( analogRead(if_d)<seuilD)
     {
      ch=ch+'0';}
       else {
        ch=ch+'1';}
 

  //IR DROITE EXTREME------------------------------------------------
    if (digitalRead(if_dd))
    {
      ch=ch+'1';}
      else {
        ch=ch+'0';}
        return ch;}

///-----------------------------loop------------
void loop() {

//------------------Logique blanc-------------------------------

  if (analogRead(if_gg) > seuilGG) //the far left of the robot detects a line
  { past = 1;
  counterpast = 0; 
  }
  
  if (digitalRead(if_dd))           //the far right of the robot detects a line
  { past = 0;
  counterpast = 0;
  }
  
  String chaine = readSensors(); 
  Serial.println(chaine);
  
  if (chaine == "00100"){
    counterpast++;                  //continuous line
  }
   
  if (counterpast>40){
    past = 3;
    counterpast = 0;                // has been on a continious line for some time
  }
  
   /*forward-------------------------------------------------*/
   
   if ( chaine=="00100" /*or chaine=="10010" */ or chaine=="01010" 
   /*or chaine=="10110" */or chaine=="01110" or chaine=="10001" or 
   chaine=="01001"/* or chaine=="11001" */or chaine=="10101" 
   /*or chaine=="01101" */ /*or chaine=="10011" or chaine=="01011" 
   or chaine=="11011" */ /*zedna 10111 01001 bch me yerja3ch lel lowel*/
   or chaine=="10111" or chaine=="11111" or chaine=="10100" 
   or chaine=="11010" or  chaine=="11110" or chaine=="01111" )
   /*11010 zedneha ll forward*/
   {forward();}
   
   /*left-------------------------------------------------*/
   else if (chaine=="10000" or chaine=="01000" or chaine=="11000" or 
    /*or chaine=="10100"  hattineha forward*/chaine=="01100" or chaine=="11100"
    /*or chaine=="11101"*/) 
   {
  // if ( past == 3 ){
      left();
      
  //  }
  //  else if (past == 1){
  //    forward();
  //  }
  //  else{
  //    left();}
    }
    
  
  /*right-------------------------------------------------*/
   else if (chaine=="00110" or chaine=="00001" or chaine=="00011" or chaine=="00111"
   or chaine=="00101" or chaine=="00010" ) 
   {
   right();
    } 
    else if (chaine=="00000" ){
      if (past == 1){
        left();
      }
      else if (past == 0)
      {
        right();
      }
    else if (past == 3){
      forward();
    }
    }
  
    if ( digitalRead(finCourseR)  && digitalRead(finCourseL) ){
       maze(86);
        maze(-86);
        maze(-86);
        maze(86);
        maze(86);
        maze(-86);
        maze(-86);
        maze(86);
        
        chaine = readSensors();
  
          //------------------Maze to partie noire------------------------
          
        while(!((chaine[0] == '1') && (chaine[4] == '1'))){
          chaine = readSensors();
          forward(); 
        }
  
         //--------------------Partie noire---------------------
        while(true){
          if (analogRead(if_gg) < seuilGG){ 
            past = 1;
            }
  
          if (!digitalRead(if_dd)){ 
            past = 0;
            }
            
          String chaine = readSensors();
          chaine = inverse_chaine(chaine);
  
          if (chaine == "11111"){
            counterwhite++;
          }
  
          if (counterwhite > 150){
            arrete();
            counterwhite = 0;
            delay(15000);
          }
  
          Serial.println(inverse_chaine(chaine));
          
          if (chaine == "00100"){
            counterpast++;
            }
            
          if (counterpast>90){
            past = 3;
            counterpast = 0;
            }
   
   
   /*forward-------------------------------------------------*/
   
   if ( chaine=="00100" /*or chaine=="10010" */
   or chaine=="01010" 
   /*or chaine=="10110" */or chaine=="01110" or chaine=="10001" /*or chaine=="01001" 
   or chaine=="11001" */or chaine=="10101" /*or chaine=="01101" */
   /*or chaine=="10011" or chaine=="01011" or chaine=="11011" or chaine=="10111"  */ 
   or chaine=="11111" or
   chaine=="11000" or 
   chaine=="11100" or chaine=="10100" or chaine=="10000"  or chaine=="11010"  )
   {forward();}
   
   /*left-------------------------------------------------*/
   else if ( chaine=="01000" or chaine=="01100" or chaine=="11110" /*or chaine=="11101"*/) 
   {
    left();}
  // 
  /*right-------------------------------------------------*/
   else if (chaine=="00110" or chaine=="00001" or chaine=="00011" or chaine=="00111"
    or chaine=="01111" or chaine=="00101" or chaine=="00010" ) 
   {
   right();
    } 
    
    else if (chaine=="00000" ){
      
      if (past == 1){
        left();
      }
      else if (past == 0)
      {
        right();
      }
      else if (past == 3){
      forward();
    }
    }
    }
    } 
  }
   
