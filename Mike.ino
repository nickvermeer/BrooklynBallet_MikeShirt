#include "FastLED.h"
#include <Adafruit_NeoPixel.h>
#define NUM_LEDS_ARMS 28
#define NUM_LEDS_PEC 21

#define NUM_PART 1
#define GRAVITY 0.1
#define MAX_V 0.25
#define L_ARM_PIN 0
#define R_ARM_PIN 1
#define L_PEC_PIN 6
#define R_PEC_PIN 12

int l_accel,r_accel;
int l_diff,r_diff;
typedef struct particle{
  float pos;
  float vel;
  int brightness;
  long last_update;
}particle_t;
Adafruit_NeoPixel l_arm_strip = Adafruit_NeoPixel(NUM_LEDS_ARMS, L_ARM_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel r_arm_strip = Adafruit_NeoPixel(NUM_LEDS_ARMS, R_ARM_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel l_pec_strip = Adafruit_NeoPixel(NUM_LEDS_PEC, L_PEC_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel r_pec_strip = Adafruit_NeoPixel(NUM_LEDS_PEC, R_PEC_PIN, NEO_GRB + NEO_KHZ800);

particle_t particles[2][NUM_PART];
CRGB arm_leds[2][NUM_LEDS_ARMS];
void setup(){
  MPUsetup();
  delay(2000);
  randomSeed(analogRead(0));
  pinMode(L_ARM_PIN,OUTPUT);
  pinMode(R_ARM_PIN,OUTPUT);
  Serial.begin(115200);
  l_arm_strip.begin();
  l_arm_strip.show();
  r_arm_strip.begin();
  r_arm_strip.show();
  l_pec_strip.begin();
  l_pec_strip.show();
  r_pec_strip.begin();
  r_pec_strip.show();

  //FastLED.addLeds<WS2811,L_ARM_PIN,RGB>(arm_leds[0],NUM_LEDS_ARMS);
  //FastLED.addLeds<WS2811,R_ARM_PIN,RGB>(arm_leds[1],NUM_LEDS_ARMS);
  for(int strand = 0; strand < 2; strand++){
     for(int clearLed = 0; clearLed < NUM_LEDS_ARMS; clearLed++) {
       arm_leds[strand][clearLed] = CRGB(0,0,0);//CRGB::Black;  
     }
     for(int part=0;part<NUM_PART;part++){
       particles[strand][part].pos=0;//(float)NUM_LEDS_ARMS/2.0;
       particles[strand][part].vel=0;
       particles[strand][part].brightness=100;
       particles[strand][part].last_update=millis();
     }
   }
}

void loop(){
  dmpDataReady();
  dmpDataReady2();
  MPUloop();
  int old_l_accel=l_accel;
  int old_r_accel=r_accel;
  
  l_accel = analogRead(A9);
  r_accel = analogRead(A10);
  l_diff=old_l_accel-l_accel;
  r_diff=old_r_accel-r_accel;
  
  int curr_r = abs(r_diff)*200/1024;
  int curr_l = abs(l_diff)*200/1024;
  
  
  for(int strand = 0; strand < 2; strand++){
     for(int clearLed = 0; clearLed < NUM_LEDS_ARMS; clearLed++) {
       arm_leds[strand][clearLed]-=CRGB(6,6,3);  
     }
   }
   /*
   float ypr_l[3];
   mpu1GetYPR(ypr_l);
   float ypr_r[3];
   mpu1GetYPR2(ypr_r);
   
   float tilt = ypr_l[1]/M_PI*2.0;
   float grav = GRAVITY * -tilt;
   float roll = ypr_l[2] * 255.0/M_PI + 127;
   
   float tilt2 = ypr_r[1]/M_PI*2.0;
   float grav2 = GRAVITY * -tilt2;
   float roll2 = ypr_r[2] * 255.0/M_PI + 127;
   */
   //Serial.println(roll2);
   float grav = GRAVITY;
   float grav2 = GRAVITY;
   
   particles[0][0].vel=grav;
   if (particles[0][0].vel > MAX_V)
     particles[0][0].vel=MAX_V;
   if (particles[0][0].vel < -MAX_V)
     particles[0][0].vel=-MAX_V;
   particles[0][0].pos+=particles[0][0].vel;
   if (particles[0][0].pos > NUM_LEDS_ARMS){
     particles[0][0].vel=0;
     particles[0][0].pos= NUM_LEDS_ARMS-1;
   }     
   if (particles[0][0].pos < 0){
     particles[0][0].vel=0;
     particles[0][0].pos=0;
   }
   CRGB color=CHSV(20,2,255);//(int)roll,255,particles[0][0].brightness);
   arm_leds[0][(int)particles[0][0].pos]=color;//CRGB(particles[0][0].brightness,particles[0][0].brightness,particles[0][0].brightness);

   particles[1][0].vel=grav2;
   if (particles[1][0].vel > MAX_V)
     particles[1][0].vel=MAX_V;
   if (particles[1][0].vel < -MAX_V)
     particles[1][0].vel=-MAX_V;
   particles[1][0].pos+=particles[1][0].vel;
   if (particles[1][0].pos > NUM_LEDS_ARMS){
     particles[1][0].vel=0;
     particles[1][0].pos= NUM_LEDS_ARMS-1;
   }     
   if (particles[1][0].pos < 0){
     particles[1][0].vel=0;
     particles[1][0].pos=0;
   }
   CRGB color2=CHSV(20,2,255);//(int)roll2,255,particles[1][0].brightness);
   arm_leds[1][(int)particles[1][0].pos]=color2;
  
   /*for(int strand = 0; strand < 2; strand++){
     for(int led = 0; led < NUM_LEDS_ARMS; led++){     
     
     }
   }*/
  
     for(int strand = 0; strand < 2; strand++){
       for(int led = 0; led < NUM_LEDS_ARMS; led++){
         if (strand == 0){
             l_arm_strip.setPixelColor(led,l_arm_strip.Color(arm_leds[strand][led][0],arm_leds[strand][led][1],arm_leds[strand][led][2]));
         }else{
             r_arm_strip.setPixelColor(led,r_arm_strip.Color(arm_leds[strand][led][0],arm_leds[strand][led][1],arm_leds[strand][led][2]));
         }     
       }
     }
     if (curr_r > 20){
       curr_r=curr_r*5;
       particles[0][0].vel=0;
       particles[0][0].pos=0;
     }
     if (curr_l > 20){
       curr_l=curr_l*5;
       particles[1][0].vel=0;
       particles[1][0].pos=0;
     }
     for(int led = 0;led < NUM_LEDS_PEC; led++){
       r_pec_strip.setPixelColor(led,r_pec_strip.Color(curr_r,curr_r,curr_r));
     }for(int led = 0;led < NUM_LEDS_PEC; led++){
       l_pec_strip.setPixelColor(led,l_pec_strip.Color(curr_l,curr_l,curr_l));
     }
 
     l_arm_strip.show();
     r_arm_strip.show();
     l_pec_strip.show();
     r_pec_strip.show();
     //delay(1);
}
