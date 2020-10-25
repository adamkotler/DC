//Declare constant pins
const int lim=7;
const int dir=12;
const int mvolt=3;
const int brake=9;
const int enc=6;

//Declare and initialize overhead constant/dynamic parameters
volatile long int pulses=0;
long int initial_Pulses=0;
long int t5MM=230,t10MM=460,t30MM=1380;
float v,d=0;
float e_n=0,e_o=0;
int v_n;

//Experimental gain factors "look-up table" for control loop
float kp=0.4,kd=0.4;

//Control setup
void setup() {
  pinMode(dir,OUTPUT); // Dir
  pinMode(mvolt,OUTPUT);  // Current
  pinMode(brake,OUTPUT);  // brake
  pinMode(enc,INPUT);   //encoder
  pinMode(lim,INPUT_PULLUP);  //momentary button
  attachInterrupt(digitalPinToInterrupt(2),count,RISING); //interrupt for continuous acquisition
  digitalWrite(brake,LOW);
  Serial.begin(115200);
}

\\DC-MOTOR CONTROL SEQUENCE
void loop() {
  digitalWrite(dir,HIGH);
  while(digitalRead(lim)==HIGH){
    Serial.println("========WAITING FOR USER TOGGLE========");          //Print nothings happening until user presses button
  }                                     
  delay(1000);                                                          //Delay 1 second
  
  //SEQUENCE 1: MOVE TOWARD SWITCH AT 20mm/s
  while(digitalRead(lim)==HIGH){
    v=dcVel(10000);
    d=8.0*(pulses/374.0);
    Serial.println(String(millis())+" "+String(v)+" "+String(d));
    e_n=20+v;
    v_n=v_n+int(kp*e_n+kd*(e_n-e_o));
    if(v_n<0){
      v_n=0;
    }else if(v_n>255){
      v_n=255;
    }
    analogWrite(mvolt,v_n);
    e_o=e_n;
  }
  Serial.println("=========END LIMIT PULL==========");                  //signal sequence end
  //SEQUENCE 1 END
  
  //SEQUENCE 2: RETRACT 5 MM at 10MM/s
  digitalWrite(dir,LOW);
  v_n=0;e_o=0;
  initial_Pulses=pulses;
  analogWrite(mvolt,0);
  delay(500);
  while((pulses-initial_Pulses<0.4*t5MM)){
    v=dcVel(10000);
    d=8.0 * (pulses/374.0);
    Serial.println(String(millis())+" "+String(v)+" "+String(d));
    e_n=10-v;                                                           //5mm/s gave me trouble so I bumped up to 10 mm/s
    v_n=v_n+int(0.1*e_n+kd*(e_n-e_o));
    if(v_n<0){
      v_n=0;
    }else if(v_n>255){
      v_n=255;
    }
    analogWrite(mvolt,v_n);
    e_o=e_n;
  }
  Serial.println("=========END 10MM RETRACT==========");                //signal sequence end
  //SEQUENCE 2 END
  
  //SEQUENCE 3: MOVE TOWARD SWITCH AT 1 MM/S
  digitalWrite(dir,HIGH);
  analogWrite(mvolt,127);  //My motor wont function at any lower input
  while(digitalRead(lim)==HIGH){
    v=dcVel(10000);
    d=8.0 * (pulses/374.0);
    Serial.println(String(millis())+" "+String(v)+" "+String(d));
  }
  delay(1000);
  Serial.println("=========END SLOW-LIMIT PULL==========");             //signal sequence end
  //SEQUENCE 3: END
  
  //SEQUENCE 4: RETRACT 30 MM at 5MM/s
  v_n=0;e_o=0;                      //RESET errors for good measure
  initial_Pulses=pulses;
  digitalWrite(dir,LOW);
  analogWrite(mvolt,100);
  while((pulses-initial_Pulses<t30MM)){
    v=dcVel(10000);
    d=8.0 * (pulses/374.0);
    Serial.println(String(millis())+" "+String(v)+" "+String(d));
    e_n=10-v;                                                           //5mm/s gave me trouble so I bumped up to 10 mm/s
    v_n=v_n+int(0.3*e_n+kd*(e_n-e_o));
    if(v_n<0){
      v_n=0;
    }
    if(v_n>254){
      v_n=254;
    }
    analogWrite(mvolt,v_n);
    e_o=e_n;
  }
  Serial.println("=========END 30MM RETRACT==========");                //signal sequence end
  //SEQUENCE 4: END

  //END CONTROL SEQUENCE
  digitalWrite(mvolt,LOW);
  digitalWrite(brake,HIGH);
  delay(3000);
  Serial.println("===================================");    
  Serial.println("=========THAT'S ALL FOLKS!!========");    //Sequence Terminate
  Serial.println("===================================");    
  while(true);
}

//Count function to automatically update pulse count with interrupt
void count(){
  if(digitalRead(7)==HIGH)
    pulses++;
  else
    pulses--;
}

//Function used to calculate dc motor velocity; Used for closed-loop control logic
double dcVel(long int t){
  long int initial_Pulses=0;
  long int d_pulses=0;
  long int dt;
  long int t_0=0;
  float v_m=0.0;
  t_0=millis();
  while(millis()-t_0<t);
  dt=t;
  initial_Pulses=pulses;
  d_pulses=pulses-initial_Pulses;
  v_m=8.0*(double(d_pulses)/374.0);
  v_m=v_m/(double(dt)*.000001);
  return(v_m);
}
