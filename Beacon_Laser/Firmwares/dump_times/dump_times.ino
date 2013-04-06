  /*
 * beacon_1_dump.c
 * --------------------
 * Copyright : (c) 2013, Germain Haessig <germain.haessig@crans.org>
 * Licence   : BSD3
 *
 * This file is a part of [kro]bot.
 */


int pin_interrupt = 2 ;
int pin_A = 3 ;
int pin_B = 4 ;
int pin_C = 5 ;

volatile unsigned long times[12]={0,0,0,0,0,0,0,0,0,0,0,0};
volatile int index_times=0;

int i;

volatile boolean flag_data_ready,cmd_get_data;
float N ;
float l = 2 ;
float L = 3 ;
float k = 0.5*l ;
float r = 28E-3 ;

float gamma_A,gamma_B,gamma_C;
float d_A,d_B,d_C;

float alpha, beta,gamma;

float x_angle,y_angle;


void setup() {
  delay(2000);
  flag_data_ready = false ;
  cmd_get_data = true;
  Serial.begin(115200);
  attachInterrupt(0,interrupt1,RISING);
}



void loop() {
  static float A = 0 ;
  static float B = 0 ;
  static float denom = 0 ;

  if(flag_data_ready)  {
    flag_data_ready = false ;
    N=(1E6/(times[6]-times[0])*60);  // beacon RPM 
    
    /*gamma_A = N*(times[1]-times[0])*M_PI/30;
    gamma_B = N*(times[3]-times[2])*M_PI/30;
    gamma_C = N*(times[5]-times[1])*M_PI/30;
    
    d_A = r/(sin(gamma_A/2));
    d_B = r/(sin(gamma_B/2));
    d_C = r/(sin(gamma_C/2));*/
    /*
    alpha=N*(times[2]-times[0])*1E-6*M_PI/30;
    beta=N*(times[4]-times[2])*1E-6*M_PI/30;
    gamma=N*(times[6]-times[4])*1E-6*M_PI/30;
    
    A = tan(alpha);
    B = tan(beta) ;
    
    denom = B*B*l*l+A*A*l*l*B*B+A*A*B*B*k*k+A*A*L*L+A*A*k*k+B*B*A*A*L*L-2*A*A*l*B*B*k-2*A*A*l*B*L+2*B*l*A*k-2*B*B*l*A*L;
    x_angle = (l*B-B*k-L)*(A*B*L-A*k-B*k-L)*A*l/denom;
    y_angle = l*(-A*k+A*B*L-B*k-L)*(-l*B-A*k+A*B*L)/denom;
    */
    
    for(i=1;i<=11;i++)  {
     Serial.print(times[i]-times[0]);
     Serial.print(" "); 
    }
    Serial.println();
    
    cmd_get_data = true ;
  }

}


void interrupt1() {
  static int last_beacon = -1 ;
  static boolean measuring = false ;
  unsigned long time = micros();
  int beacon = -1 ; 
  
  if(digitalRead(pin_A))  {
    beacon = 0 ;
  } else if(digitalRead(pin_B))  {
    beacon = 1 ;
  } else if(digitalRead(pin_C))  {
    beacon = 2 ;
  }
  
  if (!measuring && cmd_get_data && last_beacon == 2 && beacon == 0)  {
    measuring = true ;
    cmd_get_data = false ;
    index_times = 0;
  }
  if(measuring)  {
    times[index_times] = time;
    index_times++;
    if(index_times > 11)  {
      measuring = false ;
      flag_data_ready = true ;  
    }
  }
  last_beacon=beacon ;
}

