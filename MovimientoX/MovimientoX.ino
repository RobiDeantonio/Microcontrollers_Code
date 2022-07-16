float inc = 0; //es la variable que guarda lo que llega de ROS
float incAb=0; //es una variable auxiliar a inc para enviar a la función tone del motor
int SVON = 7;//estan cambiados por el rele
int SETUP = 2;//estan cambiados por el rele
int dir = 4;
int pin = 8;
boolean direct = LOW;

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh; //llama la clase ROS


void messageCb( const std_msgs::Float32& toggle_msg){//Esta es la función messageCb que cuando ROS envia algo se activa
  inc=toggle_msg.data;
  digitalWrite(SVON,LOW);
//if(inc<0){
//  inc=inc*-1;
//  direct = HIGH;
//  digitalWrite(dir,direct);
//  } else if(inc>0){
//  direct = LOW;
//  digitalWrite(dir,direct);
//  }
//       tone(pin,inc);        //Pulse generation with 3kHz
//       delay(50);
//       noTone(pin);
//       while (inc==0){
//       noTone(pin);
//       nh.spinOnce();
//       }
  
//if(inc==4){
//      digitalWrite(13, HIGH-digitalRead(13));   // blink the led
//    }
//if (inc == 49)//1        //if the read is the key "1", switch SVON to LOW
//    {  
//      digitalWrite(SVON,LOW);
//    }
//    else if (inc == 50)//2   //if the read is the key "2", switch SVON to HIGH
//    {
//      digitalWrite(SVON,HIGH);
//    }
//    else if (inc == 51)//3   //if the read is the key "3", switch SETUP to LOW
//    {
//      digitalWrite(SETUP,LOW);
//    }
//    else if (inc == 52)//4   //if the read is the key "4", switch SETUP to HIGH
//    {
//      digitalWrite(SETUP,HIGH);
//    }
//    else if (inc == 53)//5   //if the read is the key "5", initiate a routine
//    {
//        tone(pin,20000);        //Pulse generation with 3kHz
//        delay(500);           //4 second duration
//        noTone(pin);
////      tone(pin,3000);        //Pulse generation with 3kHz
////      delay(4000);           //4 second duration
////      noTone(pin);           //pulse stop
////      digitalWrite(dir,LOW); //direction toggle
////      tone(pin,2000);        //Pulse generation with 2kHz
////      delay(2000);           //2 second duration
////      noTone(pin);           //pulse stop
////      digitalWrite(dir,HIGH);//direction toggle
////      tone(pin,1000);        //Pulse generation with 1kHz
////      delay(2000);           //2 second duration
////      noTone(pin);           //pulse stop
//    }
//    else if (inc == 54)//6   //if the read is the key "5", toggle the servos direction
//    {
//      direct = !direct;
//      digitalWrite(dir,direct);
//    }
//    else if (inc == 55)//7   //if the read is the key "5", toggle the servos direction
//    {
//      for (int i = 0; i < 100; i++){
//       for (int j = 0; j < 50; j++){
//          tone(pin,4000);        //Pulse generation with 3kHz
//          delay(50);           //4 second duration
//          noTone(pin); 
//          
//        }
//      delay(100)  ;    
//      direct = !direct;
//      digitalWrite(dir,direct);
//    }
//    }
}

ros::Subscriber<std_msgs::Float32> sub("ACTUADORX", messageCb );//llama a la función messageCb y se suscribe al topico chatter


void setup() {
  pinMode(SETUP, OUTPUT);    //Pin instantiation as output for home returning when signal is ON
  pinMode(dir,OUTPUT);       //Pin instantiation for setting the direction of motion
  pinMode(SVON, OUTPUT);     //Pin instantiation for powering the Servo
  
  digitalWrite(SVON,HIGH);   //Initialization of the SVON signal as HIGH. The servo turns on when the SVON signal is in the falling edge
  digitalWrite(SETUP,HIGH);  //Initialization of the SETUP signal as HIGH. The servo goes to home when SETUP signal is in the falling edge
  digitalWrite(dir,direct);  //Initialization of the direction signal as HIGH
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  if(inc==4){
     digitalWrite(13, HIGH);   
    }
  if(inc==5){
     digitalWrite(13, LOW);  
    }
  if(inc<0){
  incAb=inc*-1;
  direct = HIGH;
  digitalWrite(dir,direct);
  } else if(inc>0){
  incAb=inc;
  direct = LOW;
  digitalWrite(dir,direct);
  }
       tone(pin,incAb);        //genera un pulso controlado por inc
       while (inc==0){// cuando se envia cero desde ROS el sistema se detiene con la función noTone
       noTone(pin);
       nh.spinOnce();//esta funcion se tiene que llamar para continuar la comunicación con ROS
       }
  nh.spinOnce();//esta funcion se tiene que llamar para continuar la comunicación con ROS
// if (Serial.available() > 0) {
//    inc = Serial.read();     //Reading of the serial port
//    
// }
}
