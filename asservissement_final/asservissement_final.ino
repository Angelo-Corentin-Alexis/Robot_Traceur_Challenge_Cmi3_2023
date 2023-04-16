#include <SimpleTimer.h>
#include <SoftwareSerial.h>

SoftwareSerial bluetooth(10, 11);


SimpleTimer timer;

int max_pwm_G;
int max_pwm_D;


//Pins moteur Gauche
const byte Encod_G = 3;
const int PWM_G = 6;
const int AIn_G = 12;
const int BIn_G = 13;

//Pins moteur Droit
const byte Encod_D = 2;
const int PWM_D = 9;
const int AIn_D = 7;
const int BIn_D = 8;


const float diametre = 7.6;//diamètre de la roue en cm
const float circonference = PI * diametre;//circonférence de la roue

const int impulsions_par_tour_G = 960*2;//960 FALLING par tour (8 ticks * 120 (coefficient de réduction))
const int impulsions_par_tour_D = 960*2;

volatile unsigned long compt_G = 0;
volatile unsigned long compt_D = 0;

//init echantillonage
unsigned int time = 0;
const int frequence_echantillonnage = 20;


//consigne
double target_cm = 0; //distance que l'on souhaite parcourir, en cm
double target_deg = (360/circonference) * target_cm; //nombre de dégrés que cette distance représente
int target_ticks_G; //plus simple d'asservir en ticks car ce sera toujours un nombre entier
int target_ticks_D;

// init calculs asservissement PID
int erreur_G = 0; //erreur du moteur gauche

int erreur_D = 0; //erreur du moteur droit


int n=0; // compteur qui servira pour les polygones

int vitMoteur_G;
int vitMoteur_D;

//Definition des constantes du correcteur PID, en l'occurence ici, uniquement du coefficient proportionnel
// Coefficient proportionnel à gauche (choisis par essais successifs)
const float kp_g = 3.1; 
// Coefficient proportionnel à droite (choisis par essais successifs)
const float kp_d = 3.3;

bool sensrotation_G_1 = 1;
bool sensrotation_G_2 = 0;

bool sensrotation_D_1 = 1;
bool sensrotation_D_2 = 0;

int timerfonct =0;
int tampontime =0;

char user_input = '0';//pour le bluetooth

void setup() {
  target_ticks_G = target_deg * impulsions_par_tour_G / 360.0; // distance cible pour le moteur gauche
  target_ticks_D = target_deg * impulsions_par_tour_D / 360.0; // distance cible pour le moteur droit
  bluetooth.begin(9600);
  Serial.begin(9600);

  pinMode(Encod_G, INPUT);
  pinMode(PWM_G, OUTPUT);
  pinMode(AIn_G, OUTPUT);
  pinMode(BIn_G, OUTPUT);

  pinMode(Encod_D, INPUT);
  pinMode(PWM_D, OUTPUT);
  pinMode(AIn_D, OUTPUT);
  pinMode(BIn_D, OUTPUT);

  digitalWrite(Encod_G, HIGH);//Resistance interne arduino ON
  digitalWrite(Encod_D, HIGH);
  
  
  attachInterrupt(digitalPinToInterrupt(Encod_G), countUp_G, CHANGE);//Interruptions pour l'asservissement
  attachInterrupt(digitalPinToInterrupt(Encod_D), countUp_D, CHANGE);

  analogWrite(PWM_G, 0);
  analogWrite(PWM_D, 0);
  delay(300);

  timer.setInterval(1000 / frequence_echantillonnage, asservissement);
}

void loop() {
  timer.run();
  
  if (bluetooth.available()) {
    user_input = bluetooth.read(); // le programme stocke la demande de l'utilisateur    
    commande(0,0,0);//réinitialise les erreurs précédentes
    switch(user_input){
      case '2': // carré
        n = 0;
        break;
      case '3': // cercle
        cercle(); // on trace directement le cercle car il n'y a qu'un côté sur cette forme, donc une seule distance à parcourir
        break;
      case '4': // triangle
        n = 2;
        break;
      case '5': // cercle et triangle circonscrit
        n = 0;
        break;
    }
  }
  switch(user_input){
    case '2':
    carre();
    break;
    case '4':
    triangle(20);
    break;
    case '5':
    triangle_cercle();
    break;
  }
  
}



void countUp_G(){ 
// incrémentation ou décrémentation du nombre de tics à gauche
  if (vitMoteur_G >= 0){ //si le robot avance
    compt_G ++;
  }
  else{ // si le robot recule
    compt_G --;
  }
}
void countUp_D(){ // incrémentation ou décrémentation de la valeur de l'encodeur droit
  if (vitMoteur_D >= 0){ 
    compt_D ++;
  }
  else{
    compt_D --;
  }
}


void asservissement(){
  time += 20;

  erreur_G = target_ticks_G - compt_G;
  erreur_D = target_ticks_D - compt_D;
  
  vitMoteur_G = kp_g * erreur_G;
  vitMoteur_D = kp_d * erreur_D;
  
  if(vitMoteur_G > max_pwm_G) vitMoteur_G = max_pwm_G;
  else if(vitMoteur_G < -max_pwm_G) vitMoteur_G = -max_pwm_G;

  if(vitMoteur_D > max_pwm_D) vitMoteur_D = max_pwm_D;
  else if(vitMoteur_D < -max_pwm_D) vitMoteur_D = -max_pwm_D;

  Tourner (vitMoteur_G, vitMoteur_D);
  /*
   Serial.print("G = ");
   Serial.print(erreur_G);
   Serial.println("");
   Serial.print("D = ");
   Serial.print(erreur_D);
  
   Serial.print("G = ");
   Serial.println(vitMoteur_G);
   Serial.println("");
   Serial.print("D = ");
   Serial.println(vitMoteur_D); 
   */
   /*
   Serial.print("G = ");
   Serial.print(compt_G);
   Serial.print("  ");
   Serial.print("D = ");
   Serial.println(compt_D);
  */
  
  

}

void Tourner(int rapportCyclique_G, int rapportCyclique_D){
  //fait avancer ou reculer le robot
  if ( rapportCyclique_G > 0 ){//vitesse gauche positive
    digitalWrite(AIn_G, sensrotation_G_1);
    digitalWrite(BIn_G, sensrotation_G_2);
    
    analogWrite(PWM_G, rapportCyclique_G);
  }
  else{//vitesse gauche négative
    digitalWrite(AIn_G, sensrotation_G_2);
    digitalWrite(BIn_G, sensrotation_G_1);
    
    analogWrite(PWM_G, -rapportCyclique_G);
  }
  if ( rapportCyclique_D > 0 ){//vitesse droite positive
    digitalWrite(AIn_D, sensrotation_D_1);
    digitalWrite(BIn_D, sensrotation_D_2);

    analogWrite(PWM_D, rapportCyclique_D);
  }
  else{//vitesse gauche négative
    digitalWrite(AIn_D, sensrotation_D_2);
    digitalWrite(BIn_D, sensrotation_D_1);

    analogWrite(PWM_D, -rapportCyclique_D);
  }
}
void commande (double distance,int rotationMG,int rotationMD){ 
  // fonction qui affecte les bonnes consignes et réinitialise 
  target_cm = distance;
  target_deg = (360/circonference) * target_cm;
 
  erreur_G = 0; // on remet les erreurs à zéro
  erreur_D = 0; 

  target_ticks_G = target_deg * impulsions_par_tour_G / 360.0;
  target_ticks_D = target_deg * impulsions_par_tour_D / 360.0;
  
  compt_G = 0; // on remet les compteurs à zéro
  compt_D = 0;
  
  analogWrite(PWM_G, 0); // on remet les consignes de vitesse à zéro
  analogWrite(PWM_D, 0);
  
  delay(300);
   
  //on détermine ensuite dans quel sens faire tourner chaque moteur
  if (rotationMG == 1){
    sensrotation_G_1 = 1;
    sensrotation_G_2 = 0;
  }
  else if(rotationMG == 0){
    sensrotation_G_1 = 0;
    sensrotation_G_2 = 1;
  }
  else{
    sensrotation_G_1 = 0;
    sensrotation_G_2 = 0;
  }
  if (rotationMD == 1){
    sensrotation_D_1 = 1;
    sensrotation_D_2 = 0;
  }
  else if(rotationMD == 0){
    sensrotation_D_1 = 0;
    sensrotation_D_2 = 1;
  }
  else{
    sensrotation_D_1 = 0;
    sensrotation_D_2 = 0;
  }
}

void carre(){
  if(millis() > 3000){//necéssaire si on veut éviter de rentrer dans la boucle trop tôt
      if (((erreur_G <=15 && erreur_G>=-15)&&(erreur_D <15 && erreur_D>-15))&&(n%2 == 0 && n<8)){
        /*Si n est pair et inférieur à 8 (8 boucles car 4 côtés + 4 angles à effectuer) 
         * et que les asservissements précédents sont accomplis*/
        max_pwm_G = 120;
        max_pwm_D = 100;
        commande(20,1,1); // tracer un côté de 20 cm
        n++;
        tampontime = millis();
      }
    timerfonct = millis()-tampontime;
    if(timerfonct > 3000){//necéssaire si on veut éviter de rentrer dans la boucle trop tot
      if (((erreur_G <=15 && erreur_G>=-15)&&(erreur_D <=15 && erreur_D>=-15))&&(n%2 == 1 && n<8)){
        /*Si n est impair et inférieur à 8 et que le côté
         * précédent est tracé*/
        max_pwm_G = 96;
        max_pwm_D = 80;
        commande(18.3*PI/4,1,0); // effectuer un angle droit
        n++;
      }
    }
  }
}

void cercle(){
  if(millis() > 1000){//necéssaire si on veut éviter de rentrer dans la boucle trop tot
      if (erreur_G <=10 && erreur_G>=-10){
        n++;//incrémentation utile dans la fonction triangle_cercle
        max_pwm_G = 150;//On redéfini la vitesse maximale
        commande((18.3*PI*2 + 2),1,-1);// -1 dans la commande pour immobiliser le moteur droit
    }
  }
}

void triangle(int d){//contrairement aux autres fonctions, prend une distance en paramètres
  if(millis() > 3000){//necéssaire si on veut éviter de rentrer dans la boucle trop tot
      if (((erreur_G <=15 && erreur_G>=-15)&&(erreur_D <=15 && erreur_D>=-15))&&(n%2 == 0 && n<8)){
        max_pwm_G = 120;
        max_pwm_D = 100;
        commande(d,1,1); //tracer un côté
        n++;
        tampontime = millis();
      }
    timerfonct = millis()-tampontime;
    if(timerfonct >3000){//necéssaire si on veut éviter de rentrer dans la boucle trop tot
      if (((erreur_G <=15 && erreur_G>=-15)&&(erreur_D <=15 && erreur_D>=-15))&&(n%2 == 1 && n<8)){
        max_pwm_G = 96;
        max_pwm_D = 80;
        commande(18.3*PI/3,1,0); // effectuer un angle de 120 degrés
        n++;
      }
    }
  }
}

void triangle_cercle(){
  if(millis() > 1000){
    if(n == 0){
      //à l'état minimal du compteur n (donc à 0), on trace un cercle
      cercle();
      tampontime = millis();
    }
    timerfonct = millis()-tampontime;
    if(timerfonct >3000){//necéssaire si on veut éviter de rentrer dans la boucle trop tot
      if ((erreur_G <=15 && erreur_G>=-15)&&(n == 1)){
        //après avoir tracé le cercle, il faut encore tourner de 60 degrés avant de commencer à tracer le triangle
        n++;
        max_pwm_G = 96;
        max_pwm_D = 80  ;
        commande(18.3*PI/6,1,0); //effectuer un angle de 60 degrés vers l'intérieur du cercle
        tampontime = millis();
      }
    }
    timerfonct = millis()-tampontime;
    if(timerfonct >1000){//necéssaire si on veut éviter de rentrer dans la boucle trop tot
      if (((erreur_G <=10 && erreur_G>=-10)&&(erreur_D <=10 && erreur_D>=-10))&&(n >= 2)){
        triangle(18.3*1.74/2); // triangle de 1.74 * rayon du cercle
      }
    }
  }
}
