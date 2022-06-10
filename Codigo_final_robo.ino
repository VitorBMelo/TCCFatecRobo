// ROBÔ DETECTOR DE GÁS: SCRIPT FINAL #V1
// FATEC SANTO ANDRÉ
// Autores:THIAGO CARVALHO & VITOR BORGES
// Data: 09/06/2022

//Bibliotecas:
#include <Servo.h> // Servo
#include <DHT.h>  // DHT11

//Definições:
Servo   S1; // Servo para HC dianteiro da frente.
Servo   S2; // Servo para HC dianteiro de tras.

#define DHTPIN A3
#define DHTTYPE DHT11
DHT     dht(DHTPIN, DHTTYPE);


#define MQ2 A0
#define MQ3 A1
#define MQ135 A2
#define Buzzer 15

#define led 2 


//Variaveis:

  //Motor traseiro
int    in1 = 28;
int    in2 = 26;
byte   pino_Pwm1 = 7;
byte   pwm1; // Velocidade do carro partida
byte   pwm11; // Velocidade da ré

  //Motor dianteiro
int    in3 = 24;
int    in4 = 22;
byte   pino_Pwm2 = 6;
byte   pwm2 = 255; // Intensidade giro de roda

  //Conjunto Servo HC
int    trigger1 = 38;
int    echo1 = 40;
int    trigger2 = 39;
int    echo2 = 41;

  //Métricas
int    delay0 = 150; // tempo de acomodação para o HC não dar erro de medida
int    delay1 = 100; // tempo de acomodação para o HC não dar erro de medida
int    delay2 = 400;

float  cronos_Echo1;
float  cronos_Echo2;

float  dist_APO1;   // medida captada pelo hc da frente
float  dist_APO2;   // medida captada pelo hc de tras
float  dist_DEQ;
float  dist_LEQD;
float  dist_LEQT;
float  dist_LC = 15; // medida de referencia do centro do sensor até a lateral esquerda com sobra.    
float  dist_DDT;
float  dist_LDTD;
float  dist_LDTT;
float  dist_Drf0;
float  dist_Dmin;     // diagonal minima para fazer uma curva
float  dist_Dmaior;   // recebe o valor da diagonal da curva a se fazer
float  dist_Dmenor;   // recebe o valor da diagonal oposta

float  dist_Ref0 = 65;  // medida de referencia para distancia
float  dist_Ref1 = 10;  // medida minima para o motor ativar
float  dist_Ref2 = 20;  // medida de referencia para lateral
float  dist_Ref3 = 30;  // medida de referencia para a ativar freio/ré
float  dist_Ref4 = dist_Ref3 + 25;  // medida para o excedente de ré

int    ang_0 = 90;  // Servo centrado  
int    ang_1 = 120; // APO esquerdo  
int    ang_2 = 60;  // APO direito
int    ang_3 = 175; // LATERAL esquerda
int    ang_4 = 5;  // LATERAL direito

int    leituraMQ2 = 0;
int    leituraMQ3 = 0;
int    leituraMQ135 = 0;

  //CONTROLE:
bool   monitora_Dianteira = true;
bool   monitora_Traseira = false;
bool   ativa_APO   = false;
bool   curva_Liberada = false;   

  //DIREÇÃO:
bool   curva_EQ = false;
bool   eq_Livre = false;
bool   curva_DT = false;
bool   dt_Livre = false;
bool   curva_Possivel = false;
bool   manobra = false;
bool   recuo_Max = false;
bool   excedente_Re = false;

bool   memoria_EQ = false;
bool   memoria_DT = false;

int    orientacao_IN3 = 1;
int    orientacao_IN4 = 1;

bool   D1 = false;  // Acelera+
bool   D2 = false; // Faz a curva para esquerda
bool   D3 = false;  // Faz a curva para direita
bool   D4 = false; // Aceleracao reduzida
bool   D5 = false;
bool   D6 = false;  // Freio/acelera-
bool   D7 = false; // Para

bool   gas_MQ2 = false;
bool   gas_MQ3 = false;
bool   gas_MQ135 = false;

bool   ativa_Buzina = false;

 //ACIONAMENTO: 
int    tom; 
int    estado_Botao;
int    botao = 4;
int    delay_Bounce = 30;

bool   memoria_Botao = false;
bool   estado_Atual = false;
bool   estado_ANT   = false;
unsigned long  delay_Botao  = 0;

//Funções ADD:
  //Sensoriamento:
void Sensoriamento(){

  leituraMQ2 = analogRead(MQ2); //Leitura sensor MQ2
  Serial.print("MQ2 GAS: ");
  Serial.println(leituraMQ2);
  if (leituraMQ2 > 140) {
    Serial.println("Concentração reconhecida por MQ2: Gases combustiveis ou fumaça");
    gas_MQ2 = true;
  //  digitalWrite(Buzzer, HIGH);
  //  digitalWrite(led, HIGH);
  } else {
    Serial.println("MQ2: Sem sinal de gás");
    gas_MQ2 = false;
  //  digitalWrite(Buzzer, LOW);
  //  digitalWrite(led, LOW);
  }

      
  leituraMQ3 = analogRead(MQ3); //Leitura sensor MQ3
  Serial.print("MQ3 GAS: ");
  Serial.println(leituraMQ3);
   if (leituraMQ3 > 800) {
    Serial.println("Concentração reconhecida por MQ3: Vapor de alcool");
    gas_MQ3 = true;
  //  digitalWrite(Buzzer, HIGH);
  //  digitalWrite(led,HIGH);
  } else {
    Serial.println("MQ3: Sem sinal de gás");
    gas_MQ3 = false;
 //   digitalWrite(Buzzer, LOW);
 //   digitalWrite(led,LOW);
  }

   
  leituraMQ135 = analogRead(MQ135); //Leitura sensor MQ135
  Serial.print("MQ135 GAS: ");
  Serial.println(leituraMQ135);
   if (leituraMQ135 > 500) {
    Serial.println("Concentração reconhecida por MQ135: TOXICIDADE ALTA: Possivel NH3 / NOX / Alcool / BENZENO");
    gas_MQ3 = true;
 //  digitalWrite(Buzzer, HIGH);
 //   digitalWrite(led,HIGH);
  } else {
    Serial.println("MQ135: Sem sinal de gás");
    gas_MQ135 = false;
 //   digitalWrite(Buzzer, LOW);
 //   digitalWrite(led,LOW);
  }
  
  //Funções responsáveis por fazer a leitura da temperatura
  float temperatura = dht.readTemperature(); //Função para ler a temperatura no sensor DHT1
  Serial.print("temperatura = ");
  Serial.println(temperatura);

  if(gas_MQ2 == true || gas_MQ3 == true || gas_MQ135 == true){
    ativa_Buzina = true;
    D7 = true;
    Serial.println("ALARME ATIVO!");
    }else{
      ativa_Buzina = false;
      D7 = false;
      }
    
  if(millis() - delay_Botao > delay_Bounce){
    estado_Botao = digitalRead(botao);
    if(estado_Botao && (estado_Botao != estado_ANT)){
      memoria_Botao = !memoria_Botao;
      D7 = !D7;
      delay_Botao = millis();
      }
    estado_ANT = estado_Botao;  
    }
    
  if(ativa_Buzina == true && memoria_Botao == true){
    digitalWrite(led,1);
    digitalWrite(Buzzer,1);
    delay(110);
    digitalWrite(Buzzer,0);
    digitalWrite(Buzzer,1);
    delay(200);
    digitalWrite(Buzzer,0);
    digitalWrite(Buzzer,1);
    delay(20);
    digitalWrite(Buzzer,0);    
    }
    else{
    digitalWrite(Buzzer,0);
    digitalWrite(led,0);    
      }
   Serial.print("D7: ");
   Serial.println(D7);   
  }
  
  //Movimentacao robótica:
void radar_APO1(){
  if(monitora_Dianteira == true && D7 == false){
    Serial.print("monitorando dianteira: ");  
    S1.write(ang_0);
    S2.write(ang_0);
    delay(delay0);
    digitalWrite(trigger1,1);
    delayMicroseconds(12);
    digitalWrite(trigger1,0);
    cronos_Echo1 = pulseIn(echo1,1);
    dist_APO1 = cronos_Echo1/58.2;
    Serial.println(dist_APO1);  
  
    if((dist_APO1 < dist_Ref0) && (dist_APO1 != 0)){
      Serial.println("Objeto detectado!.... ");
      ativa_APO = true;
      }  
      else{
        ativa_APO = false;
        manobra = false;
        memoria_EQ = false;
        memoria_DT = false;        
        }
    if(ativa_APO == false){
      curva_Liberada = false;
      } 
      
      Serial.print("Ativa_APO: ");
      Serial.println(ativa_APO);
      Serial.print("Curva_Liberada: ");
      Serial.println(curva_Liberada);      
    }     
  }
  
void radar_APO2(){
  if(monitora_Traseira == true && D7 == false){
    Serial.print("Monitorando traseira: ");
    S2.write(ang_0);
    delay(delay0);
    digitalWrite(trigger2,1);
    delayMicroseconds(12);
    digitalWrite(trigger2,0);
    cronos_Echo2 = pulseIn(echo2,1);
    dist_APO2 = cronos_Echo2/58.2;
    Serial.println(dist_APO2);

    if(dist_APO2 < dist_Ref3){
      Serial.println("Recuo maximo atingido!");
      monitora_Traseira == false;
      recuo_Max = true;
    //  D6 = false;
      }
    else{
      monitora_Traseira = true;
      recuo_Max = false;
      }        
    }
  }

void reconhecimento_Zona(){
  if(ativa_APO == true && curva_Liberada == false && manobra == false && memoria_EQ == false && memoria_DT == false && D7 == false){
    D7 = true;
    direcao7();
    D7 = false;
    
    Serial.println("reconhecimento de Zona ON");
    S1.write(ang_1);
    delay(delay0);
    digitalWrite(trigger1,1);
    delayMicroseconds(14);
    digitalWrite(trigger1,0);
    cronos_Echo1 = pulseIn(echo1,1);
    dist_DEQ = (cronos_Echo1/58.2);
    Serial.print("Diagonal EQ: ");
    Serial.println(dist_DEQ);
    
    S1.write(ang_3);
    S2.write(ang_4);
    delay(delay0);
    digitalWrite(trigger1,1);
    delayMicroseconds(14);
    digitalWrite(trigger1,0);
    cronos_Echo1 = pulseIn(echo1,1);
    dist_LEQD = (cronos_Echo1/58.2);
    Serial.print("Lateral EQ dianteira: ");
    Serial.println(dist_LEQD);

    delay(delay0);
    digitalWrite(trigger2,1);
    delayMicroseconds(14);
    digitalWrite(trigger2,0);
    cronos_Echo2 = pulseIn(echo2,1);
    dist_LEQT = (cronos_Echo2/58.2);
    Serial.print("Lateral EQ traseira: ");
    Serial.println(dist_LEQT);
    
    S1.write(ang_2);
    delay(delay1);
    digitalWrite(trigger1,1);
    delayMicroseconds(14);
    digitalWrite(trigger1,0);
    cronos_Echo1 = pulseIn(echo1,1);
    dist_DDT = (cronos_Echo1/58.2);
    Serial.print("Diagonal DT: ");
    Serial.println(dist_DDT);   
    
    S1.write(ang_4);
    S2.write(ang_3);
    delay(delay0);
    digitalWrite(trigger1,1);
    delayMicroseconds(14);
    digitalWrite(trigger1,0);
    cronos_Echo1 = pulseIn(echo1,1);
    dist_LDTD = (cronos_Echo1/58.2);
    Serial.print("Lateral DT dianteira: ");
    Serial.println(dist_LDTD);  

    delay(delay0);
    digitalWrite(trigger2,1);
    delayMicroseconds(14);
    digitalWrite(trigger2,0);
    cronos_Echo2 = pulseIn(echo2,1);
    dist_LDTT = (cronos_Echo2/58.2);
    Serial.print("Lateral DT traseira: ");
    Serial.println(dist_LDTT);
    }
  }  

void tratamento_Dados(){
  if(ativa_APO == true && curva_Liberada == false && manobra == false && memoria_EQ == false && memoria_DT == false && D7 == false){
    Serial.println("tratamento de dados on ");
    if(dist_LEQD > dist_Ref2 && dist_LEQT > dist_Ref2){
      Serial.println("Esquerda livre");
      eq_Livre = true;
      }
      else{
        Serial.println("Esquerda n livre");
        eq_Livre = false;
        }
    if(dist_LDTD > dist_Ref2 && dist_LDTT > dist_Ref2){
      Serial.println("Direita livre");
      dt_Livre = true;
      }
      else{
        Serial.println("Direita n livre");
        dt_Livre = false;
        }         

    dist_Drf0 = sqrt(sq(dist_LC)+sq(dist_Ref0));
    Serial.print("Dist_Drf0 :");
    Serial.println(dist_Drf0);
    
    dist_Dmin = sqrt(sq(dist_LC)+sq(dist_Ref3));
    Serial.print("Dist_Dmin :");
    Serial.println(dist_Dmin);
        
    if(dist_DEQ > dist_DDT){
      Serial.println("Curva sentido esquerda");
      curva_EQ = true;
      curva_DT = false;
      dist_Dmaior = dist_DEQ;
      dist_Dmenor = dist_DDT;
      }
      else{
        Serial.println("Curva sentido direita");
        curva_EQ = false;
        curva_DT = true;
        dist_Dmaior = dist_DDT;
        dist_Dmenor = dist_DEQ;                
        }
        Serial.print("Diagonal maior: ");
        Serial.println(dist_Dmaior);
        Serial.print("Diagonal menor: ");
        Serial.println(dist_Dmenor);
        
    if(dist_Dmaior >= dist_Dmin){
      Serial.println("Diagonal possivel");
      curva_Possivel = true;
      }
      else{
        Serial.println("Diagonal n possivel");
        curva_Possivel = false;
        } 
    curva_Liberada = true;        
    } 
  }

void processamento(){
  if(dist_APO1 > dist_Ref0 && D7 == false){ // D1
    Serial.println("Aceleração confirmada:");
    pwm1 = 160;
    D1 = true;
    D2 = false;
    D3 = false;
    D4 = false;
    D6 = false;

    orientacao_IN3 = 1;
    orientacao_IN4 = 1;
  }

  if(ativa_APO == true && curva_Possivel == true && curva_EQ == true && eq_Livre == true && curva_Liberada == true && D7 == false && memoria_DT == false){    // D2          
    Serial.println("Curva esquerda confirmada:");
    pwm1 = 165;
    D1 = false;
    D2 = true;
    D3 = false;
    D4 = true;
    D6 = false;
 
    orientacao_IN3 = 0;
    orientacao_IN4 = 1;

    memoria_EQ = true;
    }
   if(ativa_APO == true && curva_Liberada == true && curva_EQ == true && dist_Dmenor > dist_Dmin && dt_Livre == true && D7 == false && memoria_EQ == false){ // D3#
      Serial.println("Curva direita secundaria confirmada:");
      pwm1 = 165;
      D1 = false;
      D2 = false;
      D3 = true;
      D4 = true;
      D6 = false; 
     
      orientacao_IN4 = 1;
      orientacao_IN4 = 0;   
      memoria_DT = true;          
      }

  if(ativa_APO == true && curva_Possivel == true && curva_DT == true && dt_Livre == true && curva_Liberada == true && D7 == false && memoria_EQ == false){     // D3      
    Serial.println("Curva direita confirmada:");
    pwm1 = 150;
    D1 = false;
    D2 = false;
    D3 = true;  
    D4 = true;
    D6 = false; 
    orientacao_IN4 = 1;
    orientacao_IN4 = 0;  
    memoria_DT = true;        
    } 
    if(ativa_APO == true && curva_Liberada == true && curva_DT == true && dist_Dmenor > dist_Dmin && eq_Livre == true && D7 == false && memoria_DT == false){ // D2#
      Serial.println("Curva esquerda secundaria confirmada:");
      pwm1 = 165;
      D1 = false;
      D2 = true;
      D3 = false;
      D4 = true;
      D6 = false;
    orientacao_IN3 = 0;
    orientacao_IN4 = 1;   
    memoria_EQ = true;   
      }

  if(dist_APO1 < dist_Ref3 && dist_APO1 > 0 && excedente_Re == false){ // Logica do excedente de ré PARTE1
    excedente_Re = true;
    }
  else if(excedente_Re == true && dist_APO1 >= dist_Ref4){ // Logica do excedente de ré PARTE2
    excedente_Re = false;
    } 
  Serial.print("excedente_Re: ");  
  Serial.println(excedente_Re);
      
  if(dist_APO1 > dist_Ref3 && dist_APO1 < dist_Ref0 && D7 == false && excedente_Re == false){  // D4
    Serial.println("Aceleração reduzida confirmada!");
    pwm1 = 160;
    D4 = true;
    D6 = false;
    D1 = false;
    manobra = false;
   
    }       

  if(recuo_Max == false && D7 == false && excedente_Re == true){ // D6
    Serial.println("Confirmação de frenagem ou manobra!");
    pwm1 = 170;
    D6 = true;
    D1 = false;
    D4 = false;
    manobra == true;
    monitora_Traseira = true;
    curva_Liberada = false;
    orientacao_IN3 = !orientacao_IN3;
    orientacao_IN3 = !orientacao_IN3;
    }
  }

void direcao1(){
  if(D1 == true && D7 == false){
    Serial.println("Acelerando!");
    analogWrite(pino_Pwm1,pwm1);
    digitalWrite(in1,0);
    digitalWrite(in2,1);

    analogWrite(pino_Pwm2,pwm2);
    digitalWrite(in3,orientacao_IN3);
    digitalWrite(in4,orientacao_IN4);  
    
    }
  }

void direcao2(){
  if(D2 == true && D7 == false){
    Serial.println("Efetuando curva a esquerda");
    analogWrite(pino_Pwm1,pwm1);
    digitalWrite(in3,orientacao_IN3);
    digitalWrite(in4,orientacao_IN4);
    }
  }

void direcao3(){
  if(D3 == true && D7 == false){
    Serial.println("Efetuando curva a direita");
    analogWrite(pino_Pwm1,pwm1);
    digitalWrite(in3,orientacao_IN3);
    digitalWrite(in4,orientacao_IN4);
    }
  }

void direcao4(){
  if(D4 == true && D7 == false){
  Serial.println("Acelerando em reduzida!");
  analogWrite(pino_Pwm1,pwm1);
  digitalWrite(in1,0);
  digitalWrite(in2,1);
    }
  }    

void direcao6(){
  if(D6 == true && D7 == false){
    Serial.println("Freiando/recuando!");
    analogWrite(pino_Pwm1,pwm1);
    digitalWrite(in1,1);
    digitalWrite(in2,0);
    }
  } 

void direcao7(){
  if(D7 == true){
    Serial.println("Parada!");
    analogWrite(pino_Pwm1,pwm1);
    digitalWrite(in1,1);
    digitalWrite(in2,1);
    delay(delay2);
    }
  } 
    
//Funções padrão:
void setup() {
  Serial.begin(230400);
  
  S1.attach(48);
  S2.attach(52);
  
  dht.begin();  
  
  pinMode(trigger1,OUTPUT);
  pinMode(trigger2,OUTPUT);
  pinMode(echo1,INPUT);
  pinMode(echo2,INPUT);
  
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(pino_Pwm1,OUTPUT);
  pinMode(pino_Pwm2,OUTPUT);

  pinMode(MQ2,INPUT);
  pinMode(MQ3,INPUT);
  pinMode(MQ135,INPUT);
  pinMode(Buzzer,OUTPUT);
  pinMode(led,OUTPUT);
  pinMode(botao,INPUT_PULLUP);

}

void loop() {
//Sensores
  Sensoriamento();
  
//Funções para movimentação do robô:
  radar_APO1();                      // Radar de ativação por obstaculo dianteiro ou 'APO'
  radar_APO2();                     // Radar auxiliar APO traseiro
  reconhecimento_Zona();           // Reconhecimento de distancias de retas e diagonais referenciais
  tratamento_Dados();             // Função de tratamento das retas e diagonais referências 
  processamento();               // Função para o processamento de todos os dados
  direcao1();                   // Classe de movimento: aceleração++
  direcao2();                  // Clase de movimento: curva a esquerda
  direcao3();                 // Classe de movimento: curva a direita
  direcao4();                // Classe de movimento: aceleração reduzida
  direcao6();               // Classe de movimento: frenagem e aceleração--
  direcao7();              // Classe de movimento: parada total
  }
