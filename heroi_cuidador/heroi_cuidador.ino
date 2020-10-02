#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

bool musculo_relaxado;
bool postura_ereta;

//Variáveis
bool comecouFadiga;
bool terminouFadiga;
int valorSensorAtual;
int valoresSensor[5] = {0, 0, 0, 0, 0};
int media;

float angulo_x;
float angulo_y;
float angulo_z;
bool postura_correta;
bool postura_incorreta;
int contador_postura_correta;
int contador_postura_incorreta;

//Constantes
const int  TETO_VOLTAGEM_REPOUSO = 600;
const int  PISO_VOLTAGEM_FADIGA = 800;

const float ANGULO_POSTURA_CORRETA = 90.0;
const float VARIACAO_INFERIOR = 20.0;
const float VARIACAO_SUPERIOR = 20.0;
const int  MAX_CORRETA = 20;
const int  MAX_INCORRETA = 20;

void setup() {
  // put your setup code here, to run once:

  //Seta os Pinos
  Serial.begin(9600);

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  bool musculo_relaxado = True;
  bool postura_ereta = True;
  
  //Inicia as variáveis
  angulo_x = 0;
  angulo_y = 0;
  angulo_z = 0;
  postura_correta = false;
  postura_incorreta = false;
  contador_postura_incorreta = 0;
  contador_postura_correta = 0;
  comecouFadiga = false;
  terminouFadiga = false;
  media = 0;
}

void loop() {
  //lê os valores do sensor
  leitura();
  atualizaValores();
  
  // Monitora estado da coluna e do músculo
  if(musculo_relaxado && postura_ereta) {
    identificaFadigaMuscular();
    checaPostura();
  }

  if(!musculo_relaxado || !postura_ereta) {
    if(!musculo_relaxado) {
      checaTerminouFadigaMuscular();
      
      if(musculo_relaxado) {
        resetaSistemaEMG();
      }
    }
    if(!postura_ereta) {
      checaPosturaCorreta();
      
      if(postura_ereta) {
        resetaSistemaGiro();
      }
    }
  }

  delay(100);
}

//Funções

void leitura() {
    mpu6050.update();
    angulo_x = mpu6050.getAngleX();
    angulo_y = mpu6050.getAngleY();
    angulo_z = mpu6050.getAngleZ();
}

void atualizaValores() {
  //Reinicia o valor da médoa
  media = 0;
  
  //Leitura do valor do sensor EMG
  valorSensorAtual = analogRead(A0);

  for(int i=0; i<(5-1); i++) {
    valoresSensor[i] = valoresSensor[i+1];
    media += valoresSensor[i];
  }

  valoresSensor[4] = valorSensorAtual;
  media += valorSensorAtual;

  //Calcula a média dos valores do vetor
  media = media/5;
}

void identificaFadigaMuscular() {
  if (media > PISO_VOLTAGEM_FADIGA) {
    comecouFadiga = true;
  }
}

//Musculo em fadiga
void checaTerminouFadigaMuscular() {
  //Disparar motor

  //Fadiga terminou
  if(media < TETO_VOLTAGEM_REPOUSO) {
    comecouFadiga = false;
    terminouFadiga = true;
  }
}

//Musculo saiu da situação de fadiga
void resetaSistemaEMG() {
  //Reseta todas as variáveis para reiniciar a contagem
  comecouFadiga = false;
  terminouFadiga = false;

  //Desliga motor
}

//Checa se a postura está incorreta
void checaPostura() {
  if(angulo_x < ANGULO_POSTURA_CORRETA - VARIACAO_INFERIOR || angulo_x > ANGULO_POSTURA_CORRETA + VARIACAO_SUPERIOR) {
    contador_postura_incorreta = contador_postura_incorreta + 1;
  }

  if(contador_postura_incorreta == MAX_INCORRETA) {
    postura_incorreta = true;
  }
}

void checaPosturaCorreta() {
  if(angulo_x > ANGULO_POSTURA_CORRETA - VARIACAO_INFERIOR && angulo_x < ANGULO_POSTURA_CORRETA + VARIACAO_SUPERIOR) {
    contador_postura_correta = contador_postura_correta + 1;
  }

  if(contador_postura_correta == MAX_CORRETA) {
    postura_incorreta = false;
    postura_correta = true;
  }
}

void resetaSistemaGiro() {
    postura_correta = false;
    postura_incorreta = false;
    contador_postura_correta = 0;
    contador_postura_incorreta = 0;
}