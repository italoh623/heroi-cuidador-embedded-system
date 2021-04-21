 ITALO
#include <MPU6050_tockn.h>
#include"BluetoothSerial.h"
#include <string>

BluetoothSerial SerialBT;
MPU6050 mpu6050(Wire);

bool musculo_relaxado;
bool postura_ereta;

bool musculo_relaxado_2;
bool postura_ereta_2;

bool parada_aux;

bool aux;
bool aux2;

bool calibracao = false;
bool operacao = false;
//bool segundo_exercicio = false;
//bool terceiro_exercicio = false;
//bool finalizou_exercicio = false;

int valor_emg_atual;
int valores_emg[5] = {0, 0, 0, 0, 0};
int media_emg;
int j;

float angulo_x;
float angulo_y;
float angulo_z;

int contador_postura_correta;
int contador_postura_incorreta;
int valor_calibracao;

//Constantes
//const int  TETO_VOLTAGEM_REPOUSO = 600;
//const int  PISO_VOLTAGEM_FADIGA = 800;

const int CONTADOR_EMG = 30;
const int FAIXA_EMG_ATIVACAO = 800;

const float ANGULO_POSTURA_CORRETA_X = 90.0;
const float VARIACAO_INFERIOR_X = 65.0;
const float VARIACAO_SUPERIOR_X = 20.0;
const int  MAX_CORRETA = 20;
const int  MAX_INCORRETA = 40;

const int MOTOR_ESQUERDO = 11;
const int MOTOR_DIREITO = 10;

void setup() {
  Serial.begin(9600);
  SerialBT.begin("Heroi Cuidador");
  //  bluetooth.begin(9600);

  pinMode(MOTOR_ESQUERDO, OUTPUT);
  pinMode(MOTOR_DIREITO, OUTPUT);

  digitalWrite(MOTOR_ESQUERDO, LOW);
  digitalWrite(MOTOR_DIREITO, LOW);

  Wire.begin();
  mpu6050.begin();
  mpu6050.setGyroOffsets(-1.20, -4.25, 1.53);
  valor_calibracao = 0;
  musculo_relaxado = true;
  postura_ereta = true;

  musculo_relaxado_2 = true;
  postura_ereta_2 = true;

  parada_aux = true;

  calibracao = false;
  operacao = false;

  aux2 = false;
  aux = false;
}

void loop() {
  String  modo = receber_bluetooth();

  if (modo == "calibracao" || aux2) {
    calibracao = true;
    operacao = false;
    aux2 = false;
    aux = iniciar_calibracao();
  }
  else if (modo == "operacao" || aux ) {
    calibracao = false;
    operacao = true;
    aux = false;
    aux2 = modo_operacao();
  }

  delay(20);
}

void identificaFadigaMuscular() {
  if (media_emg == CONTADOR_EMG) {
    musculo_relaxado = false;

    //Disparar motor
    //    digitalWrite(MOTOR_ESQUERDO, HIGH);
    enviar_bluetooth("{MSGComeçou Fadiga Muscular}");
  }
}

//Musculo em fadiga
bool checaTerminouFadigaMuscular() {
  //Fadiga terminou
  if (media_emg == 0) {
    enviar_bluetooth("{MSGTerminou Fadiga Muscular}");
    musculo_relaxado_2 = true;
    return true;
  }

  return false;
}

//Musculo saiu da situação de fadiga
void resetaSistemaEMG() {
  //Desliga motor
  digitalWrite(MOTOR_ESQUERDO, LOW);
  enviar_bluetooth("{MSGTerminou Fadiga Muscular}");

  media_emg = 0;

  musculo_relaxado_2 = false;
  musculo_relaxado = true;
}

void resetaSistemaGiro() {
  // Desliga motor
  digitalWrite(MOTOR_ESQUERDO, LOW);
  enviar_bluetooth("{" + (String) "MSG" + (String) "Postura Correta" + (String) "}");

  contador_postura_correta = 0;
  contador_postura_incorreta = 0;

  postura_ereta_2 = false;
  postura_ereta = true;
}

//Checa se a postura está incorreta
void checaPostura() {
  if (angulo_x < ANGULO_POSTURA_CORRETA_X - VARIACAO_INFERIOR_X || angulo_x > ANGULO_POSTURA_CORRETA_X + VARIACAO_SUPERIOR_X) {
    contador_postura_incorreta = contador_postura_incorreta + 1;
  }

  if (contador_postura_incorreta == MAX_INCORRETA) {
    postura_ereta = false;
    contador_postura_incorreta = 0;
    enviar_bluetooth("{" + (String) "MSG" + (String) "Postura errada" + (String) "}");
  }
}

bool checaPosturaCorreta() {
  if (angulo_x > ANGULO_POSTURA_CORRETA_X - VARIACAO_INFERIOR_X && angulo_x < ANGULO_POSTURA_CORRETA_X + VARIACAO_SUPERIOR_X) {
    contador_postura_correta = contador_postura_correta + 1;
  }

  if (contador_postura_correta == MAX_CORRETA) {
    enviar_bluetooth("{" + (String) "MSG" + (String) "Postura Correta" + (String) "}");
    contador_postura_correta = 0;
    postura_ereta_2 = true;
    return true;
  }

  return false;
}

void leitura() {
  mpu6050.update();
  angulo_x = mpu6050.getAngleX();
  angulo_y = mpu6050.getAngleY();
  angulo_z = mpu6050.getAngleZ();

  //  enviar_bluetooth("{" + (String) "MPU" + (String) angulo_x + " " + (String) angulo_y + " " + (String) angulo_z + "}");

  //  if(calibracao) {
  //    enviar_bluetooth("{" + (String) "CAL"+ (String) angulo_x + "}");
  //  }

}

void atualizaValores() {

  //Leitura do valor do sensor EMG
  valor_emg_atual = analogRead(A0);

  //  if(operacao) {
  //    valores_emg[j] = valor_emg_atual;
  //
  //    if(j == 4) {
  //      int soma = 0;
  //      for(int i=0; i<5; i++) {
  //        soma = valores_emg[i];
  //      }
  //      soma = soma/5;
  //      enviar_bluetooth("{EMG"+ (String) soma + "}");
  //      j = -1;
  //    }
  //
  //    j++;
  //  }

  if (valor_emg_atual > FAIXA_EMG_ATIVACAO) {
    if (media_emg < CONTADOR_EMG ) {
      media_emg = media_emg + 1;
    }
  }

  if (valor_emg_atual < FAIXA_EMG_ATIVACAO) {
    if (media_emg > 0 ) {
      media_emg = media_emg - 1;
    }
  }

  //  enviar_bluetooth("{"+ (String) "EMG" + (String) media_emg + "}");
}

bool modo_operacao() {
  //dia a dia
  enviar_bluetooth("{MSGEntrou em modo_operacao}");
  while (operacao) {
    String comando = "";

    leitura();
    atualizaValores();

    if (postura_ereta) {
      checaPostura();
    }

    if (musculo_relaxado) {
      identificaFadigaMuscular();
    }

    if (!postura_ereta && musculo_relaxado) {
      bool corrigiu = checaPosturaCorreta();

      if (corrigiu) {
        postura_ereta = true;
        postura_ereta_2 = false;
      }
    }

    if (postura_ereta && !musculo_relaxado) {
      bool corrigiu = checaTerminouFadigaMuscular();

      if (corrigiu) {
        musculo_relaxado = true;
        musculo_relaxado_2 = false;
      }
    }

    if (!musculo_relaxado && !postura_ereta) {
      if (parada_aux) {
        parada_aux = false;
        digitalWrite(MOTOR_ESQUERDO, HIGH);
        enviar_bluetooth("{MOVincorreto}");
        delay(3000);
      }

      checaTerminouFadigaMuscular();
      checaPosturaCorreta();

      if (musculo_relaxado_2 && postura_ereta_2) {
        digitalWrite(MOTOR_ESQUERDO, LOW);
        resetaSistemaEMG();
        resetaSistemaGiro();
        parada_aux = true;
        enviar_bluetooth("{MOVanalise}");
      }
    }

    comando = receber_bluetooth();
    if (comando == "calibracao") {
      operacao = false;
      calibracao = true;
    }
    else if (comando == "resetar") {
      resetaSistemaEMG();
      resetaSistemaGiro();
      parada_aux = true;
      enviar_bluetooth("{MOVanalise}");
    }
    else if (comando == "resultado") {
      operacao = false;
      calibracao = false;
      aux = false;
      aux2 = false;
      return false;
    }
  }

  delay(20);

  return true;
}

bool iniciar_calibracao() {
  Serial.println("{MSGEntrou no modo de Calibracao}");
  String comando = "";
  valor_calibracao = 0.0;
  while (calibracao) {
    //checar envio para o celular
    comando = receber_bluetooth();
    if (comando == "operacao") {
      calibracao = false;
      operacao = true;
    }else if (comando == "fim_calibracao") {
      calibracao = false;
      operacao = true;
      //enviar média
    }
    else {
        aux_calibracao=true;
        leitura();
        //colocar dentro do while um auxiliar calibração
        
        while(aux_calibracao==true) {
          //ler mpu
          leitura();
          float variacao = 90 - angulo_x;
          if(variacao>20){
              se_inclinou=true;
            }
        Serial.println(variacao);
          if(variacao > valor_calibracao) {
            if(variacao >= 0 && variacao <= 90) {
              valor_calibracao = variacao;
            }
            
          }
          else{
            if(variacao<10&&se_inclinou){
              aux_calibracao=false;
              comando = "finalizar_exercicio";
              }
            }
          delay(20);
        }

        if (comando == "finalizar_exercicio") {
          Serial.println("{" + (String) "CAL" + (String) valor_calibracao + (String) "}");
          //enviar valor máximo pro app
          delay(3000);
          valor_calibracao = 0.0;
          se_inclinou=false;
        }
    } 
    delay(20);
  }

  return true;
}



String receber_bluetooth() {
  String  comando = "";
  if (SerialBT.available()) {
    while (SerialBT.available()) {
      char caracter = SerialBT.read();
      comando += caracter;
      delay(10);
    }

  }
  return comando;
}

void enviar_bluetooth(String mensagem) {
  int tamanho = mensagem.length();
  uint8_t mensagem_bluetooth[tamanho];

  for (int i = 0; i < tamanho; i++) {
    p[i] = (uint8_t) mensagem[i];
  }

  SerialBT.write((uint8_t*)&mensagem_bluetooth, tamanho);
}
