#include <MPU6050_tockn.h>
#include"BluetoothSerial.h"
#include <string>

BluetoothSerial SerialBT;
MPU6050 mpu6050(Wire);

//---------------------------------------------------------------------------//
//Example data:
int data_emg[64] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                   };

byte sine_data [91] =
{
  0,
  4,    9,    13,   18,   22,   27,   31,   35,   40,   44,
  49,   53,   57,   62,   66,   70,   75,   79,   83,   87,
  91,   96,   100,  104,  108,  112,  116,  120,  124,  127,
  131,  135,  139,  143,  146,  150,  153,  157,  160,  164,
  167,  171,  174,  177,  180,  183,  186,  189,  192,  195,
  198,  201,  204,  206,  209,  211,  214,  216,  219,  221,
  223,  225,  227,  229,  231,  233,  235,  236,  238,  240,
  241,  243,  244,  245,  246,  247,  248,  249,  250,  251,
  252,  253,  253,  254,  254,  254,  255,  255,  255,  255
};

float f_peaks[5]; // top 5 frequencies peaks in descending order
//---------------------------------------------------------------------------//

bool musculo_relaxado;
bool postura_ereta;

bool musculo_relaxado_2;
bool postura_ereta_2;

bool parada_aux;

bool aux;
bool aux2;

bool calibracao = false;
bool operacao = false;

float angulo_x;
float angulo_y;
float angulo_z;

int contador_postura_correta;
int contador_postura_incorreta;
int valor_calibracao;

boolean se_inclinou;

//Constantes

const int CONTADOR_EMG = 30;
const int FAIXA_EMG_ATIVACAO = 800;

const float ANGULO_POSTURA_CORRETA_X = 90.0;
const float VARIACAO_INFERIOR_X = 65.0;
const float VARIACAO_SUPERIOR_X = 20.0;
const int  MAX_CORRETA = 20;
const int  MAX_INCORRETA = 40;

const int PINO_EMG = 34;
const int PINO_MOTOR_ESQUERDO = 2;
const int PINO_MOTOR_DIREITO = 4;

void setup() {
  Serial.begin(9600);
  SerialBT.begin("Heroi Cuidador");

  pinMode(PINO_MOTOR_ESQUERDO, OUTPUT);
  pinMode(PINO_MOTOR_DIREITO, OUTPUT);

  digitalWrite(PINO_MOTOR_ESQUERDO, LOW);
  digitalWrite(PINO_MOTOR_DIREITO, LOW);

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

  se_inclinou = false;
}

void loop() {
  String  modo = receber_bluetooth();

  if (modo == "calibracao" || aux2) {
    calibracao = true;
    operacao = false;
    aux2 = false;
    aux = modo_calibracao();
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
  int amplitude_emg = (int) FFT(data_emg, 64, 50);

  if (amplitude_emg > FAIXA_EMG_ATIVACAO) {
    musculo_relaxado = false;
    enviar_bluetooth("{MSGComeçou Fadiga Muscular}");
  }
}

bool checaTerminouFadigaMuscular() {
  int amplitude_emg = (int) FFT(data_emg, 64, 50);

  if (amplitude_emg < FAIXA_EMG_ATIVACAO) {
    enviar_bluetooth("{MSGTerminou Fadiga Muscular}");
    musculo_relaxado_2 = true;
    return true;
  }

  return false;
}

//Musculo saiu da situação de fadiga
void resetaSistemaEMG() {
  //Desliga motor
  digitalWrite(PINO_MOTOR_ESQUERDO, LOW);
  enviar_bluetooth("{MSGTerminou Fadiga Muscular}");

  musculo_relaxado_2 = false;
  musculo_relaxado = true;
}

void resetaSistemaGiro() {
  // Desliga motor
  digitalWrite(PINO_MOTOR_ESQUERDO, LOW);
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

void leitura_mpu() {
  mpu6050.update();
  angulo_x = mpu6050.getAngleX();
  angulo_y = mpu6050.getAngleY();
  angulo_z = mpu6050.getAngleZ();
}

void leitura_emg() {

  //leitura do valor do sensor EMG
  int valor_emg_atual = analogRead(PINO_EMG);

  for (int i=64-1; i>0; i--) {  // delocamento do vetor de dados EMG
    data_emg[i] = data_emg[i - 1];
  }

  data_emg[0] = valor_emg_atual;  // insere o valor atual no vetor de dados EMG
}

bool modo_operacao() {
  //dia a dia
  enviar_bluetooth("{MSGEntrou em modo_operacao}");
  while (operacao) {
    String comando = "";

    leitura_mpu();
    leitura_emg();

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
        digitalWrite(PINO_MOTOR_ESQUERDO, HIGH);
        enviar_bluetooth("{MOVincorreto}");
        delay(3000);
      }

      checaTerminouFadigaMuscular();
      checaPosturaCorreta();

      if (musculo_relaxado_2 && postura_ereta_2) {
        digitalWrite(PINO_MOTOR_ESQUERDO, LOW);
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

bool modo_calibracao() {
  enviar_bluetooth("{MSGEntrou no modo de Calibracao}");
  String comando = "";
  valor_calibracao = 0.0;
  while (calibracao) {
    //checar envio para o celular
    comando = receber_bluetooth();
    if (comando == "operacao") {
      calibracao = false;
      operacao = true;
    } else if (comando == "fim_calibracao") {
      calibracao = false;
      operacao = true;
      //enviar média
    }
    else {
      boolean aux_calibracao = true;
      leitura_mpu();
      //colocar dentro do while um auxiliar calibração

      while (aux_calibracao == true) {
        //ler mpu
        leitura_mpu();
        float variacao = 90 - angulo_x;
        if (variacao > 20) {
          se_inclinou = true;
        }

        if (variacao > valor_calibracao) {
          if (variacao >= 0 && variacao <= 90) {
            valor_calibracao = variacao;
          }

        }
        else {
          if (variacao < 10 && se_inclinou) {
            aux_calibracao = false;
            comando = "finalizar_exercicio";
          }
        }
        delay(20);
      }

      if (comando == "finalizar_exercicio") {
        enviar_bluetooth("{" + (String) "CAL" + (String) valor_calibracao + (String) "}");
        //enviar valor máximo pro app
        delay(3000);
        valor_calibracao = 0.0;
        se_inclinou = false;
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
    mensagem_bluetooth[i] = (uint8_t) mensagem[i];
  }

  SerialBT.write((uint8_t*)&mensagem_bluetooth, tamanho);
}

//-----------------------------FFT Function----------------------------------------------//

float FFT(int in[], int N, float Frequency)
{
  unsigned int data[13] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048};
  int a, c1, f, o, x;
  a = N;

  for (int i = 0; i < 12; i++)          //calculating the levels
  {
    if (data[i] <= a) {
      o = i;
    }
  }

  int in_ps[data[o]] = {};   //input for sequencing
  float out_r[data[o]] = {}; //real part of transform
  float out_im[data[o]] = {}; //imaginory part of transform

  x = 0;
  for (int b = 0; b < o; b++)              // bit reversal
  {
    c1 = data[b];
    f = data[o] / (c1 + c1);
    for (int j = 0; j < c1; j++)
    {
      x = x + 1;
      in_ps[x] = in_ps[j] + f;
    }
  }


  for (int i = 0; i < data[o]; i++)     // update input array as per bit reverse order
  {
    if (in_ps[i] < a)
    {
      out_r[i] = in[in_ps[i]];
    }
    if (in_ps[i] > a)
    {
      out_r[i] = in[in_ps[i] - a];
    }
  }


  int i10, i11, n1;
  float e, c, s, tr, ti;

  for (int i = 0; i < o; i++)                             //fft
  {
    i10 = data[i];            // overall values of sine/cosine  :
    i11 = data[o] / data[i + 1]; // loop with similar sine cosine:
    e = 360 / data[i + 1];
    e = 0 - e;
    n1 = 0;

    for (int j = 0; j < i10; j++)
    {
      c = cosine(e * j);
      s = sine(e * j);
      n1 = j;

      for (int k = 0; k < i11; k++)
      {
        tr = c * out_r[i10 + n1] - s * out_im[i10 + n1];
        ti = s * out_r[i10 + n1] + c * out_im[i10 + n1];

        out_r[n1 + i10] = out_r[n1] - tr;
        out_r[n1] = out_r[n1] + tr;

        out_im[n1 + i10] = out_im[n1] - ti;
        out_im[n1] = out_im[n1] + ti;

        n1 = n1 + i10 + i10;
      }
    }
  }

  //---> here onward out_r contains amplitude and our_in conntains frequency (Hz)
  for (int i = 0; i < data[o - 1]; i++)      // getting amplitude from compex number
  {
    out_r[i] = sqrt(out_r[i] * out_r[i] + out_im[i] * out_im[i]); // to  increase the speed delete sqrt
    out_im[i] = i * Frequency / N;
  }

  x = 0;     // peak detection
  for (int i = 1; i < data[o - 1] - 1; i++)
  {
    if (out_r[i] > out_r[i - 1] && out_r[i] > out_r[i + 1])
    { in_ps[x] = i;  //in_ps array used for storage of peak number
      x = x + 1;
    }
  }


  s = 0;
  c = 0;
  for (int i = 0; i < x; i++)      // re arraange as per magnitude
  {
    for (int j = c; j < x; j++)
    {
      if (out_r[in_ps[i]] < out_r[in_ps[j]])
      { s = in_ps[i];
        in_ps[i] = in_ps[j];
        in_ps[j] = s;
      }
    }
    c = c + 1;
  }


  for (int i = 0; i < 5; i++) // updating f_peak array (global variable)with descending order
  {
    f_peaks[i] = out_im[in_ps[i]];
  }

  return out_r[0] / N;
}


float sine(int i)
{
  int j = i;
  float out;
  while (j < 0) {
    j = j + 360;
  }
  while (j > 360) {
    j = j - 360;
  }
  if (j > -1   && j < 91) {
    out = sine_data[j];
  }
  else if (j > 90  && j < 181) {
    out = sine_data[180 - j];
  }
  else if (j > 180 && j < 271) {
    out = -sine_data[j - 180];
  }
  else if (j > 270 && j < 361) {
    out = -sine_data[360 - j];
  }
  return (out / 255);
}

float cosine(int i)
{
  int j = i;
  float out;
  while (j < 0) {
    j = j + 360;
  }
  while (j > 360) {
    j = j - 360;
  }
  if (j > -1   && j < 91) {
    out = sine_data[90 - j];
  }
  else if (j > 90  && j < 181) {
    out = -sine_data[j - 90];
  }
  else if (j > 180 && j < 271) {
    out = -sine_data[270 - j];
  }
  else if (j > 270 && j < 361) {
    out = sine_data[j - 270];
  }
  return (out / 255);
}

//------------------------------------------------------------------------------------//
