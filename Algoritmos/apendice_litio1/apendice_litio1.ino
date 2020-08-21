#include <Wire.h>  //Biblioteca para comunicação I2C
#include <DS3232RTC.h>  //Biblioteca RTC DS3231
#include <Streaming.h>  //Biblioteca para impressão de variáveis
#include <Time.h> //Biblioteca relacionada à manipulação de tempo
#include <SD.h> //Biblioteca para escrita e leitura cartão SD
#include <SPI.h>  //Biblioteca para comunicação SPI
#include <Adafruit_ADS1015.h> //Biblioteca para uso do ADC ADS1115
#include <Adafruit_INA219.h> //Biblioteca para uso do sensor de corrente INA219

Adafruit_ADS1115 ads(0x48); //Endereçamento I2C do ADC para leitura dos canais de tensão
Adafruit_INA219 ina219(0x40); //Endereçamento I2C do sensor de corrente INA219

const int CS_PIN = 4; //Declaração do pino CS (chip select - SPI) no módulo SD

File file;  //Define "file" como comando para gravação de dados no cartão SD

//Definição dos pinos para manipulação do MUX nos canais de aquisição de tensão
int s0 = 10;
int s1 = 9;
int s2 = 8;

//Vetores de aquisição de tensão, tamanho do vetor indica número de canais habilitados
double AIN[8];
double v_lido[8];                              
double v_p[8];
double v_cell[8];

//Vaiáveis para aquisição de corrente
float corrente = 0;

//---------------- Temperatura ------------------------------------------------
const int SO = 6; //Definição pino MISO - Master Input, Slave Output - Para MAX6675
const int SCLK = 7; //Definição pino SCLK - Serial Clock line - Para MAX6675
const int CS1 = 5;  //Definição pino CS - Chip Select - Para MAX6675

double Temp[8]; //Vetor para aquisição de temperatura, tamanho do vetor indica número de canais habilitados 

//Definição dos pinos para manipulação do DEMUX nos canais de aquisição de temperatura
int A_t = 13;
int B_t = 12;
int C_t = 11;

byte SPImax(); //Declaração da função de comunicação SPI com MAX6675 por software
double leitura_termopar(int Selecao);

//-----------------------------------------------------------------------------

int intervalo = 2; //Tempo (em s) de amostragem das medidas (2s - 1 cell, 10 - multi)
int cont = 0; //Contador de amostras de medidas

int pausa = 5; //Pausa em us para alteração do canal do MUX/DEMUX

//----- Declaração variáveis SOC: EKF, contagem de Coulomb, e contagem modificada --------
float v_0; //Variável que recebe o valor de tensão do canal 1 (EKF aplicado)
float x_prev = 0; //Estimação prévia do SOC
float R_in = 0; //Resistência interna
float t_0 = 0;  //Variável para contagem de de tempo
float t_end = 0;  //Variável para contagem de de tempo
float t_end1 = 0; //Variável para contagem de de tempo
float t_end2 = 0; //Variável para contagem de de tempo
float t_end3 = 0; //Variável para contagem de de tempo
float x_0 = 100;  //Definição do SOC inicial e variável auxiliar na estimação SOC EKF
float ocv = 0;  //Tensão de circuito aberto
float C = 0;  //Parâmetro relacionado a matriz jacobiana EKF (Eq. 4.14)
float C1 = 0; //Variável auxiliar para cálculo de C
float C2 = 0; //Variável auxiliar para cálculo de C
float Q = 0.001;  //Covariância associada ao ruído de estado
float R = 0.0001;  //Covariância associada ao ruído do sensor
float L = 0;  //Ganho de Kalman
float P_prev = 0; //Covariância do erro de estimação prévia
float P_0 = 10000; //Covariância do erro de estimação inicial e atualizado
float cov_min = 10000;  //Parâmetro de covariância mínima associado à contagem modificada
float y_prev = 0; //Tensão terminal estimada
float x_pos = 0;  //SOC estimado atualizado
float x_posmx = 0;  //Limite superior de estimação SOC (intervalo de confiança)
float x_posmi = 0;  //Limite inferior de estimação SOC (intervalo de confiança)
float x_cc = 100; //Definição do SOC inicial para a estimação por contagem de Coulomb
float xcc_o = 0;  //SOC estimado pelo método de contagem de Coulomb modificada
float xc0 = 0;  //Definição inicial e variável auxiliar na estimação do SOC por contagem modificada
float cc = 0; //Variável auxiliar na contagem modificada: gatilho entre EKF e contagem modificada
int s = 0;  //Define sinal da corrente (carga ou descarga)
int st = 0; //Define atualização na contagem de capacidade de carga(DODc) e descarga (DOD)
float DOD = 0; //Capacidade de descarga em Ah
float DODc = 0; //Capacidade de carga em Ah
//----------------------------------------------------------------------------------------

//Função de declaração e inicialização (função padrão Arduino)
void setup(){
 
 //Define os pinos de manipulação MUX/DEMUX como saída
 pinMode(s0,OUTPUT);
 pinMode(s1,OUTPUT);
 pinMode(s2,OUTPUT);
 pinMode(A_t,OUTPUT);
 pinMode(B_t,OUTPUT);
 pinMode(C_t,OUTPUT);

 //Define pinos relacionados ao MAX6675 como saída ou entrada 
 pinMode(CS1,OUTPUT);
 pinMode(SCLK,OUTPUT);
 pinMode(SO,INPUT);
 digitalWrite(CS1,HIGH);

 //Inicializa os vetores de medição de tensão e temperatura
 for(int i = 0; i<=7; i++){
    AIN[i] = 0;
    v_lido[i] = 0;
    Temp[i] = 0; 
 }
 
 Serial.begin(115200);  //Inicia comunicação serial na taxa de 115200 bits/segundo

 inicia_SD(); //Inicialização do módulo SD
 
 //Comando da biblioteca Time.h para executar sincronização com o RTC DS3231
 setSyncProvider(RTC.get); //RTC.get é função da biblioteca DS3231.h e retorna tempo atual
 Serial << F("RTC Sync"); //Imprime na tela confirmação de sincronização
 if (timeStatus() != timeSet) Serial << F(" FALHA!"); //Imprime na tela erro de sincronização
 Serial << endl;

  //Configuração da resolução e ganho no uso do ADS1115
  //                                                                 ADS1115
  //Ganho ADC para aquisição de tensão                               -------
  //ads.setGain(GAIN_TWOTHIRDS);   // 2/3x ganho +/- 6.144V  1 bit = 0.1875mV 
  ads.setGain(GAIN_ONE);           // 1x ganho   +/- 4.096V  1 bit = 0.125mV 
  // ads.setGain(GAIN_TWO);        // 2x ganho   +/- 2.048V  1 bit = 0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x ganho   +/- 1.024V  1 bit = 0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x ganho   +/- 0.512V  1 bit = 0.015625mV 
  // ads.setGain(GAIN_SIXTEEN);    // 16x ganho  +/- 0.256V  1 bit = 0.0078125mV
  
  ads.begin();  //Início da operação do ADC de aquisição de tensão
  ina219.begin(); //Início da operação do sensor de corrente INA219

  Serial.println("Iniciando monitoramento ...");
}

//Função de repetição do Arduíno
void loop(){

  time_t t; //Variável tipo time (32 bits): número de segundos desde 1970  
  tmElements_t tm;  //Estrutura para manipulação de tempo

    //Configuração de data e hora por entrada serial
    //Formato reconhecido: ANO,MES,DIA,HORA,MINUTO,SEGUNDO
    if (Serial.available() >= 12) { //Verifica se existe alteração na data/hora via serial

        int y = Serial.parseInt();
        if (y >= 100 && y < 1000){
            Serial << F("Erro: Ano deve ter 2 ou 4 digitos") << endl;
        }else{
            if (y >= 1000)
                tm.Year = CalendarYrToTm(y);
            else
            tm.Year = y2kYearToTm(y);
            tm.Month = Serial.parseInt();
            tm.Day = Serial.parseInt();
            tm.Hour = Serial.parseInt();
            tm.Minute = Serial.parseInt();
            tm.Second = Serial.parseInt();
            t = makeTime(tm); //t recebe a data e hora da estrutura tm em segundos
            RTC.set(t); //Configura a data e hora do RTC para corresponder a t
            setTime(t); //Define a hora atual baseada em t
            Serial << F("RTC configurado para: ");
            printDateTime(t); //Imprime na tela data e hora configurada
            Serial << endl;
            while (Serial.available() > 0) Serial.read(); //Ignora qualquer entrada serial distinta da desejada
        }
    }

    monitoramento(t); //Função principal de Monitoramento: tensão, corrente, temperatura e SOC
       
}

//Função para aquisição de tensão, corrente, temperatura
//Estimação SOC
//Gravação de Dados
//Comunicação Serial
void monitoramento(time_t t){

  t_0 = millis();
  
  int16_t adc3; //Variavel associada à leitura de tensão ADS1115
  time_t anterior; //Variável auxiiar no tempo de amostragem
  time_t atual; //Variável auxiliar no tempo de amostragem
  anterior = now(); //Recebe tempo atual
  atual = now();  //Recebe tempo atual

  //Laço de repetição para amostragem dos sinais de tensão, corrente e temperatura
  while(abs(atual-anterior) < intervalo)
  {
    //CÉLULA 1
    digitalWrite(s0,HIGH);
    digitalWrite(s1,LOW); //Seleção do canal do MUX CD4051
    digitalWrite(s2,HIGH);

    delayMicroseconds(pausa);
    
    adc3 = ads.readADC_SingleEnded(3);
    v_lido[0] = (adc3*0.125)/1000;
    AIN[0] = AIN[0] + v_lido[0];

    //Para habilitar demais canais de tensão, retirar comentário
    /*
    //CÉLULA 2
    digitalWrite(s0,HIGH);
    digitalWrite(s1,HIGH);
    digitalWrite(s2,HIGH);

    delayMicroseconds(pausa);
    
    adc3 = ads.readADC_SingleEnded(3);
    v_lido[1] = (adc3*0.125)/1000;
    AIN[1] = AIN[1] + v_lido[1];
    
    //CÉLULA 3
    digitalWrite(s0,LOW);
    digitalWrite(s1,HIGH);
    digitalWrite(s2,HIGH);

    delayMicroseconds(pausa);
    
    adc3 = ads.readADC_SingleEnded(3);
    v_lido[2] = (adc3*0.125)/1000;
    AIN[2] = AIN[2] + v_lido[2];
    
    //CÉLULA 4
    digitalWrite(s0,LOW);
    digitalWrite(s1,LOW);
    digitalWrite(s2,HIGH);

    delayMicroseconds(pausa);
    
    adc3 = ads.readADC_SingleEnded(3);
    v_lido[3] = (adc3*0.125)/1000;
    AIN[3] = AIN[3] + v_lido[3];

    //CÉLULA 5
    digitalWrite(s0,HIGH);
    digitalWrite(s1,HIGH);
    digitalWrite(s2,LOW);

    delayMicroseconds(pausa);
    
    adc3 = ads.readADC_SingleEnded(3);
    v_lido[4] = (adc3*0.125)/1000;
    AIN[4] = AIN[4] + v_lido[4];

    //CÉLULA 6
    digitalWrite(s0,LOW);
    digitalWrite(s1,LOW);
    digitalWrite(s2,LOW);

    delayMicroseconds(pausa);
    
    adc3 = ads.readADC_SingleEnded(3);
    v_lido[5] = (adc3*0.125)/1000;
    AIN[5] = AIN[5] + v_lido[5];

    //CÉLULA 7
    digitalWrite(s0,HIGH);
    digitalWrite(s1,LOW);
    digitalWrite(s2,LOW);

    delayMicroseconds(pausa);
    
    adc3 = ads.readADC_SingleEnded(3);
    v_lido[6] = (adc3*0.125)/1000;
    AIN[6] = AIN[6] + v_lido[6];

    //CÉLULA 8
    digitalWrite(s0,LOW);
    digitalWrite(s1,HIGH);
    digitalWrite(s2,LOW);

    delayMicroseconds(pausa);
    
    adc3 = ads.readADC_SingleEnded(3);
    v_lido[7] = (adc3*0.125)/1000;
    AIN[7] = AIN[7] + v_lido[7];

    delayMicroseconds(pausa);
    */
    
    corrente = corrente + ina219.getCurrent_mA(); //Aquisição de corrente (mA) pelo sensor INA219
    
    //Para habilitar aquisição de temperatura, retirar comentário
    //temperatura(); //Função de aquisição de temperatura
        
    cont = cont +1;
    atual = now();
    
   }

  //Conversão do sinal de entrada do ADC para valores de tensão
  AIN[0] = AIN[0]/cont;
  v_p[0] = 2*AIN[0];

  AIN[1] = AIN[1]/cont;
  v_p[1] = 3.195*AIN[1];

  AIN[2] = AIN[2]/cont;
  v_p[2] = 4.019*AIN[2];

  AIN[3] = AIN[3]/cont;
  v_p[3] = 5.7296*AIN[3];

  AIN[4] = AIN[4]/cont;
  v_p[4] = 7.9203*AIN[4];

  AIN[5] = AIN[5]/cont;
  v_p[5] = 8.9333*AIN[5];

  AIN[6] = AIN[6]/cont;
  v_p[6] = 10.0268*AIN[6];

  AIN[7] = AIN[7]/cont;
  v_p[7] = 11.1652*AIN[7];

  //Estabelece a tensão terminal em cada canal de aquisição
  for(int i = 0; i <= 7; i++){
    if(v_p[i] < 0.1){
      v_p[i] = 0;
      v_cell[i] = 0;
    }else{
      v_cell[i] = v_p[i];
      if(i>0){
        for(int j = 0; j<i; j++){
          v_cell[i] = v_cell[i] - v_cell[j];
        }
      }
    }
  }

  //Estabelece a temperatura por canal
  for(int i = 0; i <= 7; i++){
    Temp[i] = Temp[i]/cont;
  }

  //Estabelece o valor de corrente em mA
  corrente = corrente/cont;

  //Define se a corrente é no sentido carga ou descarga
  if(corrente < 0)                  
  {
    s=1; //descarga
  }else{
    s=-1; //carga
  }

  //Calibração do sinal de corrente em mA
  corrente = s*(-0.00000006*pow(abs(corrente),2) + 0.98712717*abs(corrente) - 0.04186109);  

  if(abs(corrente)<10) //Descarta possíveis ruídos de medição
  {
    corrente = 0;
  }

  corrente = corrente/1000; //Corrente em A
  
  v_0 = v_cell[0];  //Tensão canal 1 habilitada para a estimativa SOC via EKF
    
  //------------------------- DESCARGA/REPOUSO -----------------------------
  if((v_0 > 1) && (corrente >= 0))
    {
      //Função responsável pela estimação do SOC na descarga via: EKF, contagem de Coulomb e contagem modificada
      EKF();
    }
  //----------------------------- CARGA ------------------------------------
  if(corrente < 0)
    {
      //Função responsável pela estimação do SOC na descarga: contagem de Coulomb
      Coulomb_carga(); 
    }
  //------------------------------------------------------------------------  
  t = now();
  //printDateTime(t); //Habilitar caso usuário não use interface Node-RED
  
  //Envio dos resultados por comunicação serial
  Serial.print("{\"V1\":");
  Serial.print(v_cell[0],4);
  Serial.print(",\"I\":");
  Serial.print(corrente,3);
  Serial.print(",\"x_pos\":");
  Serial.print(x_pos);
  Serial.print(",\"x_mx\":");
  Serial.print(x_posmx);
  Serial.print(",\"x_mn\":");
  Serial.print(x_posmi);
  Serial.print(",\"x_cc\":");
  Serial.print(x_cc);
  Serial.print(",\"x_otm\":");
  Serial.print(xcc_o);
  Serial.print(",\"DOD_d\":");
  Serial.print(DOD);
  Serial.print(",\"DOD_c\":");
  Serial.print(DODc);
  Serial.print(",\"y_ekf\":");
  Serial.print(y_prev,4);
  Serial.print(",\"delta_t\":");
  Serial.print(abs(t_0 - t_end1));
  Serial.print(",\"V2\":");
  Serial.print(v_cell[1],4);
  Serial.print(",\"V3\":");
  Serial.print(v_cell[2],4);
  Serial.print(",\"V4\":");
  Serial.print(v_cell[3],4);
  Serial.print(",\"V5\":");
  Serial.print(v_cell[4],4);
  Serial.print(",\"V6\":");
  Serial.print(v_cell[5],4);
  Serial.print(",\"V7\":");
  Serial.print(v_cell[6],4);
  Serial.print(",\"V8\":");
  Serial.print(v_cell[7],4);
  Serial.print(",\"cont\":");
  Serial.print(cont);
  Serial.print(",\"T1\":");
  Serial.print(Temp[0],1);
  Serial.print(",\"T2\":");
  Serial.print(Temp[1],1);
  Serial.print(",\"T3\":");
  Serial.print(Temp[2],1);
  Serial.print(",\"T4\":");
  Serial.print(Temp[3],1);
  Serial.print(",\"T5\":");
  Serial.print(Temp[4],1);
  Serial.print(",\"T6\":");
  Serial.print(Temp[5],1);
  Serial.print(",\"T7\":");
  Serial.print(Temp[6],1);
  Serial.print(",\"T8\":");
  Serial.print(Temp[7],1);
  Serial.println("}");
  
  abre_arquivo("DADOS.txt");  //Abre (ou cria) arquivo de texto DADOS.txt no cartão SD
  
  grava_tempo(t); //Grava data e hora no cartão SD
  //Dados gravados no cartão SD
  //Dados BMS unicelular com EKF
  file.print(" ADC1: ");
  file.print(AIN[0],4);
  file.print(" Ponto A: ");
  file.print(v_p[0],4);
  file.print(" V1: ");
  file.print(v_0,4);
  file.print(" Vekf: ");
  file.print(y_prev,4);
  file.print(" SOC: ");
  file.print(x_pos,4);
  file.print(" ");
  file.print(x_posmx,4);
  file.print(" ");
  file.print(x_posmi,4);
  file.print(" SOCcc: ");
  file.print(x_cc,4);
  file.print(" SOCccot: ");
  file.print(xcc_o,4);
  file.print(" DODd (mAh): ");
  file.print(DOD);
  file.print(" DODc (mAh): ");
  file.print(DODc);
  file.print(" delta_t (ms): ");
  file.print(abs(t_0 - t_end1));
  
  //Dados BMS multicelular
  //Comentar caso não tenha necessidade dos canais de tensão e temperatura
  file.print(" V2: ");
  file.print(v_cell[1],4);
  file.print(" V3: ");
  file.print(v_cell[2],4);
  file.print(" V4: ");
  file.print(v_cell[3],4);
  file.print(" V5: ");
  file.print(v_cell[4],4);
  file.print(" V6 ");
  file.print(v_cell[5],4);
  file.print(" V7: ");
  file.print(v_cell[6],4);
  file.print(" V8: ");
  file.print(v_cell[7],4);
  file.print(" cont: ");
  file.print(cont);
  file.print(" T1(C): ");
  file.print(Temp[0],1);
  file.print(" T2: ");
  file.print(Temp[1],1);
  file.print(" T3: ");
  file.print(Temp[2],1);
  file.print(" T4: ");
  file.print(Temp[3],1);
  file.print(" T5: ");
  file.print(Temp[4],1);
  file.print(" T6: ");
  file.print(Temp[5],1);
  file.print(" T7: ");
  file.print(Temp[6],1);
  file.print(" T8: ");
  file.print(Temp[7],1);
  
  file.print("\tI(A): ");
  file.println(corrente,3);

  fecha_arquivo();  //Fecha arquivo DADOS.txt

  //Reinicia variáves
  for(int i = 0; i<=7; i++){
    AIN[i] = 0;
    v_lido[i] = 0;
    Temp[i] = 0; 
  }
  
  corrente = 0;
  cont = 0; 
}

//Função de inicialização do cartão SD
void inicia_SD()
{
  pinMode(CS_PIN, OUTPUT); //Define o pino CS_PIN como saída
  //Verifica se o cartão SD está presente
  if (SD.begin()){
    
  }else{
    Serial.println("Erro no cartao"); //Mensagem de erro
    return;
  }
}

//Função para abrir/criar arquivo de gravação
int abre_arquivo(char filename[])
{
  file = SD.open(filename, FILE_WRITE);
  //Verifica se existe erro no arquivo
  if(file)
  {
    return 1;
  }else{
    Serial.println("Erro na gravacao"); //Mensagem de erro
    return 0;
  }
}

//Função para fechar arquivo de gravação
void fecha_arquivo()
{
  if (file)
  {
    file.close();
  }
}

//Função para gravar data e hora no arquivo do cartão SD
void grava_tempo(time_t t)
 {
   file.print(day(t));
   file.print("/");
   if (month(t) < 10)
   {
     file.print("0");
   }
   file.print(month(t));
   file.print("/");
   file.print(year(t));
   file.print(" Hora: ");
   if (hour(t) < 10)
   {
     file.print("0");
   }
   file.print(hour(t));
   file.print(":");
   if (minute(t) < 10)
   {
     file.print("0");
   }
   file.print(minute(t));
   file.print(":");
   if (second(t) < 10)
   {
     file.print("0");
   }
   file.print(second(t)); 
  }
  
//Função que imprime data e hora na serial
void printDateTime(time_t t)
{
    printDate(t); //Data
    Serial << ' ';
    printTime(t); //Hora
}

//Função que define formato de impressão da hora
void printTime(time_t t)
{
    printI00(hour(t), ':');
    printI00(minute(t), ':');
    printI00(second(t), ' ');
}

//Função que define formato de impressão da data
void printDate(time_t t)
{
    printI00(day(t), 0);
    Serial << monthShortStr(month(t)) << _DEC(year(t));
}

//Função auxiliar na impressão de hora, substitui "0" por "00"
void printI00(int val, char delim)
{
    if (val < 10) Serial << '0';
    Serial << _DEC(val);
    if (delim > 0) Serial << delim;
    return;
}

//Função para aquisição de temperatura
void temperatura(){
    
    //SENSOR 1
    digitalWrite(A_t,HIGH);
    digitalWrite(B_t,LOW);  //Definição do canal do DEMUX CD4051
    digitalWrite(C_t,HIGH);
    
    delayMicroseconds(5);
    
    Temp[0] = Temp[0] + leitura_termopar(CS1);
  
    //SENSOR 2
    digitalWrite(A_t,HIGH);
    digitalWrite(B_t,HIGH);
    digitalWrite(C_t,HIGH);
    
    delayMicroseconds(5);
    
    Temp[1] = Temp[1] + leitura_termopar(CS1);
   
    //SENSOR 3
    digitalWrite(A_t,LOW);
    digitalWrite(B_t,HIGH);
    digitalWrite(C_t,HIGH);
  
    delayMicroseconds(5);
  
    Temp[2] = Temp[2] + leitura_termopar(CS1);
  
    //SENSOR 4
    digitalWrite(A_t,LOW);
    digitalWrite(B_t,LOW);
    digitalWrite(C_t,HIGH);
  
    delayMicroseconds(5);
  
    Temp[3] = Temp[3] + leitura_termopar(CS1);

    //SENSOR 5
    digitalWrite(A_t,LOW);
    digitalWrite(B_t,HIGH);
    digitalWrite(C_t,LOW);
  
    delayMicroseconds(5);
  
    Temp[4] = Temp[4] + leitura_termopar(CS1);

    //SENSOR 6
    digitalWrite(A_t,HIGH);
    digitalWrite(B_t,LOW);
    digitalWrite(C_t,LOW);
  
    delayMicroseconds(5);
  
    Temp[5] = Temp[5] + leitura_termopar(CS1);

    //SENSOR 7
    digitalWrite(A_t,LOW);
    digitalWrite(B_t,LOW);
    digitalWrite(C_t,LOW);
  
    delayMicroseconds(5);
  
    Temp[6] = Temp[6] + leitura_termopar(CS1);

    //SENSOR 8
    digitalWrite(A_t,HIGH);
    digitalWrite(B_t,HIGH);
    digitalWrite(C_t,LOW);
  
    delayMicroseconds(5);
  
    Temp[7] = Temp[7] + leitura_termopar(CS1);
    
}

//Função para leitura de temperatura MAX6675
double leitura_termopar(int Selecao) {
  uint16_t max_temp;  //Declara variável de 16 bits que recebe os bits lidos na porta SO
  digitalWrite(Selecao, LOW); //Inicia transmissão de dados
  delay(1);
  max_temp = SPImax();  //Recebe a leitura dos 8 primeiros bits lidos pela função SPImax
  max_temp <<= 8; //Desloca 8 posições para a esquerda: D15-D8
  max_temp |= SPImax(); //Recebe a leitura dos 8 últimos bits lidos pela função SPImax - D7-D0
  digitalWrite(Selecao, HIGH); //Finaliza transmissão de dados
 
  //Verifica se o termpopar esta conectado: bit 2 (D2) deve ser igual a 4
  if (max_temp & 0x4) {
    return NAN; //Caso não conectado retorna NAN
  }
  max_temp >>= 3; //Desloca para a direita os 3 primeiros bits, eliminando-os, leitura vai de D3 até D14
  return max_temp*0.25; //Coverte leitura pela resolução do MAX6675
}

//Função para cmunicação SPI MAX6675 – leitura dos bits na saída SO (1 byte – 8 bits)
byte SPImax(void) {
  int i;
  byte d_temp = 0; //Variável byte que vai armazenar a leitura
 
  //Laço para ler 8 bits de cada vez
  for (i = 7; i >= 0; i--)    
  {
    digitalWrite(SCLK, LOW);  //Início do clock (SCK)
    delay(1);
    if (digitalRead(SO)) {  //Leitura dos dados da porta SO do MAX6675
      d_temp |= (1 << i); //Armazena os bits, 0 ou 1,deslocando para esquerda até formar o byte
    }
    digitalWrite(SCLK, HIGH); //Fim do clock (SCK)
    delay(1);
  }
  return d_temp;  //Retorna o byte de leitura do MAX6675
}

//Função que implementa o EKF, contagem modificada, e contagem de Coulomb (DESCARGA/REPOUSO)
void EKF(){   
    t_end = millis();
    //Estimação previa SOC: primeira equação do modelo (contagem de Coulomb)
    x_prev = x_0 - ((abs(t_end - t_0)+abs(t_0 - t_end1))/3.6e06)*(corrente/2.574)*100;
    //Estimação SOC por contagem de Coulomb
    x_cc = x_cc - ((abs(t_end - t_0)+abs(t_0 - t_end1))/3.6e06)*(corrente/2.574)*100;

    if((st==0) && (corrente>0)) //Redefine parâmetros após carga
    {
      DOD = 0; //Capacidade de descarga reiniciada após processo de carga (st==0)
      st=1; //Carrega informação do processo atual de descarga
    }
    //Calcula mAh descarregado
    DOD = DOD + ((abs(t_end - t_0)+abs(t_0 - t_end1))/3.6e06)*corrente*1000;
    t_end1 = millis();
    
    //Limita SOC previo na faixa 0%-100%
    if(x_prev>100)
      {
        x_prev = 100;
      }
    
    if(x_prev<0)
      {
        x_prev = 0;
      }

    //-------------------------------- Cálculo da Resistencia interna -------------------------------------------------- 
    //Rin em função do SOC prévio e corrente medida
    if(x_prev<=10)
       {      
          R_in = 0.3151 + (-0.03611)*((x_prev-5.823)/2.711) + 0.005852*((corrente-0.747)/0.2741) + (-0.01819)*pow(((x_prev-5.823)/2.711),2) + (-0.01725)*((x_prev-5.823)/2.711)*((corrente-0.747)/0.2741) + 0.007562*pow(((corrente-0.747)/0.2741),2) + 0.01651*pow(((x_prev-5.823)/2.711),3) + 0.02735*pow(((x_prev-5.823)/2.711),2)*((corrente-0.747)/0.2741) + (-0.00206)*((x_prev-5.823)/2.711)*pow(((corrente-0.747)/0.2741),2) + 0.003966*pow(((corrente-0.747)/0.2741),3) + 0.00395*pow(((x_prev-5.823)/2.711),4) + (-0.01219)*pow(((x_prev-5.823)/2.711),3)*((corrente-0.747)/0.2741) + 0.0009657*pow(((x_prev-5.823)/2.711),2)*pow(((corrente-0.747)/0.2741),2) + (-0.005224)*((x_prev-5.823)/2.711)*pow(((corrente-0.747)/0.2741),3) + (-0.001676)*pow(((corrente-0.747)/0.2741),4) + (-0.002886)*pow(((x_prev-5.823)/2.711),5) + (2.973e-05)*pow(((x_prev-5.823)/2.711),4)*((corrente-0.747)/0.2741) + (-0.0003836)*pow(((x_prev-5.823)/2.711),3)*pow(((corrente-0.747)/0.2741),2) + 0.001074*pow(((x_prev-5.823)/2.711),2)*pow(((corrente-0.747)/0.2741),3) + 0.001075*((x_prev-5.823)/2.711)*pow(((corrente-0.747)/0.2741),4);
       }
 
    
    if(x_prev>10 && x_prev<=25)
      {    
          R_in = 0.2703 + 0.007529*((x_prev-17.5)/4.33) + 0.007854*((corrente-0.8943)/0.3587) + 0.003017*pow(((x_prev-17.5)/4.33),2) + (-0.004974)*((x_prev-17.5)/4.33)*((corrente-0.8943)/0.3587) + (-0.0006822)*pow(((corrente-0.8943)/0.3587),2) + (-0.003448)*pow(((x_prev-17.5)/4.33),3) + (-0.003636)*pow(((x_prev-17.5)/4.33),2)*((corrente-0.8943)/0.3587) + 0.002376*((x_prev-17.5)/4.33)*pow(((corrente-0.8943)/0.3587),2) + 0.0001759*pow(((corrente-0.8943)/0.3587),3) + (-0.0006072)*pow(((x_prev-17.5)/4.33),4) + 0.001206*pow(((x_prev-17.5)/4.33),3)*((corrente-0.8943)/0.3587) + 0.001659*pow(((x_prev-17.5)/4.33),2)*pow(((corrente-0.8943)/0.3587),2) + 0.0002326*((x_prev-17.5)/4.33)*pow(((corrente-0.8943)/0.3587),3) + (-0.0007371)*pow(((corrente-0.8943)/0.3587),4) + 0.000641*pow(((x_prev-17.5)/4.33),5) + 0.0007264*pow(((x_prev-17.5)/4.33),4)*((corrente-0.8943)/0.3587) + (-0.000805)*pow(((x_prev-17.5)/4.33),3)*pow(((corrente-0.8943)/0.3587),2) + (-0.0005324)*pow(((x_prev-17.5)/4.33),2)*pow(((corrente-0.8943)/0.3587),3) + (-0.0001295)*((x_prev-17.5)/4.33)*pow(((corrente-0.8943)/0.3587),4);
      }
    
    if(x_prev>25 && x_prev<=50)
       {       
          R_in = 0.2569 + (-0.01361)*((x_prev-37.5)/7.218) + 0.002375*((corrente-0.8942)/0.3587) + 0.004825*pow(((x_prev-37.5)/7.218),2) + 0.002651*((x_prev-37.5)/7.218)*((corrente-0.8942)/0.3587) + 0.0006763*pow(((corrente-0.8942)/0.3587),2) + 0.002141*pow(((x_prev-37.5)/7.218),3) + (-0.001762)*pow(((x_prev-37.5)/7.218),2)*((corrente-0.8942)/0.3587) + (-0.001722)*((x_prev-37.5)/7.218)*pow(((corrente-0.8942)/0.3587),2) + (-0.001647)*pow(((corrente-0.8942)/0.3587),3) + (-0.000695)*pow(((x_prev-37.5)/7.218),4) + (-0.000845)*pow(((x_prev-37.5)/7.218),3)*((corrente-0.8942)/0.3587) + 0.0006235*pow(((x_prev-37.5)/7.218),2)*pow(((corrente-0.8942)/0.3587),2) + (-0.0008456)*((x_prev-37.5)/7.218)*pow(((corrente-0.8942)/0.3587),3) + 0.0004218*pow(((corrente-0.8942)/0.3587),4) + (-2.97e-05)*pow(((x_prev-37.5)/7.218),5) + 0.0002467*pow(((x_prev-37.5)/7.218),4)*((corrente-0.8942)/0.3587) + 0.0002202*pow(((x_prev-37.5)/7.218),3)*pow(((corrente-0.8942)/0.3587),2) + (-0.0001799)*pow(((x_prev-37.5)/7.218),2)*pow(((corrente-0.8942)/0.3587),3) + 0.0007599*((x_prev-37.5)/7.218)*pow(((corrente-0.8942)/0.3587),4);
       }
    
    if(x_prev>50 && x_prev<=90)
      {     
          R_in = 0.2527 + (-0.01543)*((x_prev-70)/11.55) + 0.0006656*((corrente-0.8945)/0.3589) + (-0.003163)*pow(((x_prev-70)/11.55),2) + 0.006117*((x_prev-70)/11.55)*((corrente-0.8945)/0.3589) + (-0.001499)*pow(((corrente-0.8945)/0.3589),2) + 0.004131*pow(((x_prev-70)/11.55),3) + (-0.001866)*pow(((x_prev-70)/11.55),2)*((corrente-0.8945)/0.3589) + (-0.001238)*((x_prev-70)/11.55)*pow(((corrente-0.8945)/0.3589),2) + (-0.0004141)*pow(((corrente-0.8945)/0.3589),3) + (-0.0006509)*pow(((x_prev-70)/11.55),4) + (-0.001221)*pow(((x_prev-70)/11.55),3)*((corrente-0.8945)/0.3589) + 0.0006916*pow(((x_prev-70)/11.55),2)*pow(((corrente-0.8945)/0.3589),2) + 0.0008269*((x_prev-70)/11.55)*pow(((corrente-0.8945)/0.3589),3) + 0.0004386*pow(((corrente-0.8945)/0.3589),4) + (-0.0005025)*pow(((x_prev-70)/11.55),5) + 0.0007932*pow(((x_prev-70)/11.55),4)*((corrente-0.8945)/0.3589) + (1.523e-05)*pow(((x_prev-70)/11.55),3)*pow(((corrente-0.8945)/0.3589),2) + (-7.907e-05)*pow(((x_prev-70)/11.55),2)*pow(((corrente-0.8945)/0.3589),3) + (-0.0002693)*((x_prev-70)/11.55)*pow(((corrente-0.8945)/0.3589),4);
      }

    if(x_prev>90 && x_prev<=100)
      {      
          R_in = 0.2206 + (-0.002433)*((x_prev-95)/2.89) + 0.005412*((corrente-0.8959)/0.3595) + 0.003349*pow(((x_prev-95)/2.89),2) + (-0.0005536)*((x_prev-95)/2.89)*((corrente-0.8959)/0.3595) + (-0.003426)*pow(((corrente-0.8959)/0.3595),2) + 0.001748*pow(((x_prev-95)/2.89),3) + (-0.001878)*pow(((x_prev-95)/2.89),2)*((corrente-0.8959)/0.3595) + (-0.0007936)*((x_prev-95)/2.89)*pow(((corrente-0.8959)/0.3595),2) + 0.003316*pow(((corrente-0.8959)/0.3595),3) + (-0.002214)*pow(((x_prev-95)/2.89),4) + 0.0004109*pow(((x_prev-95)/2.89),3)*((corrente-0.8959)/0.3595) + 0.0003828*pow(((x_prev-95)/2.89),2)*pow(((corrente-0.8959)/0.3595),2) + (-0.0003747)*((x_prev-95)/2.89)*pow(((corrente-0.8959)/0.3595),3) + (-0.0008102)*pow(((corrente-0.8959)/0.3595),4) + (-0.001346)*pow(((x_prev-95)/2.89),5) + 0.000345*pow(((x_prev-95)/2.89),4)*((corrente-0.8959)/0.3595) + (-8.352e-05)*pow(((x_prev-95)/2.89),3)*pow(((corrente-0.8959)/0.3595),2) + 0.0001859*pow(((x_prev-95)/2.89),2)*pow(((corrente-0.8959)/0.3595),3) + 0.0004602*((x_prev-95)/2.89)*pow(((corrente-0.8959)/0.3595),4);
      }
    
    //-------------------------------- Cálculo da OCV -------------------------------------------------- 
    //OCV como função do SOC prévio
    if(x_prev<=10)
      {          
          ocv = (-1.029)*pow(((x_prev-5.001)/2.888),9) + (-0.6268)*pow(((x_prev-5.001)/2.888),8) + 10.81*pow(((x_prev-5.001)/2.888),7) + (-1.151)*pow(((x_prev-5.001)/2.888),6) + (-33.57)*pow(((x_prev-5.001)/2.888),5) + 9.933*pow(((x_prev-5.001)/2.888),4) + 60.02*pow(((x_prev-5.001)/2.888),3) + (-64.76)*pow(((x_prev-5.001)/2.888),2) + 36*((x_prev-5.001)/2.888) + 3440;
      }
    
    if(x_prev>10)
      {
          ocv = 1.572*pow(((x_prev-55.01)/25.98),8) + (-3.833)*pow(((x_prev-55.01)/25.98),7) + (-5.709)*pow(((x_prev-55.01)/25.98),6) + 28.48*pow(((x_prev-55.01)/25.98),5) + (-13.67)*pow(((x_prev-55.01)/25.98),4) + (-48.59)*pow(((x_prev-55.01)/25.98),3) + 89.34*pow(((x_prev-55.01)/25.98),2) + 199.7*((x_prev-55.01)/25.98) + 3702;
      }

    //-------------------------------- Calculo do parâmetro C -----------------------------------------------------   
    //Calculado em duas partes:
    //C = d(yk)/d(xk) com xk=xkprev, C = C2-C1, C1 = d(Rin(xk)*i)/dxk
    //C2 = d(ocv(xk))/dxk
    
    if(x_prev<=10)
      {
          C1 = (1/2.711)*(-0.03611) + 2*(1/2.711)*(-0.01819)*((x_prev-5.823)/2.711) + (1/2.711)*(-0.01725)*((corrente-0.747)/0.2741) + 3*(1/2.711)*0.01651*pow(((x_prev-5.823)/2.711),2) + 2*(1/2.711)*0.02735*((x_prev-5.823)/2.711)*((corrente-0.747)/0.2741) + (1/2.711)*(-0.00206)*pow(((corrente-0.747)/0.2741),2) + 4*(1/2.711)*0.00395*pow(((x_prev-5.823)/2.711),3) + 3*(1/2.711)*(-0.01219)*pow(((x_prev-5.823)/2.711),2)*((corrente-0.747)/0.2741) + 2*(1/2.711)*0.0009657*((x_prev-5.823)/2.711)*pow(((corrente-0.747)/0.2741),2) + (1/2.711)*(-0.005224)*pow(((corrente-0.747)/0.2741),3) + 5*(1/2.711)*(-0.002886)*pow(((x_prev-5.823)/2.711),4) + 4*(1/2.711)*(2.973e-05)*pow(((x_prev-5.823)/2.711),3)*((corrente-0.747)/0.2741) + 3*(1/2.711)*(-0.0003836)*pow(((x_prev-5.823)/2.711),2)*pow(((corrente-0.747)/0.2741),2) + 2*(1/2.711)*0.001074*((x_prev-5.823)/2.711)*pow(((corrente-0.747)/0.2741),3) + (1/2.711)*0.001075*pow(((corrente-0.747)/0.2741),4);
      }
    
    if(x_prev>10 && x_prev<=25)
      {             
          C1 = (1/4.33)*0.007529 + 2*(1/4.33)*0.003017*((x_prev-17.5)/4.33) + (1/4.33)*(-0.004974)*((corrente-0.8943)/0.3587) + 3*(1/4.33)*(-0.003448)*pow(((x_prev-17.5)/4.33),2) + 2*(1/4.33)*(-0.003636)*((x_prev-17.5)/4.33)*((corrente-0.8943)/0.3587) + (1/4.33)*0.002376*pow(((corrente-0.8943)/0.3587),2) + 4*(1/4.33)*(-0.0006072)*pow(((x_prev-17.5)/4.33),3) + 3*(1/4.33)*0.001206*pow(((x_prev-17.5)/4.33),2)*((corrente-0.8943)/0.3587) + 2*(1/4.33)*0.001659*((x_prev-17.5)/4.33)*pow(((corrente-0.8943)/0.3587),2) + (1/4.33)*0.0002326*pow(((corrente-0.8943)/0.3587),3) + 5*(1/4.33)*0.000641*pow(((x_prev-17.5)/4.33),4) + 4*(1/4.33)*0.0007264*pow(((x_prev-17.5)/4.33),3)*((corrente-0.8943)/0.3587) + 3*(1/4.33)*(-0.000805)*pow(((x_prev-17.5)/4.33),2)*pow(((corrente-0.8943)/0.3587),2) + 2*(1/4.33)*(-0.0005324)*((x_prev-17.5)/4.33)*pow(((corrente-0.8943)/0.3587),3) + (1/4.33)*(-0.0001295)*pow(((corrente-0.8943)/0.3587),4);
      }
    
    if(x_prev>25 && x_prev<=50)
      {      
          C1 = (1/7.218)*(-0.01361) + 2*(1/7.218)*0.004825*((x_prev-37.5)/7.218) + (1/7.218)*0.002651*((corrente-0.8942)/0.3587) + 3*(1/7.218)*0.002141*pow(((x_prev-37.5)/7.218),2) + 2*(1/7.218)*(-0.001762)*((x_prev-37.5)/7.218)*((corrente-0.8942)/0.3587) + (1/7.218)*(-0.001722)*pow(((corrente-0.8942)/0.3587),2) + 4*(1/7.218)*(-0.000695)*pow(((x_prev-37.5)/7.218),3) + 3*(1/7.218)*(-0.000845)*pow(((x_prev-37.5)/7.218),2)*((corrente-0.8942)/0.3587) + 2*(1/7.218)*0.0006235*((x_prev-37.5)/7.218)*pow(((corrente-0.8942)/0.3587),2) + (1/7.218)*(-0.0008456)*pow(((corrente-0.8942)/0.3587),3) + 5*(1/7.218)*(-2.97e-05)*pow(((x_prev-37.5)/7.218),4) + 4*(1/7.218)*0.0002467*pow(((x_prev-37.5)/7.218),3)*((corrente-0.8942)/0.3587) + 3*(1/7.218)*0.0002202*pow(((x_prev-37.5)/7.218),2)*pow(((corrente-0.8942)/0.3587),2) + 2*(1/7.218)*(-0.0001799)*((x_prev-37.5)/7.218)*pow(((corrente-0.8942)/0.3587),3) + (1/7.218)*0.0007599*pow(((corrente-0.8942)/0.3587),4);
      }
    
    if(x_prev>50 && x_prev<=90)
      {     
          C1 = (1/11.55)*(-0.01543) + 2*(1/11.55)*(-0.003163)*((x_prev-70)/11.55) + (1/11.55)*0.006117*((corrente-0.8945)/0.3589) + 3*(1/11.55)*0.004131*pow(((x_prev-70)/11.55),2) + 2*(1/11.55)*(-0.001866)*((x_prev-70)/11.55)*((corrente-0.8945)/0.3589) + (1/11.55)*(-0.001238)*pow(((corrente-0.8945)/0.3589),2) + 4*(1/11.55)*(-0.0006509)*pow(((x_prev-70)/11.55),3) + 3*(1/11.55)*(-0.001221)*pow(((x_prev-70)/11.55),2)*((corrente-0.8945)/0.3589) + 2*(1/11.55)*0.0006916*((x_prev-70)/11.55)*pow(((corrente-0.8945)/0.3589),2) + (1/11.55)*0.0008269*pow(((corrente-0.8945)/0.3589),3) + 5*(1/11.55)*(-0.0005025)*pow(((x_prev-70)/11.55),4) + 4*(1/11.55)*0.0007932*pow(((x_prev-70)/11.55),3)*((corrente-0.8945)/0.3589) + 3*(1/11.55)*(1.523e-05)*pow(((x_prev-70)/11.55),2)*pow(((corrente-0.8945)/0.3589),2) + 2*(1/11.55)*(-7.907e-05)*((x_prev-70)/11.55)*pow(((corrente-0.8945)/0.3589),3) + (1/11.55)*(-0.0002693)*pow(((corrente-0.8945)/0.3589),4);
      }

    if(x_prev>90 && x_prev<=100)
      {     
          C1 = (1/2.89)*(-0.002433) + 2*(1/2.89)*0.003349*((x_prev-95)/2.89) + (1/2.89)*(-0.0005536)*((corrente-0.8959)/0.3595) + 3*(1/2.89)*0.001748*pow(((x_prev-95)/2.89),2) + 2*(1/2.89)*(-0.001878)*((x_prev-95)/2.89)*((corrente-0.8959)/0.3595) + (1/2.89)*(-0.0007936)*pow(((corrente-0.8959)/0.3595),2) + 4*(1/2.89)*(-0.002214)*pow(((x_prev-95)/2.89),3) + 3*(1/2.89)*0.0004109*pow(((x_prev-95)/2.89),2)*((corrente-0.8959)/0.3595) + 2*(1/2.89)*0.0003828*((x_prev-95)/2.89)*pow(((corrente-0.8959)/0.3595),2) + (1/2.89)*(-0.0003747)*pow(((corrente-0.8959)/0.3595),3) + 5*(1/2.89)*(-0.001346)*pow(((x_prev-95)/2.89),4) + 4*(1/2.89)*0.000345*pow(((x_prev-95)/2.89),3)*((corrente-0.8959)/0.3595) + 3*(1/2.89)*(-8.352e-05)*pow(((x_prev-95)/2.89),2)*pow(((corrente-0.8959)/0.3595),2) + 2*(1/2.89)*0.0001859*((x_prev-95)/2.89)*pow(((corrente-0.8959)/0.3595),3) + (1/2.89)*0.0004602*pow(((corrente-0.8959)/0.3595),4);
      }
      
    if(x_prev<=10)
      {     
          C2 = 9*(1/2.888)*(-1.029)*pow(((x_prev-5.001)/2.888),8) + 8*(1/2.888)*(-0.6268)*pow(((x_prev-5.001)/2.888),7) + 7*(1/2.888)*10.81*pow(((x_prev-5.001)/2.888),6) + 6*(1/2.888)*(-1.151)*pow(((x_prev-5.001)/2.888),5) + 5*(1/2.888)*(-33.57)*pow(((x_prev-5.001)/2.888),4) + 4*(1/2.888)*9.933*pow(((x_prev-5.001)/2.888),3) + 3*(1/2.888)*60.02*pow(((x_prev-5.001)/2.888),2) + 2*(1/2.888)*(-64.76)*((x_prev-5.001)/2.888) + (1/2.888)*36;
          C2 = C2/1000;
      }   
      
   if(x_prev>10)
      {            
          C2 = 8*(1/25.98)*1.572*pow(((x_prev-55.01)/25.98),7) + 7*(1/25.98)*(-3.833)*pow(((x_prev-55.01)/25.98),6) + 6*(1/25.98)*(-5.709)*pow(((x_prev-55.01)/25.98),5) + 5*(1/25.98)*28.48*pow(((x_prev-55.01)/25.98),4) + 4*(1/25.98)*(-13.67)*pow(((x_prev-55.01)/25.98),3) + 3*(1/25.98)*(-48.59)*pow(((x_prev-55.01)/25.98),2) + 2*(1/25.98)*89.34*((x_prev-55.01)/25.98) + (1/25.98)*199.7;
          C2 = C2/1000;
      }  
    
    C = C2-C1*corrente;
    //-------------------------------------------------------------------------------------------------------------------
    y_prev = (ocv/1000) - R_in*corrente; //Tensão terminal estimada a partir do SOC prévio
                
    P_prev = 1*P_0*1+Q; //Covariância do erro na estimativa a priori do estado
    
    L = (P_prev*C)/(C*P_prev*C+R); //Ganho de Kalman

    //Atualização do estado estimado   
    x_pos = x_prev + ((P_prev*C)/(C*P_prev*C+R))*(v_0 - y_prev);

    if(x_pos>100)
      {
        x_pos = 100;
      }

    if(x_pos<0)
      {
        x_pos = 0;
      }

    //Atualização da covariância do erro de estimativa
    P_0 = (1-L*C)*P_prev*(1-L*C)+L*R*L;
       
    //-------------- Contagem de Coulomb Modificada -------------------------------
    t_end2 = millis();
    if(cov_min>P_0) //Compara atual cobariância de erro com valor atua da covariância mínima
    {
        //cc igual a zero indica que ainda não houve período crescente em P_0 na descarga
        if(cc==0)
        { 
            //Contagem modificada recebe ultimo valor do SOC atualizado
            xcc_o = x_pos - ((abs(t_end2 - t_end1))/3.6e06)*(corrente/38)*100;
            xc0 = xcc_o;
            //Em caso de descarga covariância mínima é atualizada
            if(corrente > 0)
            {
              cov_min = P_0;
            }
        }else{
        //Caso cc indicar que houve período crescente em P_0, contagem modificada é atualizada por contagem de Coulomb
        xcc_o = xc0 - ((abs(t_end2 - t_0)+abs(t_0 - t_end3))/3.6e06)*(corrente/38)*100;
        xc0 = xcc_o;
        }
    }

    //Altera variável cc em caso de covariância mínima menor que P_0 atualizado
    if(cov_min<P_0)
    {
        xcc_o = xc0 - ((abs(t_end2 - t_0)+abs(t_0 - t_end3))/3.6e06)*(corrente/38)*100;
        xc0 = xcc_o;
        cc = 1;
    }
    t_end3 = millis();
    
    //Cálcula intervalo de confiança e adiciona à estimativa EKF 
    x_posmx = x_pos + 20*sqrt(P_0);
    x_posmi = x_pos - 20*sqrt(P_0);  
    
    //Atualiza SOC inicial para próximo passo 
    x_0 = x_pos;
    
}

//Função para estimativa SOC via contagem de Coulomb na Carga
void Coulomb_carga(){
     t_end = millis();
     //Atualiza variáveis do SOC: EKF, contagem de Coulomb, contagem modificada
     x_pos = x_0 - 1*((abs(t_end - t_0)+abs(t_0 - t_end1))/3.6e06)*(corrente/2.574)*100;
     x_cc = x_cc - 1*((abs(t_end - t_0)+abs(t_0 - t_end1))/3.6e06)*(corrente/2.574)*100;
     xcc_o = xcc_o - 1*((abs(t_end - t_0)+abs(t_0 - t_end1))/3.6e06)*(corrente/2.574)*100;

     //Atualiza variáveis usadas na contagem modificada
     cc = 0;
     cov_min = 10000;

     if(st==1)  //Redefine parâmetros após descarga/repouso
     {
       DODc = 0; //Capacidade de carga reiniciada após processo de descarga (st==1)
       st=0;  //Carrega informação do processo atual de descarga
     }

     //Cálculo de mAh carregado
     DODc = DODc + ((abs(t_end - t_0)+abs(t_0 - t_end1))/3.6e06)*corrente*1000;
     t_end1 = millis();

     //Verifica se a condição de bateria carregada foi alcançada
     if((v_0>=4.15)&&(v_0<=4.25)&&(abs(corrente)<=0.035))
     {
       //Atualiza variáveis
       x_pos = 100;
       x_cc = 100;
     }

     if(x_pos>100)
     {
       x_pos = 100;
     }
     
     x_0 = x_pos;
}



