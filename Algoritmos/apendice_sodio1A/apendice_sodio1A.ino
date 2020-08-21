#include <Wire.h>  //Biblioteca para comunicação I2C
#include <DS3232RTC.h>  //Biblioteca RTC DS3231
#include <Streaming.h>  //Biblioteca para impressão de variáveis
#include <Time.h> //Biblioteca relacionada à manipulação de tempo
#include <SD.h> //Biblioteca para escrita e leitura cartão SD
#include <SPI.h>  //Biblioteca para comunicação SPI
#include <Adafruit_ADS1015.h> //Biblioteca para uso do ADC ADS1115

Adafruit_ADS1115 ads(0x48); //Endereçamento I2C do ADC para leitura dos canais de tensão
Adafruit_ADS1115 ads1(0x49); //Endereçamento I2C do ADC para leitura do sinal do ACS712

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
float acs = 0;

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
float C = 0;  //Parâmetro relacionado a mtriz jacobiana EKF (Eq. 4.14)
float C1 = 0; //Variável auxiliar para cálculo de C
float C2 = 0; //Variável auxiliar para cálculo de C
float Q = 0.001;  //Covariância associada ao ruído de estado
float R = 0.001;  //Covariância associada ao ruído do sensor
float L = 0;  //Ganho de Kalman
float P_prev = 0; //Covariância do erro de estimação prévia
float P_0 = 10000; //Covariância do erro de estimação inicial e atualizado
float cov_min = 10000;  //Parâmetro de covariância mínima associado à contagem modificada
float y_prev = 0; //Tensão terminal estimada
float x_pos = 0;  //SOC estimado atualizado
float x_posmx = 0;  //Limite superior de estimação SOC (intervalo de confiança)
float x_posmi = 0;  //Limite inferior de estimação SOC (intervalo de confiança)
float x_cc = 100; //Definição do SOC inicial para a estimação por contagem de Coulomb
float xcc_o = 0;  //SOC estimado pelo método de contagem de Coulomb Modificada
float xc0 = 0;  //Definição inicial e variável auxiliar na estimação do SOC por contagem modificada
float cc = 0; //Variável auxiliar na contagem modificada: gatilho entre EKF e contagem modificada
int s = 0;  //Define sinal da corrente (carga ou descarga)
int st = 0; //Define atualização na contagem de capacidade de carga(DODc) e descarga (DOD)
int estado = 1; //Define condição de uso do EKF na proposta A
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

  ads1.setGain(GAIN_TWOTHIRDS); //Ganho ADC do sensor de corrente ACS712
  
  ads.begin();  //Início da operação do ADC de aquisição de tensão
  ads1.begin(); //Início da operação do ADC do sensor ACS712

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
    
    acs = acs + ads1.readADC_SingleEnded(3); //Leitura sinal sensor de corrente ACS712
    
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

  //Estabelece o valor de corrente em A
  acs = acs/cont;
  acs = (acs*0.1875)/1000;
  corrente = ((acs - 2.51)*1000)/66;

  //Define se a corrente é no sentido carga ou descarga
  if(corrente < 0)                  
  {
    s=1; //descarga
  }else{
    s=-1; //carga
  }

  //Calibração do sinal de corrente
  corrente = s*(0.96444433*abs(corrente) + 0.00092722); 
  
  if(abs(corrente)<0.2) //Descarta possíveis ruídos de medição
  {
    corrente = 0;
  }
  
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
  file.print(" DODd (Ah): ");
  file.print(DOD);
  file.print(" DODc (Ah): ");
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
  
  acs = 0;
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
  
    if(x_0 = 100) //Se SOC inicial for 100%, EKF esta habilitado
    {
      estado = 1;
    }

    if((corrente==0) && (x_0<100)) //Se SOC inicial for 100%, EKF esta habilitado
    {
      estado = 0;
    }
    
    t_end = millis();
    //Estimação previa SOC: primeira equação do modelo (contagem de Coulomb)
    x_prev = x_0 - ((abs(t_end - t_0)+abs(t_0 - t_end1))/3.6e06)*(corrente/38)*100;
    //Estimação SOC por contagem de Coulomb
    x_cc = x_cc - ((abs(t_end - t_0)+abs(t_0 - t_end1))/3.6e06)*(corrente/38)*100;

    if((st==0) && (corrente>0)) //Redefine parâmetros após carga
    {
      DOD = 0; //Capacidade de descarga reiniciada após processo de carga (st==0)
      P_0 = 10000; //Todo o inicio de descarga a covariancia do erro é reiniciada no maximo
      st=1; //Carrega informação do processo atual de descarga
    }
    //Calcula Ah descarregado
    DOD = DOD + ((abs(t_end - t_0)+abs(t_0 - t_end1))/3.6e06)*corrente;
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
    if(x_prev<=4)
    {
        if(corrente>5.2){
          R_in = 0.04853 + (-0.00505)*((x_prev-2.709)/0.7584) + 0.002539*((corrente-6.752)/1.416) + (-0.001252)*pow(((x_prev-2.709)/0.7584),2) + (-0.003083)*((x_prev-2.709)/0.7584)*((corrente-6.752)/1.416) + 0.000267*pow(((corrente-6.752)/1.416),2) + 0.001197*pow(((x_prev-2.709)/0.7584),3) + 0.001032*pow(((x_prev-2.709)/0.7584),2)*((corrente-6.752)/1.416) + 0.0004252*((x_prev-2.709)/0.7584)*pow(((corrente-6.752)/1.416),2) + (-0.001797)*pow(((corrente-6.752)/1.416),3) + 0.0006883*pow(((x_prev-2.709)/0.7584),4) + 0.001825*pow(((x_prev-2.709)/0.7584),3)*((corrente-6.752)/1.416) + 0.001299*pow(((x_prev-2.709)/0.7584),2)*pow(((corrente-6.752)/1.416),2) + 0.0008473*((x_prev-2.709)/0.7584)*pow(((corrente-6.752)/1.416),3) + (-0.0003871)*pow(((x_prev-2.709)/0.7584),5) + (-0.0008726)*pow(((x_prev-2.709)/0.7584),4)*((corrente-6.752)/1.416) + (-0.0008458)*pow(((x_prev-2.709)/0.7584),3)*pow(((corrente-6.752)/1.416),2) + (-0.0004691)*pow(((x_prev-2.709)/0.7584),2)*pow(((corrente-6.752)/1.416),3);     
        }
        if(corrente<4.8){
          R_in = 0.03703 + (-0.001523)*((x_prev-2.015)/1.163) + 0.0003423*((corrente-4.183)/0.384) + 0.0008378*pow(((x_prev-2.015)/1.163),2) + (-8.558e-05)*((x_prev-2.015)/1.163)*((corrente-4.183)/0.384) + 0.001394*pow(((corrente-4.183)/0.384),2) + (-0.0007461)*pow(((x_prev-2.015)/1.163),3) + (-2.369e-05)*pow(((x_prev-2.015)/1.163),2)*((corrente-4.183)/0.384) + (-0.0005578)*((x_prev-2.015)/1.163)*pow(((corrente-4.183)/0.384),2) + (-0.0002139)*pow(((x_prev-2.015)/1.163),4) + (9.475e-08)*pow(((x_prev-2.015)/1.163),3)*((corrente-4.183)/0.384) + 0.0001958*pow(((x_prev-2.015)/1.163),2)*pow(((corrente-4.183)/0.384),2) + 0.0002165*pow(((x_prev-2.015)/1.163),5) + (3.401e-06)*pow(((x_prev-2.015)/1.163),4)*((corrente-4.183)/0.384) + (-1.072e-05)*pow(((x_prev-2.015)/1.163),3)*pow(((corrente-4.183)/0.384),2);
        }
        if(corrente>=4.8 && corrente<=5.2){
          R_in = 0.04316 + (-0.00304)*((x_prev-2.274)/1.082) + 0.004703*((corrente-4.957)/0.3148) + 0.00569*pow(((x_prev-2.274)/1.082),2) + (-0.000955)*((x_prev-2.274)/1.082)*((corrente-4.957)/0.3148) + (-0.00886)*pow(((x_prev-2.274)/1.082),3) + 0.006438*pow(((x_prev-2.274)/1.082),2)*((corrente-4.957)/0.3148) + 0.003353*pow(((x_prev-2.274)/1.082),4) + (-0.01107)*pow(((x_prev-2.274)/1.082),3)*((corrente-4.957)/0.3148) + (7.194e-05)*pow(((x_prev-2.274)/1.082),5) + 0.004513*pow(((x_prev-2.274)/1.082),4)*((corrente-4.957)/0.3148);
        }
    }
 
    if(x_prev>4 && x_prev<=9)
    {
        if(corrente>5.2){
          R_in = 0.03765 + (-0.001484)*((x_prev-6.495)/1.441) + 0.0002972*((corrente-6.85)/1.465) + 0.0003909*pow(((x_prev-6.495)/1.441),2) + (-0.0002367)*((x_prev-6.495)/1.441)*((corrente-6.85)/1.465) + 0.0002397*pow(((corrente-6.85)/1.465),2) + (9.751e-05)*pow(((x_prev-6.495)/1.441),3) + 0.0001669*pow(((x_prev-6.495)/1.441),2)*((corrente-6.85)/1.465) + (-1.468e-05)*((x_prev-6.495)/1.441)*pow(((corrente-6.85)/1.465),2) + (-0.0005759)*pow(((corrente-6.85)/1.465),3) + 0.0001646*pow(((x_prev-6.495)/1.441),4) + (-5.204e-05)*pow(((x_prev-6.495)/1.441),3)*((corrente-6.85)/1.465) + (8.695e-05)*pow(((x_prev-6.495)/1.441),2)*pow(((corrente-6.85)/1.465),2) + 0.0001396*((x_prev-6.495)/1.441)*pow(((corrente-6.85)/1.465),3) + (7.202e-06)*pow(((x_prev-6.495)/1.441),5) + (-3.427e-05)*pow(((x_prev-6.495)/1.441),4)*((corrente-6.85)/1.465) + (2.325e-05)*pow(((x_prev-6.495)/1.441),3)*pow(((corrente-6.85)/1.465),2) + (-0.0001276)*pow(((x_prev-6.495)/1.441),2)*pow(((corrente-6.85)/1.465),3);
        }
        if(corrente<4.8){
          R_in = 0.03356 + 0.0001603*((x_prev-6.511)/1.439) + (-0.0001194)*((corrente-4.184)/0.3837) + 0.0002072*pow(((x_prev-6.511)/1.439),2) + (-0.0001582)*((x_prev-6.511)/1.439)*((corrente-4.184)/0.3837) + 0.0006368*pow(((corrente-4.184)/0.3837),2) + 0.0002363*pow(((x_prev-6.511)/1.439),3) + (2.297e-05)*pow(((x_prev-6.511)/1.439),2)*((corrente-4.184)/0.3837) + (-0.0001236)*((x_prev-6.511)/1.439)*pow(((corrente-4.184)/0.3837),2) + 0.0002429*pow(((x_prev-6.511)/1.439),4) + (-4.096e-05)*pow(((x_prev-6.511)/1.439),3)*((corrente-4.184)/0.3837) + (2.418e-05)*pow(((x_prev-6.511)/1.439),2)*pow(((corrente-4.184)/0.3837),2) + (2.786e-05)*pow(((x_prev-6.511)/1.439),5) + (-2.739e-05)*pow(((x_prev-6.511)/1.439),4)*((corrente-4.184)/0.3837) + (3.668e-06)*pow(((x_prev-6.511)/1.439),3)*pow(((corrente-4.184)/0.3837),2);
        }
        if(corrente>=4.8 && corrente<=5.2){
          R_in = 0.03629 + (-0.000808)*((x_prev-6.504)/1.435) + 0.001786*((corrente-5.02)/0.3248) + 0.0003394*pow(((x_prev-6.504)/1.435),2) + (-0.0005195)*((x_prev-6.504)/1.435)*((corrente-5.02)/0.3248) + 0.0001824*pow(((x_prev-6.504)/1.435),3) + (7.764e-05)*pow(((x_prev-6.504)/1.435),2)*((corrente-5.02)/0.3248) + 0.0002064*pow(((x_prev-6.504)/1.435),4) + (-3.958e-05)*pow(((x_prev-6.504)/1.435),3)*((corrente-5.02)/0.3248) + (1.595e-05)*pow(((x_prev-6.504)/1.435),5) + (-1.549e-06)*pow(((x_prev-6.504)/1.435),4)*((corrente-5.02)/0.3248);
        }
    }
    
    if(x_prev>9 && x_prev<12.85)
    {
        R_in = 0.04868 + 0.003541*((x_prev-10.93)/1.117) + (-0.003892)*((corrente-5.382)/1.675) + (-0.004053)*pow(((x_prev-10.93)/1.117),2) + (-0.0009337)*((x_prev-10.93)/1.117)*((corrente-5.382)/1.675) + 0.001253*pow(((corrente-5.382)/1.675),2) + (-0.0001759)*pow(((x_prev-10.93)/1.117),3) + 0.0009216*pow(((x_prev-10.93)/1.117),2)*((corrente-5.382)/1.675) + 0.0002525*((x_prev-10.93)/1.117)*pow(((corrente-5.382)/1.675),2) + (-0.000421)*pow(((corrente-5.382)/1.675),3) + 0.002184*pow(((x_prev-10.93)/1.117),4) + (-0.0008189)*pow(((x_prev-10.93)/1.117),3)*((corrente-5.382)/1.675) + (3.38e-05)*pow(((x_prev-10.93)/1.117),2)*pow(((corrente-5.382)/1.675),2) + (-2.969e-05)*((x_prev-10.93)/1.117)*pow(((corrente-5.382)/1.675),3) + 0.0007862*pow(((x_prev-10.93)/1.117),5) + (-0.0005258)*pow(((x_prev-10.93)/1.117),4)*((corrente-5.382)/1.675) + 0.0001627*pow(((x_prev-10.93)/1.117),3)*pow(((corrente-5.382)/1.675),2) + (1.307e-05)*pow(((x_prev-10.93)/1.117),2)*pow(((corrente-5.382)/1.675),3);
    }

    if(x_prev>=12.85 && x_prev<85)
    {
        R_in = 0.0395 + (-0.007157)*((x_prev-56.5)/25.19) + 0.0004231*((corrente-5.899)/1.879) + 0.0006393*pow(((x_prev-56.5)/25.19),2) + (-0.0009451)*((x_prev-56.5)/25.19)*((corrente-5.899)/1.879) + (-0.0002236)*pow(((corrente-5.899)/1.879),2) + (-0.001634)*pow(((x_prev-56.5)/25.19),3) + (-0.001597)*pow(((x_prev-56.5)/25.19),2)*((corrente-5.899)/1.879) + 0.0004455*((x_prev-56.5)/25.19)*pow(((corrente-5.899)/1.879),2) + (-8.647e-05)*pow(((corrente-5.899)/1.879),3) + 0.0009919*pow(((x_prev-56.5)/25.19),4) + 0.001507*pow(((x_prev-56.5)/25.19),3)*((corrente-5.899)/1.879) + (3.905e-06)*pow(((x_prev-56.5)/25.19),2)*pow(((corrente-5.899)/1.879),2) + 0.0002688*((x_prev-56.5)/25.19)*pow(((corrente-5.899)/1.879),3) + (-2.37e-05)*pow(((x_prev-56.5)/25.19),5) + (-0.0003696)*pow(((x_prev-56.5)/25.19),4)*((corrente-5.899)/1.879) + (-0.0004639)*pow(((x_prev-56.5)/25.19),3)*pow(((corrente-5.899)/1.879),2) + 0.0003097*pow(((x_prev-56.5)/25.19),2)*pow(((corrente-5.899)/1.879),3);    
    }

    if(x_prev>=85 && x_prev<=100)
    {
        R_in = 0.03068 + (-0.0008392)*((x_prev-91.99)/4.044) + (-0.0002334)*((corrente-5.378)/1.672) + (-0.0002535)*pow(((x_prev-91.99)/4.044),2) + 0.0001377*((x_prev-91.99)/4.044)*((corrente-5.378)/1.672) + (-0.001664)*pow(((corrente-5.378)/1.672),2) + (-0.0001703)*pow(((x_prev-91.99)/4.044),3) + 0.0001708*pow(((x_prev-91.99)/4.044),2)*((corrente-5.378)/1.672) + (-0.000181)*((x_prev-91.99)/4.044)*pow(((corrente-5.378)/1.672),2) + 0.0001103*pow(((corrente-5.378)/1.672),3) + (2.963e-05)*pow(((x_prev-91.99)/4.044),4) + (9.641e-07)*pow(((x_prev-91.99)/4.044),3)*((corrente-5.378)/1.672) + 0.0001391*pow(((x_prev-91.99)/4.044),2)*pow(((corrente-5.378)/1.672),2) + (3.553e-05)*((x_prev-91.99)/4.044)*pow(((corrente-5.378)/1.672),3) + 0.0002528*pow(((corrente-5.378)/1.672),4) + (4.3e-05)*pow(((x_prev-91.99)/4.044),5) + (-5.053e-06)*pow(((x_prev-91.99)/4.044),4)*((corrente-5.378)/1.672) + (-1.403e-05)*pow(((x_prev-91.99)/4.044),3)*pow(((corrente-5.378)/1.672),2) + (-8.667e-05)*pow(((x_prev-91.99)/4.044),2)*pow(((corrente-5.378)/1.672),3) + (1.061e-05)*((x_prev-91.99)/4.044)*pow(((corrente-5.378)/1.672),4);
    }
    
    //-------------------------------- Cálculo da OCV -------------------------------------------------- 
    //OCV como função do SOC prévio
    if(x_prev<=12.85)
    {
        ocv = 0.006391*pow(((x_prev-6.415)/3.712),9) + 0.009434*pow(((x_prev-6.415)/3.712),8) + (-0.03024)*pow(((x_prev-6.415)/3.712),7) + (-0.04723)*pow(((x_prev-6.415)/3.712),6) + 0.03629*pow(((x_prev-6.415)/3.712),5) + 0.06931*pow(((x_prev-6.415)/3.712),4) + 0.008763*pow(((x_prev-6.415)/3.712),3) + (-0.002912)*pow(((x_prev-6.415)/3.712),2) + 0.02003*((x_prev-6.415)/3.712) + 2.35;   
    }
   
    if(x_prev>12.85)
    {
        ocv = 0.0001792*pow(((x_prev-56.24)/24.77),8) + (-0.0003101)*pow(((x_prev-56.24)/24.77),7) + (-0.001027)*pow(((x_prev-56.24)/24.77),6) + 0.00225*pow(((x_prev-56.24)/24.77),5) + 0.001846*pow(((x_prev-56.24)/24.77),4) + (-0.005562)*pow(((x_prev-56.24)/24.77),3) + (-0.0002777)*pow(((x_prev-56.24)/24.77),2) + 0.006952*((x_prev-56.24)/24.77) + 2.581;
    }

    //-------------------------------- Calculo do parâmetro C -----------------------------------------------------   
    //Calculado em duas partes:
    //C = d(yk)/d(xk) com xk=xkprev, C = C2-C1, C1 = d(Rin(xk)*i)/dxk
    //C2 = d(ocv(xk))/dxk
    
    if(x_prev<=4)
    {
        if(corrente>5.2){
          C1 = (1/0.7584)*(-0.00505) + 2*(1/0.7584)*(-0.001252)*((x_prev-2.709)/0.7584) + (1/0.7584)*(-0.003083)*((corrente-6.752)/1.416) + 3*(1/0.7584)*0.001197*pow(((x_prev-2.709)/0.7584),2) + 2*(1/0.7584)*0.001032*((x_prev-2.709)/0.7584)*((corrente-6.752)/1.416) + (1/0.7584)*0.0004252*pow(((corrente-6.752)/1.416),2) + 4*(1/0.7584)*0.0006883*pow(((x_prev-2.709)/0.7584),3) + 3*(1/0.7584)*0.001825*pow(((x_prev-2.709)/0.7584),2)*((corrente-6.752)/1.416) + 2*(1/0.7584)*0.001299*((x_prev-2.709)/0.7584)*pow(((corrente-6.752)/1.416),2) + (1/0.7584)*0.0008473*pow(((corrente-6.752)/1.416),3) + 5*(1/0.7584)*(-0.0003871)*pow(((x_prev-2.709)/0.7584),4) + 4*(1/0.7584)*(-0.0008726)*pow(((x_prev-2.709)/0.7584),3)*((corrente-6.752)/1.416) + 3*(1/0.7584)*(-0.0008458)*pow(((x_prev-2.709)/0.7584),2)*pow(((corrente-6.752)/1.416),2) + 2*(1/0.7584)*(-0.0004691)*((x_prev-2.709)/0.7584)*pow(((corrente-6.752)/1.416),3);
        }
        if(corrente<4.8){
          C1 = (1/1.163)*(-0.001523) + 2*(1/1.163)*0.0008378*((x_prev-2.015)/1.163) + (1/1.163)*(-8.558e-05)*((corrente-4.183)/0.384) + 3*(1/1.163)*(-0.0007461)*pow(((x_prev-2.015)/1.163),2) + 2*(1/1.163)*(-2.369e-05)*((x_prev-2.015)/1.163)*((corrente-4.183)/0.384) + (1/1.163)*(-0.0005578)*pow(((corrente-4.183)/0.384),2) + 4*(1/1.163)*(-0.0002139)*pow(((x_prev-2.015)/1.163),3) + 3*(1/1.163)*(9.475e-08)*pow(((x_prev-2.015)/1.163),2)*((corrente-4.183)/0.384) + 2*(1/1.163)*0.0001958*((x_prev-2.015)/1.163)*pow(((corrente-4.183)/0.384),2) + 5*(1/1.163)*0.0002165*pow(((x_prev-2.015)/1.163),4) + 4*(1/1.163)*(3.401e-06)*pow(((x_prev-2.015)/1.163),3)*((corrente-4.183)/0.384) + 3*(1/1.163)*(-1.072e-05)*pow(((x_prev-2.015)/1.163),2)*pow(((corrente-4.183)/0.384),2);
        }
        if(corrente>=4.8 && corrente<=5.2){
          C1 = (1/1.082)*(-0.00304) + 2*(1/1.082)*0.00569*((x_prev-2.274)/1.082) + (1/1.082)*(-0.000955)*((corrente-4.957)/0.3148) + 3*(1/1.082)*(-0.00886)*pow(((x_prev-2.274)/1.082),2) + 2*(1/1.082)*0.006438*((x_prev-2.274)/1.082)*((corrente-4.957)/0.3148) + 4*(1/1.082)*0.003353*pow(((x_prev-2.274)/1.082),3) + 3*(1/1.082)*(-0.01107)*pow(((x_prev-2.274)/1.082),2)*((corrente-4.957)/0.3148) + 5*(1/1.082)*(7.194e-05)*pow(((x_prev-2.274)/1.082),4) + 4*(1/1.082)*0.004513*pow(((x_prev-2.274)/1.082),3)*((corrente-4.957)/0.3148);
        }
    }
 
    if(x_prev>4 && x_prev<=9)
    {
        if(corrente>5.2){
          C1 = (1/1.441)*(-0.001484) + 2*(1/1.441)*0.0003909*((x_prev-6.495)/1.441) + (1/1.441)*(-0.0002367)*((corrente-6.85)/1.465) + 3*(1/1.441)*(9.751e-05)*pow(((x_prev-6.495)/1.441),2) + 2*(1/1.441)*0.0001669*((x_prev-6.495)/1.441)*((corrente-6.85)/1.465) + (1/1.441)*(-1.468e-05)*pow(((corrente-6.85)/1.465),2) + 4*(1/1.441)*0.0001646*pow(((x_prev-6.495)/1.441),3) + 3*(1/1.441)*(-5.204e-05)*pow(((x_prev-6.495)/1.441),2)*((corrente-6.85)/1.465) + 2*(1/1.441)*(8.695e-05)*((x_prev-6.495)/1.441)*pow(((corrente-6.85)/1.465),2) + (1/1.441)*0.0001396*pow(((corrente-6.85)/1.465),3) + 5*(1/1.441)*(7.202e-06)*pow(((x_prev-6.495)/1.441),4) + 4*(1/1.441)*(-3.427e-05)*pow(((x_prev-6.495)/1.441),3)*((corrente-6.85)/1.465) + 3*(1/1.441)*(2.325e-05)*pow(((x_prev-6.495)/1.441),2)*pow(((corrente-6.85)/1.465),2) + 2*(1/1.441)*(-0.0001276)*((x_prev-6.495)/1.441)*pow(((corrente-6.85)/1.465),3);
        }
        if(corrente<4.8){
          C1 = (1/1.439)*0.0001603 + 2*(1/1.439)*0.0002072*((x_prev-6.511)/1.439) + (1/1.439)*(-0.0001582)*((corrente-4.184)/0.3837) + 3*(1/1.439)*0.0002363*pow(((x_prev-6.511)/1.439),2) + 2*(1/1.439)*(2.297e-05)*((x_prev-6.511)/1.439)*((corrente-4.184)/0.3837) + (1/1.439)*(-0.0001236)*pow(((corrente-4.184)/0.3837),2) + 4*(1/1.439)*(0.0002429)*pow(((x_prev-6.511)/1.439),3) + 3*(1/1.439)*(-4.096e-05)*pow(((x_prev-6.511)/1.439),2)*((corrente-4.184)/0.3837) + 2*(1/1.439)*(2.418e-05)*((x_prev-6.511)/1.439)*pow(((corrente-4.184)/0.3837),2) + 5*(1/1.439)*(2.786e-05)*pow(((x_prev-6.511)/1.439),4) + 4*(1/1.439)*(-2.739e-05)*pow(((x_prev-6.511)/1.439),3)*((corrente-4.184)/0.3837) + 3*(1/1.439)*(3.668e-06)*pow(((x_prev-6.511)/1.439),2)*pow(((corrente-4.184)/0.3837),2);
        }
        if(corrente>=4.8 && corrente<=5.2){
          C1 = (1/1.435)*(-0.000808) + 2*(1/1.435)*0.0003394*((x_prev-6.504)/1.435) + (1/1.435)*(-0.0005195)*((corrente-5.02)/0.3248) + 3*(1/1.435)*0.0001824*pow(((x_prev-6.504)/1.435),2) + 2*(1/1.435)*(7.764e-05)*((x_prev-6.504)/1.435)*((corrente-5.02)/0.3248) + 4*(1/1.435)*0.0002064*pow(((x_prev-6.504)/1.435),3) + 3*(1/1.435)*(-3.958e-05)*pow(((x_prev-6.504)/1.435),2)*((corrente-5.02)/0.3248) + 5*(1/1.435)*(1.595e-05)*pow(((x_prev-6.504)/1.435),4) + 4*(1/1.435)*(-1.549e-06)*pow(((x_prev-6.504)/1.435),3)*((corrente-5.02)/0.3248);
        }
    }
    
    if(x_prev>9 && x_prev<12.85)
    {
        C1 = (1/1.117)*0.003541 + 2*(1/1.117)*(-0.004053)*((x_prev-10.93)/1.117) + (1/1.117)*(-0.0009337)*((corrente-5.382)/1.675) + 3*(1/1.117)*(-0.0001759)*pow(((x_prev-10.93)/1.117),2) + 2*(1/1.117)*0.0009216*((x_prev-10.93)/1.117)*((corrente-5.382)/1.675) + (1/1.117)*0.0002525*pow(((corrente-5.382)/1.675),2) + 4*(1/1.117)*0.002184*pow(((x_prev-10.93)/1.117),3) + 3*(1/1.117)*(-0.0008189)*pow(((x_prev-10.93)/1.117),2)*((corrente-5.382)/1.675) + 2*(1/1.117)*(3.38e-05)*((x_prev-10.93)/1.117)*pow(((corrente-5.382)/1.675),2) + (1/1.117)*(-2.969e-05)*pow(((corrente-5.382)/1.675),3) + 5*(1/1.117)*0.0007862*pow(((x_prev-10.93)/1.117),4) + 4*(1/1.117)*(-0.0005258)*pow(((x_prev-10.93)/1.117),3)*((corrente-5.382)/1.675) + 3*(1/1.117)*0.0001627*pow(((x_prev-10.93)/1.117),2)*pow(((corrente-5.382)/1.675),2) + 2*(1/1.117)*(1.307e-05)*((x_prev-10.93)/1.117)*pow(((corrente-5.382)/1.675),3);       
    }

    if(x_prev>=12.85 && x_prev<85)
    {
        C1 = (1/25.19)*(-0.007157) + 2*(1/25.19)*0.0006393*((x_prev-56.5)/25.19) + (1/25.19)*(-0.0009451)*((corrente-5.899)/1.879) + 3*(1/25.19)*(-0.001634)*pow(((x_prev-56.5)/25.19),2) + 2*(1/25.19)*(-0.001597)*((x_prev-56.5)/25.19)*((corrente-5.899)/1.879) + (1/25.19)*0.0004455*pow(((corrente-5.899)/1.879),2) + 4*(1/25.19)*0.0009919*pow(((x_prev-56.5)/25.19),3) + 3*(1/25.19)*0.001507*pow(((x_prev-56.5)/25.19),2)*((corrente-5.899)/1.879) + 2*(1/25.19)*(3.905e-06)*((x_prev-56.5)/25.19)*pow(((corrente-5.899)/1.879),2) + (1/25.19)*0.0002688*pow(((corrente-5.899)/1.879),3) + 5*(1/25.19)*(-2.37e-05)*pow(((x_prev-56.5)/25.19),4) + 4*(1/25.19)*(-0.0003696)*pow(((x_prev-56.5)/25.19),3)*((corrente-5.899)/1.879) + 3*(1/25.19)*(-0.0004639)*pow(((x_prev-56.5)/25.19),2)*pow(((corrente-5.899)/1.879),2) + 2*(1/25.19)*0.0003097*((x_prev-56.5)/25.19)*pow(((corrente-5.899)/1.879),3);       
    }

    if(x_prev>85 && x_prev<=100)
    {
        C1 = (1/4.044)*(-0.0008392) + 2*(1/4.044)*(-0.0002535)*((x_prev-91.99)/4.044) + (1/4.044)*0.0001377*((corrente-5.378)/1.672) + 3*(1/4.044)*(-0.0001703)*pow(((x_prev-91.99)/4.044),2) + 2*(1/4.044)*0.0001708*((x_prev-91.99)/4.044)*((corrente-5.378)/1.672) + (1/4.044)*(-0.000181)*pow(((corrente-5.378)/1.672),2) + 4*(1/4.044)*(2.963e-05)*pow(((x_prev-91.99)/4.044),3) + 3*(1/4.044)*(9.641e-07)*pow(((x_prev-91.99)/4.044),2)*((corrente-5.378)/1.672) + 2*(1/4.044)*0.0001391*((x_prev-91.99)/4.044)*pow(((corrente-5.378)/1.672),2) + (1/4.044)*(3.553e-05)*pow(((corrente-5.378)/1.672),3) + 5*(1/4.044)*(4.3e-05)*pow(((x_prev-91.99)/4.044),4) + 4*(1/4.044)*(-5.053e-06)*pow(((x_prev-91.99)/4.044),3)*((corrente-5.378)/1.672) + 3*(1/4.044)*(-1.403e-05)*pow(((x_prev-91.99)/4.044),2)*pow(((corrente-5.378)/1.672),2) + 2*(1/4.044)*(-8.667e-05)*((x_prev-91.99)/4.044)*pow(((corrente-5.378)/1.672),3) + (1/4.044)*(1.061e-05)*pow(((corrente-5.378)/1.672),4);
    }

    if(x_prev<=12.85)
    {
        C2 = 9*(1/3.712)*0.006391*pow(((x_prev-6.415)/3.712),8) + 8*(1/3.712)*0.009434*pow(((x_prev-6.415)/3.712),7) + 7*(1/3.712)*(-0.03024)*pow(((x_prev-6.415)/3.712),6) + 6*(1/3.712)*(-0.04723)*pow(((x_prev-6.415)/3.712),5) + 5*(1/3.712)*0.03629*pow(((x_prev-6.415)/3.712),4) + 4*(1/3.712)*0.06931*pow(((x_prev-6.415)/3.712),3) + 3*(1/3.712)*0.008763*pow(((x_prev-6.415)/3.712),2) + 2*(1/3.712)*(-0.002912)*((x_prev-6.415)/3.712) + (1/3.712)*0.02003;
    }
   
    if(x_prev>12.85)
    {
        C2 = 8*(1/24.77)*0.0001792*pow(((x_prev-56.24)/24.77),7) + 7*(1/24.77)*(-0.0003101)*pow(((x_prev-56.24)/24.77),6) + 6*(1/24.77)*(-0.001027)*pow(((x_prev-56.24)/24.77),5) + 5*(1/24.77)*0.00225*pow(((x_prev-56.24)/24.77),4) + 4*(1/24.77)*0.001846*pow(((x_prev-56.24)/24.77),3) + 3*(1/24.77)*(-0.005562)*pow(((x_prev-56.24)/24.77),2) + 2*(1/24.77)*(-0.0002777)*((x_prev-56.24)/24.77) + (1/24.77)*0.006952;
    }  
    
    C = C2-C1*corrente;
    //-------------------------------------------------------------------------------------------------------------------
    y_prev = ocv - R_in*corrente; //Tensão terminal estimada a partir do SOC prévio
    
    //Alterações da Proposta A quando às covariãncias Q e R em função do desvio de tensão do modelo
    if((corrente>0)&&(abs(v_0-y_prev)>0.002)&&(P_0<1))
    {
      Q = 0.0001;
      R = 0.01;

      if(abs(v_0-y_prev)>0.01)
      {
        R = 0.1;
      } 
    }else{
      Q = 0.001;
      R = 0.001;
    }
            
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

    //Alteração proposta A
    if((corrente>0)&&(abs(v_0-y_prev)>0.25))
    {
      P_0 = 10000;
    }
    
    //Se a variavel estado não indicar que a bateria sofreu uma carga completa antes da atual descarga
    //EKF não atualizado (recebe Contagem de Coulomb)
    if(estado == 0)
    {
      x_pos = x_prev;
    }
    
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
    x_posmx = x_pos + 10*sqrt(P_0);
    x_posmi = x_pos - 10*sqrt(P_0);  
    
    //Atualiza SOC inicial para próximo passo 
    x_0 = x_pos;
    
}

//Função para estimativa SOC via contagem de Coulomb na Carga
void Coulomb_carga(){
     t_end = millis();
     //Atualiza variáveis do SOC: EKF, contagem de Coulomb, contagem modificada
     x_pos = x_0 - 1*((abs(t_end - t_0)+abs(t_0 - t_end1))/3.6e06)*(corrente/38)*100;
     x_cc = x_cc - 1*((abs(t_end - t_0)+abs(t_0 - t_end1))/3.6e06)*(corrente/38)*100;
     xcc_o = xcc_o - 1*((abs(t_end - t_0)+abs(t_0 - t_end1))/3.6e06)*(corrente/38)*100;

     //Atualiza variáveis usadas na contagem modificada
     cc = 0;
     cov_min = 10000;

     if(st==1)  //Redefine parâmetros após descarga/repouso
     {
       DODc = 0; //Capacidade de carga reiniciada após processo de descarga (st==1)
       st=0;  //Carrega informação do processo atual de descarga
       estado = 0; //Indica carga incompleta
     }

     //Cálculo de Ah carregado
     DODc = DODc + ((abs(t_end - t_0)+abs(t_0 - t_end1))/3.6e06)*corrente;
     t_end1 = millis();

     //Verifica se a condição de bateria carregada foi alcançada
     if((v_0>=2.62)&&(v_0<=2.72)&&(abs(corrente)<=0.8))
     {
       //Atualiza variáveis
       x_pos = 100;
       x_cc = 100;
       estado = 1;
     }

     if(x_pos>100)
     {
      x_pos = 100;
     }
     
     x_0 = x_pos;
}



