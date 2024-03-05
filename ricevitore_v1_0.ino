/*
  Ricevitore.ino
  Created by Alessandro Disarò, March 2024.
*/

#include <DueTimer.h> //libreria DueTimer di Ivan Seidel https://github.com/ivanseidel/DueTimer
#include <DueAdcFast.h> //libreria DueAdcFast di Antonio Previtali https://github.com/AntonioPrevitali/DueAdcFast

/* DEFINIZIONE DEI PARAMETRI DI FUNZIONAMENTO */
#define TS 200 //periodo di campionamento 
#define OSR 5 //tasso di sovracampionamento
#define SYNC_LEN 254 * OSR //lunghezza del segnale di sincronizzazione in campioni (simboli * OSR)
#define SAMPLES_BEFORE_MAX 200 //elementi da attendere prima di confermare il massimo
#define alphaAbsCorr 0.01 //coefficiente per la media mobile della cross-correlazione
#define alphaAdc 0.005 //coefficiente per la media mobile dei valori letti dall'Adc 
#define SPB 4 //simboli per byte (attualmente fisso, valori diversi da 4 non implementati)
#define SPM 1024 * SPB //lunghezza in simboli del messaggio (byte * SPB)

/* VARIABILI */

//segnale di sincronizzazione
int mls[SYNC_LEN] = {1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,1,1,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,1,1,1,1,1};
/* Stato della comunicazione:
0 → riempimento iniziale
1 → sincronizzazione
2 → rilevazione soglie
3 → lettura messaggio
*/
byte status = 0;
int lastAdcValue; //ultimo valore letto
int adcValues[SYNC_LEN]; //buffer circolare per memorizzare i valori letti
short posAdcValues = 0; //posizione del prossimo elemento da inserire nel buffer
int maxCrossCorrelation = 0; //massimo attuale della cross-correlazione
int maxTimer = SAMPLES_BEFORE_MAX; //timer per la conferma del massimo
float meanAbsCorr = -1; //media mobile dei valori assoluti della cross-correlazione
float meanAdc = 0; //media mobile dei valori letti
short boundCounter = OSR; //contatore per il rilevamento delle soglie
int lowerBound = 0; //estremo inferiore di trasmissione
int upperBound = 0; //estremo superiore di trasmissione
int decodingBounds[3]; //soglie per la decodifica
int expectedSymbols = SPM; //simboli attesi nel messaggio corrente
int expectedValues = OSR; //valori attesi per il simbolo corrente
int meanSymbolValue = 0; //media dei valori letti per il simbolo corrente
byte decodedByte = 0; //byte decodificato
float meanAdcStart = 0; //valore della media mobile Adc a inizio messaggio
float symbolValueAdjusted = 0; //valore associato al simbolo dopo l'aggiustamento rispetto alla media


//predispongo un buffer di 1024 elementi per la lettura con DueAdcFast
DueAdcFast DueAdcF(1024); 

void setup() {
  //led integrato che si accende per indicare l'avvenuta sincronizzazione
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  //imposto 12 bit per lettura (10 di default)
  analogReadResolution(12);
  //indico il pin da usare per la lettura
  DueAdcF.EnablePin(A0);
  //avvio la lettura alla velocità di default (667 kHz)
  DueAdcF.Start();
  //avvio la comunicazione seriale
  Serial.begin(115200);
  while (!Serial) {}
  //collego l'interrupt al timer 3 e lo avvio
  Timer3.attachInterrupt(readSample); 
  Timer3.start(TS);
}

//delego alla libreria DueAdcFast la gestione quando il buffer è pieno
void ADC_Handler() {
  DueAdcF.adcHandler();
}

//funzione chiamata ogni TS microsecondi
void readSample(){
  //leggo l'ultimo valore disponibile
  lastAdcValue = DueAdcF.FindValueForPin(A0);
  //aggiorno la media mobile dei valori
  meanAdc = (1 - alphaAdc) * meanAdc + (alphaAdc) * lastAdcValue;
  /*********
    RIEMPIMENTO INIZIALE
    Il buffer deve essere riempito prima di poter iniziare la sincronizzazione
  *********/
  if(status == 0){
    //inserisco l'ultimo valore letto nel buffer
    adcValues[posAdcValues++] = lastAdcValue;
    //se il buffer si è riempito passo allo stato 1 (sincronizzazione)
    if (posAdcValues == SYNC_LEN) {
      status = 1; 
    }
    //aggiorno la posizione
    posAdcValues %= SYNC_LEN;
  }
  /*********
    SINCRONIZZAZIONE
    Calcolo della cross-correlazione e ricerca del picco per inizio del messaggio
  *********/
  else if(status == 1){
    //inserisco l'ultimo valore letto nel buffer
    adcValues[posAdcValues++] = lastAdcValue;
    //aggiorno la posizione
    posAdcValues %= SYNC_LEN;
    //calcolo la cross-correlazione
    int crossCorr = cross_correlation(adcValues, mls, SYNC_LEN, posAdcValues);
    //se il valore calcolato è un nuovo massimo lo aggiorno e resetto il contatore
    if (crossCorr > maxCrossCorrelation) {
      maxCrossCorrelation = crossCorr;
      maxTimer = SAMPLES_BEFORE_MAX;
    }
    //altrimenti aggiorno il contatore ed effettuo i controlli sulla media
    else {
      //decremento del contatore
      maxTimer--;
      //se il contatore scende troppo (rischia di andare in overflow e poi tornare a 0) viene rimesso a -1
      if(maxTimer <= -2000000000){
        maxTimer = -1;
      }
      //aggiornamento della media mobile dei valori assoluti di cross-correlazione
      if (crossCorr < 0) {
        crossCorr *= -1;
      }
      if (meanAbsCorr == -1) {
        meanAbsCorr = crossCorr;
      }
      meanAbsCorr = (1 - alphaAbsCorr) * meanAbsCorr + (alphaAbsCorr) * crossCorr;
      //controllo rispetto alla media attuale
      if(maxCrossCorrelation <= 9 * meanAbsCorr){
        maxTimer = -1;
      }
      //se il timer è 1 si considera trovato il punto di sincronizzazione (imposto stato 2 -> rilevamento soglie)
      if(maxTimer == 1){          
        digitalWrite(LED_BUILTIN, HIGH);
        status = 2;
      }
    }
  }
  /*********
    RILEVAMENTO SOGLIE
    Vengono letti 100 valori per la soglia inferiore (led spento) e 100 per quella superiore (led acceso completamente)
  *********/
  else if(status == 2){
    //l'ultimo valore letto va assegnato alla soglia superiore
    upperBound = lastAdcValue;
    boundCounter--;
    //gli ultimi 199 valori dell'array vanno assegnati a gruppi di 5 alle soglie inferiore e superiore
    bool actuallyUpper = true;
    while(maxTimer<SAMPLES_BEFORE_MAX){
      posAdcValues--;
      posAdcValues %= SYNC_LEN;
      if(actuallyUpper){
        upperBound+=adcValues[posAdcValues];
      } else {
        lowerBound+=adcValues[posAdcValues];
      }
      boundCounter--;
      if(boundCounter == 0){
        boundCounter = OSR;
        actuallyUpper = !actuallyUpper;
      }
      maxTimer++;
    }
    //calcolo le medie
    lowerBound /= 100;
    upperBound /= 100;
    //calcolo le soglie equidistanti
    decodingBounds[0] = upperBound - (upperBound - lowerBound) / 6;
    decodingBounds[1] = (upperBound + lowerBound) / 2;
    decodingBounds[2] = lowerBound + (upperBound - lowerBound) / 6;
    //salvo il valore della media a inizio messaggio
    meanAdcStart = meanAdc;
    //passo allo stato 3 -> ricezione messaggio
    status = 3;
  }
  /*********
    RICEZIONE MESSAGGIO
    Lettura di OSR valori per ogni simbolo, decodifica del simbolo, trasmissione ogni 4 simboli (1 byte)
  *********/
  else if(status == 3){
    //sommo il valore letto per poi fare la media e aggiorno il contatore dei valori attesi
    meanSymbolValue += lastAdcValue;
    expectedValues--;
    //se sono stati letti tutti i valori procedo alla decodifica
    if(expectedValues == 0){
      //calcolo della media
      meanSymbolValue/= OSR;
      //scorrimento dei bit per scrivere la nuova coppia
      decodedByte <<= 2;
      //calcolo valore simbolo con aggiustamento rispetto alla media
      symbolValueAdjusted = meanSymbolValue + meanAdcStart - meanAdc;
      //confronto con le soglie di decodifica e inserimento dei bit
      if(symbolValueAdjusted > decodingBounds[0]){
        decodedByte |= 0b00000010;
      } else if (symbolValueAdjusted > decodingBounds[1]){
        decodedByte |= 0b00000011;
      } else if(symbolValueAdjusted > decodingBounds[2]){
        decodedByte |= 0b00000001;
      }
      //aggiorno il contatore dei simboli attesi
      expectedSymbols--;
      //azzero la media del simbolo
      meanSymbolValue = 0;
      //se il byte è pieno lo trasmetto
      if(expectedSymbols % SPB == 0){
        Serial.write(decodedByte);
        decodedByte = 0b00000000;
      }
      //se il messasggio è terminato ripristino le variabili e imposto lo stato a 0
      if(expectedSymbols <= 0){
        status = 0;
        posAdcValues = 0;
        maxCrossCorrelation = 0;
        maxTimer = SAMPLES_BEFORE_MAX;
        meanAbsCorr = -1;
        meanAdc = 0;
        boundCounter = OSR;
        lowerBound = 0;
        upperBound = 0;
        expectedSymbols = SPM;
        decodedByte = 0b00000000;        
        digitalWrite(LED_BUILTIN, LOW);
      }
    }
    //ripristino il numero di valori attesi per il nuovo simbolo
    expectedValues = OSR;
  }
}

void loop() {}

//Funzione per eseguire la cross-correlazione tra due segnali di interi con la stessa dimensione (ottimizzato per dimensione 1270)
int cross_correlation(int* pA, int* pB, int len, int pos){
  int sum = 0;
  int i = (len - pos) / 31;
  int r = (len - pos) % 31;
  int* px = pA + pos;
  int* py = pB; 

  while(i > 0){
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    i--;
  }

  while(r > 0){
    sum += *px++ * *py++;
    r--;
  }

  i = pos / 31;
  r = pos % 31;
  px = pA;

  while(i > 0){
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    sum += *px++ * *py++;
    i--;
  }

  while(r > 0){
    sum += *px++ * *py++;
    r--;
  }

  return sum;
}

