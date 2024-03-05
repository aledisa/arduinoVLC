/*
  Trasmettitore.ino
  Created by Alessandro Disarò, March 2024.
*/

#include <DueTimer.h> //libreria DueTimer di Ivan Seidel https://github.com/ivanseidel/DueTimer

/* DEFINIZIONE DEI PARAMETRI DI FUNZIONAMENTO */
#define TS 200 //periodo di campionamento al ricevitore in microsecondi
#define OSR 5 //tasso di sovracampionamento al ricevitore
#define TT TS * OSR //periodo di trasmissione (ovvero di simbolo)
#define SYNC_LEN 254 //lunghezza del segnale di sincronizzazione in simboli
#define BOUND_LEN 40 //lunghezza del segnale di rilevamento soglie
#define BUFF_SIZE 400 //dimensione del buffer per i valori da trasmettere
#define SPB 4 //simboli per byte (attualmente fisso, valori diversi da 4 non implementati)
#define SPM 1024 * SPB //lunghezza in simboli del messaggio (byte * SPB)

/* VARIABILI */
//segnale di sincronizzazione
byte mls[SYNC_LEN] = {2,0,0,2,2,2,2,0,0,0,2,2,0,2,2,0,0,0,0,2,0,0,0,2,0,2,2,2,0,2,0,2,2,2,2,0,2,2,0,2,2,2,2,2,0,0,0,0,2,2,0,2,0,0,2,2,0,2,0,2,2,0,2,2,0,2,0,2,0,0,0,0,0,2,0,0,2,2,2,0,2,2,0,0,2,0,0,2,0,0,2,2,0,0,0,0,0,0,2,2,2,0,2,0,0,2,0,0,0,2,2,2,0,0,0,2,0,0,0,0,0,0,0,2,0,2,2,0,0,0,2,2,2,2,0,2,0,0,0,0,2,2,2,2,2,2,2,2,0,0,2,0,0,0,0,2,0,2,0,0,2,2,2,2,2,0,2,0,2,0,2,0,2,2,2,0,0,0,0,0,2,2,0,0,0,2,0,2,0,2,2,0,0,2,2,0,0,2,0,2,2,2,2,2,2,0,2,2,2,2,0,0,2,2,0,2,2,2,0,2,2,2,0,0,2,0,2,0,2,0,0,2,0,2,0,0,0,2,0,0,2,0,2,2,0,2,0,0,0,2,2,0,0,2};
/*Stato della trasmissione:
0 → nessun messaggio da inviare
1 → trasmissione della sincronizzazione
2 → trasmissione del rilevamento soglie
3 → trasmissione del messaggio
*/
volatile byte status = 0;
//segnale di rilevamento soglie
byte boundSignal[BOUND_LEN] = {0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2,0,2};
const int TX_LEVELS[] = {0, 2992, 4095, 3585}; //valori per il DAC dei simboli nell'ordine (0,1,3,2)
int txValues[BUFF_SIZE]; //buffer circolare per i valori da trasmettere
volatile short posWTxValues = 0; //posizione del prossimo elemento da inserire nel buffer
short posRTxValues = 0; //posizione del prossimo elemento da leggere dal buffer
short posStartBuff = 0; //posizione del prossimo elemento da leggere nella fase iniziale
volatile short symbols = 0; //simboli validi nel buffer di trasmissione
int transmittedSymbols = 0; //simboli trasmessi nel messaggio corrente

void setup() {
  //imposto il DAC0 con valore iniziale nullo e risoluzione 12 bit
  pinMode(DAC0, OUTPUT);
  analogWriteResolution(12);
  analogWrite(DAC0, 0);
  //collego l'interrupt al timer 3
  Timer3.attachInterrupt(setLed);
  //avvio la comunicazione seriale
  Serial.begin(115200);
  while (!Serial) {}
}

//funzione chiamata ogni TT microsecondi
void setLed(){
  //stato 1 → sincronizzazione
  if(status == 1){
    //regolo il led e incremento la posizione
    analogWrite(DAC0, TX_LEVELS[mls[posStartBuff++]]);
    //se ho finito il segnale di sincronizzazione passo al rilevamento soglie
    if(posStartBuff == SYNC_LEN){
      posStartBuff = 0;
      status = 2;
    }
  }
  //stato 2 → rilevamento soglie
  else if(status == 2){
    //regolo il led e incremento la posizione
    analogWrite(DAC0, TX_LEVELS[boundSignal[posStartBuff++]]);
    //se ho finito il rilevamento soglie passo al messaggio
    if(posStartBuff == BOUND_LEN){
      posStartBuff = 0;
      status = 3;
    }
  }
  //stato 3 → messaggio
  else if(status == 3){
    //se ci sono simboli da trasmettere
    if(symbols > 0){
      //regolo il led, incremento la posizione e aggiorno il contatore dei simboli validi
      analogWrite(DAC0, txValues[posRTxValues++]);
      posRTxValues %= BUFF_SIZE;
      symbols--;
    }
    //altrimenti se non ci sono simboli trasmetto 0
    else {
      analogWrite(DAC0, 0);
    }
    //aggiorno il contatore dei simboli trasmessi
    transmittedSymbols++;
    //se ho finito il messaggio
    if(transmittedSymbols == SPM){
      //azzero i simboli trasmessi
      transmittedSymbols = 0;
      //se ci sono ancora simboli validi torno alla sincronizzazione, altrimenti allo stato 0
      if(symbols > 0){
        status = 1;
      } else {
        status = 0;
        //interrompo il timer
        Timer3.stop();
      }
    }
  }
}

void loop() {
  //se ci sono dati disponibili in seriale e c'è spazio nel buffer...
  if(Serial.available() && (status == 0 || BUFF_SIZE - symbols > 3)) {
    //leggo il primo byte disponibile
    byte processedByte = Serial.read();
    //estraggo le diverse coppie di bit e inserisco nel buffer di scrittura il valore per il DAC corrispondente
    txValues[posWTxValues++] = TX_LEVELS[(processedByte & 0b11000000) >> 6];
    posWTxValues %= BUFF_SIZE;
    symbols++;
    txValues[posWTxValues++] = TX_LEVELS[(processedByte & 0b00110000) >> 4];
    posWTxValues %= BUFF_SIZE;
    symbols++;
    txValues[posWTxValues++] = TX_LEVELS[(processedByte & 0b00001100) >> 2];
    posWTxValues %= BUFF_SIZE;
    symbols++;
    txValues[posWTxValues++] = TX_LEVELS[(processedByte & 0b00000011)];
    posWTxValues %= BUFF_SIZE;
    symbols++;
  }
  //se non ci sono trasmissioni in corso ed è stato scritto qualcosa nel buffer, aggiorno lo stato e avvio il timer
  if(status == 0 && symbols > 0) {
    status = 1;
    Timer3.start(TT);
  }
}