/*
Kort forklaring av koden:
Ved bruk av bevegeelsessensor og temperatursensor registrerer programmet om fuglekassa er bebodd, og tenner utelampen på 
fuglehuset når kriteriene for bebodd fuglekasse er oppfylt. Hvis fuglekassa ikke er bebodd eller blir fraflyttet slukkes utelampa.

Kassa regnes som bebodd dersom:
  - Steg 1: Bevegelsessensoren har registrert tilstrekkelig bevegelse (bevegelsesterskel: konstant i programmet).
  - Steg 2: Temperaturen i kassa har økt i etterkant av at bevegelsesterskelen er nådd (temperaturøkningsterskel: kontstant i programmet).
  - Steg 3: Temperaturøkningen er stabil over tid (terskel for tidsendring: konstant i programmet).

Kassa regnes som ubebodd dersom: 
  - Kun steg 1 er oppfylt, men ikke steg 2 og steg 3.
  - Dersom det går lang tid mellom utløsning av bevegelsessensoren antas det at det er ulike fugler som besøker kassa, 
    og telleren som holder oversikt over antall registreringer nullstilles (terskel for endring i tid - bevegelse: konstant i programmet).

Kassa regnes som fraflyttet dersom:
  - Kassa tidligere har vært bebodd.
  - Temperaturøkningen ikke er stabil over tid.

Seriell kommunikasjon og kontroll-LED er brukt for å holde oversikt over kodeflyt under kjøring av programmet (ved testing av koden).
*/

// VARIABELDEKLARASJON:
//Deklarering av variabler til input fra sensorer:
const int bevegelseSensor = 2;
const int temperaturSensor = A0;  

//Deklarering av variabler til output fra LED:
const int utelampe = 4;
const int kontrollLED = 13;

//Deklarerer variabel til bevegelse:
const int terskelBevegelse = 1000; //Konstant - Steg 1: Bevegelsesterskel

int sensorVerdiBevegelse = 0;
int antallRegistreringerBevegelse = 0;

//Deklarering av variabler til temperatur:
const float terskelTemperaturEndring = 2; //Konstant - Steg 2: Temperaturøkningsterskel

float temperatur = 0;
float startTemperatur = 0; 
float temperaturEndring = 0; 
float sensorVerdiTemperatur = 0; 

//Deklarering av variabler til tid (bevegelse):
const int terskelEndringTidBevegelse = 5000; //Konstant: Maksimal tid det kan gå før antallRegistreringerBevegelse nullstilles.

unsigned long tidForrigeBevegelse = 0;
unsigned long endringTidBevegelse = 0;

//Deklarering av variabler til tid (temperatur):
const int terskelEndringTidTemperatur = 5000; //Konstant - Steg 3: Terskel for tidsendring.
const int stabiliseringTid = 500; //Intervall for stabilisering av innlesing av temperatur.

unsigned long tidOverTemperaturTerskel = 0; 
unsigned long endringTidTemperatur = 0; 

void setup() {
  //Starter Seriell kommunikasjon:
  Serial.begin(9600);

  //Setter sensorene til input:
  pinMode(bevegelseSensor, INPUT);
  pinMode(temperaturSensor, INPUT);

  //Setter LED til output:
  pinMode(utelampe, OUTPUT);
  pinMode(kontrollLED, OUTPUT);
}

void loop() {  
  //Steg 1: Bevegelsesterskel
  RegistrerBevegelse(); //Bevegelse registreres helt til terskelen for bevegelse er nådd.

  if (antallRegistreringerBevegelse >= terskelBevegelse){
    //Printer temperatur:
    Serial.print("Temperatur: ");
    Serial.println(temperatur);

    SettStartTemperatur(); //Når bevegelsesterskelen er nådd registreres temperaturen i kassa som startTemperatur (skjer èn gang).

    //Steg 2: Temperaturendring i fuglekassa
    SjekkTemperaturEndring(); //Temperaturøkningen i fuglekassa sjekkes helt til den er stabilt høyere enn terskelen for temperaturendring.

    if (temperaturEndring > terskelTemperaturEndring){
      tidOverTemperaturTerskel = millis(); //Registrerer tidspunktet temperaturendringen oversteg terskel for temperaturendring.

      //Steg 3: Stabil temperaturendring over tid
      SjekkTemperaturEndringStabilitetOverTid(); //Sjekker om temperaturøkningen holder seg stabil over tid.

      //Fuglekassa antas å være bebodd.
      FuglekasseBebodd(); //Så lenge temperaturendringen holder seg over tid antas det at fuglekassa er bebodd.
    }
    else{
      FuglekasseIkkeBebodd(); //Dersom temperaturøkningen ikke er stor nok eller ikke holder seg stabil over tid, antas fuglekassa å ikke være bebodd.
    }
  }
  else{
    FuglekasseIkkeBebodd(); //Dersom terskelen for bevegelse ikke er nådd, antas fuglekassa å ikke være bebodd.
  }
}

void RegistrerBevegelse(){ //Metoden registrerer bevegelse helt til bevegelsesterskelen er nådd.
  while (antallRegistreringerBevegelse < terskelBevegelse){
    //Printer antall registrerte bevegelser:
    Serial.print("Antall registreringer - bevegelse: ");
    Serial.println(antallRegistreringerBevegelse);

    sensorVerdiBevegelse = digitalRead(bevegelseSensor);
    OkAntallRegistreringer();
  }
}

void OkAntallRegistreringer(){ //Metoden registrerer bevegelse ved høy sensorverdi, og sjekker tid siden forrige bevegelse ved lav sensorverdi.
  if (sensorVerdiBevegelse == HIGH){
    antallRegistreringerBevegelse ++;
    tidForrigeBevegelse = millis();
    digitalWrite(kontrollLED, HIGH); //Kontroll-LED skrus på når sensoren registrerer bevegelse.
  }
  else{
    SjekkEndringTidBevegelse();
    digitalWrite(kontrollLED, LOW); //Kontroll-LED skrus av når sensoren ikke registrerer bevegelse.
  }
}

void SjekkEndringTidBevegelse(){ //Metoden sjekker hvor lang tid det har gått siden forrige bevegelsesregistrering, og nullstiller antallRegistreringer
  // dersom det går for lang tid mellom registreringene.
  OppdaterEndringTidBevegelse();
  
  if (endringTidBevegelse >= terskelEndringTidBevegelse){
    antallRegistreringerBevegelse = 0;

    //Printer informasjon om antall registreringer:
    Serial.println("Antall registreringer er nullstilt!");  
    Serial.print("Antall registreringer: ");
    Serial.println(antallRegistreringerBevegelse);
  }
}

void OppdaterEndringTidBevegelse(){ //Metoden oppdaterer variabelen som holder endring i tid siden forrige bevegelse.
  endringTidBevegelse = millis() - tidForrigeBevegelse;
}

void SettStartTemperatur(){ //Metoden leser fra temperatursensoren og oppdaterer starttemperaturen dersom den ikke er satt enda.
  if (startTemperatur == 0){
    sensorVerdiTemperatur = analogRead(temperaturSensor);
    startTemperatur = (((sensorVerdiTemperatur/1024) * 5) - 0.5) * 100;

    //Printer informasjon om starttemperatur:
    Serial.print("Starttemperatur er oppdatert: ");
    Serial.println(startTemperatur);
    Serial.println();
  }
}

void SjekkTemperaturEndring(){ //Metoden sjekker om endring i temperatur er større enn terskel for temperaturendring, og om endringen er stabil.
  Serial.println("Kontrollerer endring i temperatur: ");
  digitalWrite(kontrollLED, LOW); //Kontroll-LED skrus av.

  OppdaterEndringTemperatur(); //Endring i temperatur oppdateres.

  while (temperaturEndring < terskelTemperaturEndring){
    //Printer start-temperatur, temperaturen nå og endring i temperatur:
    Serial.print("Start-temperatur: ");
    Serial.println(startTemperatur);
    Serial.print("Temperatur: ");
    Serial.println(temperatur);
    Serial.print("Endring i temperatur: ");
    Serial.println(temperaturEndring);
    Serial.println();

    OppdaterEndringTemperatur(); //Endring i temperatur oppdateres.
  }
  StabiliserEndringTemperatur(); //Når temperaturøkningen har nådd terskelen for endring i temperatur sjekkes det om endringen er stabil.
}

void StabiliserEndringTemperatur(){ //Metoden sjekker om temperaturøkningen er stabil, slik at små variasjoner over/under terskel ikke gir utslag.
  while (temperaturEndring > terskelTemperaturEndring && endringTidTemperatur < stabiliseringTid){
    //Printer informasjon om temperatur:
    Serial.println("Stabiliserer endring i temperatur ");
    Serial.print("Temperatur: ");
    Serial.println(temperatur);

    //Endring i temperatur og tid oppdateres:
    OppdaterEndringTemperatur();
    OppdaterEndringTidTemperatur();
  }
}

void OppdaterTemperatur(){ //Metoden leser inn fra temperatursensoren og regner om til temperatur i grader C.
  sensorVerdiTemperatur = analogRead(temperaturSensor);
  temperatur = (((sensorVerdiTemperatur/1024) * 5) - 0.5) * 100;
}

void OppdaterEndringTemperatur(){ //Metoden oppdaterer variabelen som holder endring i temperatur.
  OppdaterTemperatur();
  temperaturEndring = temperatur - startTemperatur;
}

void OppdaterEndringTidTemperatur(){ //Metoden oppdaterer variabelen som holder endring i tid fra temperaturendringsterskelen ble nådd.
  endringTidTemperatur = millis() - tidOverTemperaturTerskel;
}

void SjekkTemperaturEndringStabilitetOverTid(){ //Metoden sjekker om temperaturendringen holder seg stabil over tid.
  //Endring i temperatur og tid oppdateres:
  OppdaterEndringTemperatur();
  OppdaterEndringTidTemperatur();

  while (temperaturEndring > terskelTemperaturEndring && endringTidTemperatur < terskelEndringTidTemperatur){
    //Printer informasjon om temperatur og endring i tid:
    Serial.println("Kontrollerer tidsrom for endring i temperatur: ");
    Serial.print("Temperatur: ");
    Serial.println(temperatur);
    Serial.print("Endring i tid: ");
    Serial.println(endringTidTemperatur); 
    
    digitalWrite(kontrollLED, HIGH); //Kontroll-LED viser at stabilitet over tid sjekkes.

    //Endring i temperatur og tid oppdateres:
    OppdaterEndringTemperatur();
    OppdaterEndringTidTemperatur();
  }
}

void FuglekasseBebodd(){ //Fuglekassa antas å være bebodd så lenge temperaturøkningen holder seg over tid.
  while (temperaturEndring > terskelTemperaturEndring && endringTidTemperatur > terskelEndringTidTemperatur){
    Serial.println("Fuglekasse bebodd!");
    Serial.print("Temperatur: ");
    Serial.println(temperatur);

    SignalPaa(); //Signalet om at fuglekassa er bebodd skrus på.

    //Endring i temperatur og tid oppdateres:
    OppdaterEndringTemperatur();
    OppdaterEndringTidTemperatur();
  }
}

void SignalPaa(){ //Metoden skrur på signalet om at fuglekassa er bebodd.
  //Kontroll-LED skrus av og utelampa skrus på:
  digitalWrite(kontrollLED, LOW);
  digitalWrite(utelampe, HIGH);
}

void FuglekasseIkkeBebodd(){ //Hvis kriteriene for bebodd fuglekasse ikke er oppfylt antas det at fuglekassa ikke er bebodd.
  Serial.println("Fuglekasse ikke bebodd");
  Serial.print("Temperatur: ");
  Serial.println(temperatur);

  SignalAv(); //Signalet om at fuglekassa er bebodd skrus av
}

void SignalAv(){ //Metoden skrur av signalet om at fuglekassa er bebodd.
  //Både kontrollLED og utelampe skrus av:
  digitalWrite(utelampe, LOW);
  digitalWrite(kontrollLED, LOW);
}
