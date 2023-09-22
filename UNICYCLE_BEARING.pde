/* //<>//
  NOME: LORENZO
  COGNOME: CASAVECCHIA
  MATRICOLA: 0291937
  
  ISTRUZIONI PER L'USO:
  COME DA SPECIFICA QUESTO SCRIPT PERMETTE LA SIMULAZIONE DI LOCALIZZAZIONE TRAMITE EKF
  DI UN ROBOT DI TIPO UNICICLO
  SULLA SCENA SONO DISPOSTI A CIRCONFERENZA 5 LANDMARKS RESPONSABILI DI MISURARE
  LA POSIZIONE DELL'UNICICLO (AVREI POTUTO METTERNE 10, MA GIA' AD 8 LANDMARKS
  LA SIMULAZIONE DIVENTA ALQUANTO LENTA)
  NELLO SPECIFICO L'I-ESIMO LANDMARK MISURERA' L'ANGOLO DI BEARING COMPRESO TRA
  L'ASSE DI MOVIMENTO DEL ROBOT E LA CONGIUNGENTE ROBOT-LANDMARK, INTESO COME ROTAZIONE
  (QUESTO PER DIRE CHE SARA' POSITIVO SE LA ROTAZIONE DOVESSE AVVENIRE IN SENSO ANTI-ORARIO,
  NEGATIVO ALTRIMENTI)
  PER OGNI CICLO D'ESECUZIONE DELLA FUNZIONE DRAW L'I-ESIMO LANDMARK PUO' ESSERE
  * SPENTO SE L'UTENTE, DOVESSE AVER PREMUTO L'I-ESIMO TASTO NUMERICO
  DELLA TASTIERA
  * INVISIBILE DI DEFAULT O PREMENDO NUOVAMENTE L'I-ESIMO TASTO NUMERICO DELLA TASTIERA,
  QUALORA IL LANDMARK DOVESSERE ESSERE STATO SPENTO PRECEDENTEMENTE
 * VISIBILE SE PRECEDENTEMENTE INVISIBILE MA COMUNQUE NEL RANGE DI VISIBILITA' DEL ROBOT
 IL RANGE DI VISIBILITA' DEL ROBOT CONSISTE IN UN ARCO DI CIRCONFERENZA SIMMETRICO RISPETTO
 ALAL DIREZIONE DI MOVIMENTO DEL ROBOT ED IL CUI RAGGIO PUO'
 ESSERE MODIFICATO TRAMITE TASTI 'r' ED 'R' DELLA TASTIERA (IL PRIMO PER DIMINUIRLO, IL SECONDO
 PER AUMENTARLO) MENTRE L'ANGOLO DI APERTURA PUO' ESSERE RISTRETTO CON TASTO 'b' E AMPLIATO
 CON TASTO 'B'
 IL VALOR MINIMO PER IL RAGGIO DI VISIBILITA' E' IMPOSTATO A 120 PIXELS, MENTRE VALORI MINIMO E
 MASSIMO DELL'ANGOLO DI APERTURA SONO RISPETTIVAMENTE DI 15° E 240°
 CLICCARE CON IL MOUSE UNO SPECIFICO PUNTO DELLA SCENA PORTERA' AL MOVIMENTO DEL ROBOT E DELLA SUA STIMA
 VERSO QUELLA POSIZIONE
*/

// Dimensioni finestra
int sizeX = 1250;
int sizeY =  950;

// Coordinate attuali uniciclo
float x = 0;
float y = 0;
float theta = 0;

// Coordinate desiderate uniciclo
float xDes = 0;
float yDes = 0;

// Caratteristiche fisiche uniciclo
float r = 8; // raggio ruote in pixel
float d = 25; // distanza tra le ruote in pixel
float w = 5; // spessore ruota in pixel
float R = 1.2 * sqrt(pow(r, 2) + pow(d / 2 + w / 2, 2)); // raggio robot

float dt = 1./60; // tempo di campionamento

float e_p = 0; // errore posizionamento
float v1 = 0; // velocità lineare robot
float kv1 = 1; // costante legge proporzionale controllo v1
float v2 = 0; // velocità rotazionale robot
float kv2 = 2;  // costante legge proporzionale controllo v2
int nGiri = 0; // conta quanti giri su se stesso ha fatto il robot
float thetaDes; // orientamento desiderato (per raggiungere il target)
float t0, tempo; // tempo inizio e attuale missione
float tempoUltimaMisura; // tempo in cui si è effettuata l'ultima misura
long nStime = 0; // numero progressivo totale delle stime fatte

float omegaR = 0; // velocità angolare ruota destra
float omegaL = 0; // velocità angolare ruota sinistra
float uR, uL, uRe, uLe; // spostamenti ruote destra e sinistra veri e presunti (comandati)

// Variabili relative alla stima e al filtro di Kalman esteso
float sigmaX0 = 10; // deviazione standard dell'errore di stima del valore iniziale di x0
float sigmaY0 = 10; // deviazione standard dell'errore di stima del valore iniziale di y0
float sigmaTheta0 = radians(10); // deviazione standard (in rad) dell'errore di stima del valore iniziale di theta0

float xHat = x + Gaussian(0, sigmaX0); // stima di x inizializzata al valore vero + una perturbazione gaussiana a media 0 e deviazione standard sigmaX0
float yHat = y + Gaussian(0, sigmaY0); // stima di y inizializzata al valore vero + una perturbazione gaussiana a media 0 e deviazione standard sigmaY0
float thetaHat = theta + Gaussian(0,sigmaTheta0); // stima di theta inizializzata al valore vero + una perturbazione gaussiana a media 0 e deviazione standard sigmaTheta0 rad

float xHatMeno, yHatMeno, thetaHatMeno; // stime a priori di x,y,theta

float KR = 0.01; // coefficiente varianza errore odometrico ruota destra
float KL = KR; // coefficiente varianza errore odometrico ruota sinistra

float stDevR,stDevL; // deviazione standard errore odometrico ruota destra e sinistra

int nLmax = 5;
float [][] Landmark = new float[nLmax][2];
int nL = nLmax;
int nLprev = nL;

// Seguono le matrici utilizzate dal filtro di Kalman esteso (EKF)
float[][] F = {{1, 0, 0},{0, 1, 0},{0, 0, 1}}; // matrice giacobiana F=df/dx (alcuni elementi delle prime due righe vanno aggiustati durante l'esecuzione)
float[][] P = {{pow(sigmaX0, 2), 0, 0}, {0, pow(sigmaY0, 2), 0}, {0, 0, pow(sigmaTheta0,2)}}; // matrice di covarianza P inizializzata in base all'incertezza iniziale

float[][] Pmeno = new float[3][3]; // matrice di covarianza a priori P-

float[][] W = {{1, 0},{0,1}, {1 / d, - 1 / d}}; //  matrice giacobiana W = df/dw (gli elementi delle prime due righe vanno aggiustati durante l'esecuzione) 
float[][] Q = {{1, 0},{0,1}}; // matrice di covarianza del rumore di misura odometrico w (gli elementi sulla diagonale vanno aggiustati durante l'esecuzione)

float DeltaX, DeltaY, DeltaXY; // variabili di supporto
float sigmaLandmark = radians(10); // deviazione standard errore di misura dell'angolo rispetto ai landmark (in radianti)

float[][] correzione = new float[3][1]; // termine correttivo stima
float tStep = 1 / 60; // tempo (in ms) tra una misura e la successiva (impostabile da tastiera)

// PARAMETRI PER LA REGIONE DI VISIBILITA' DELL'UNICICLO
float visBeta = PI / 2;
float visR = 120;

// ARRAY DI STATO DI VISIBILITA' DEI LANDMARKS
int []isVis = new int[nLmax];

// DISTANZA DEI LANDMARK DAL CENTRO DELLA SCENA
float landmarkRadius = .5 * sizeY;

// SCALE DI GRIGIO PER I LANDMARKS (VISIBILE, SPENTO E INVISIBILE RISPETTIVAMENTE)
color lVis = 255, lOff = 0, lInv = 100;

// VARIABILI UTILI AL CAMBIO DI STATO DEI LANDMARKS
int waitRelease, k;

float[] misureLandmark = new float[nL]; // vettore con le misure vere di distanza dagli nL landmark
float[] misureAtteseLandmark = new float[nL]; // vettore con le misure attese di distanza dagli nL landmark

float[][] H = new float[nL][3]; // matrice giacobiana H = dh/dx
float[][] K = new float[3][nL]; // guadagno di Kalman
float[][] Rs = new float[nL][nL]; // matrice di covarianza errore misura dai landmark
    
float[][] innovazione = new float[nL][1]; // innovazione EKF

void setup() 
{
  size(1250, 950);
  tempoUltimaMisura = 0; // Inizializzo a zero il tempo in cui è stata effettuata l'ultima misura
  
  for(int i = 0; i < nLmax; i++){
    isVis[i] = 0; // INIZIALIZZO TUTTI I LANDMARKS A NON-VISIBILE
    
    Landmark[i][0] = landmarkRadius * cos(i * TAU / nLmax);
    Landmark[i][1] = landmarkRadius * sin(i * TAU / nLmax);
  }
  
  xDes = yDes = 0;
}

void draw() 
{
  background(0);

  pushMatrix();
  translate(sizeX / 2,sizeY / 2);
  
  if (keyPressed)
  {
    if (keyCode == UP) // aumento di 1 il tempo tra una misura e la successiva
      tStep += 1;
    if (keyCode == DOWN)  // decremento di 1 il tempo tra una misura e la successiva
      tStep = max(0, tStep - 1);
    if (keyCode == RIGHT) // moltiplico per due il tempo tra una lettura e la successiva
      tStep = tStep * 2;
    if (keyCode == LEFT) // divido per due il tempo tra una lettura e la successiva
      tStep = tStep / 2;
    switch(key){
      case 'R':
        visR  += 10;
        break;
      case 'r':
        if(visR < 120)
          break;
        visR -= 10;
        break;
      case 'B':
        if(visBeta >= 2 * TAU / 3) 
          break;
        visBeta += .1;
        break;
      case 'b':
        if(visBeta <= PI / 12) 
          break;
        visBeta -= .1;
        break;
      default:
        // NEL CASO PIU' GENERALE L'UTENTE PUO' ATTIVARE O DISATTIVARE
        // UNO TRA GLI NLMAX LANDMAKRS, IL CHE VUOL DIRE CHE DOVREBBE 
        // POTER ACCEDERE AI TASTI DI TASTIERA 0, ..., NLMAX (ASSUMENDO
        // QUESTO SIA <= 9 E >= 0)
        
        // COME PRIMO PASSO CONVERTO KEY IN UNA STRINGA E CONTROLLO
        // SE IL SUO FORMATO E' ALFA NUMERICO (IN QUESTO CASO DEVE ESSERE NUMERICO
        // E COMPRESO TRA 0 E 9
        if((key + "").matches("-?[0-9]+")){
          // SE COSI' FOSSE ALLORA CONVERTO IL VALORE DELLA STRINGA
          // KEY + "" IN UN INTERO E LO SALVO IN K (K DICE CHE TASTO NUMERICO
          // HO PREMUTO DA TASTIERA E VERRA' USATO COME INDICE PER IMPOSTARE
          // LO STATO DI VISIBILITA' DEL K-ESIMO LANDMARK)
          k = Integer.parseInt(key + "");
          
          // CAMBIO DELLO STATO DEL LANDMARK SCELTO (NOTARE IL COMPORTAMENTO FLIP FLOP
          // DELLO STATO DEI LANDMARKS)
          if(k < nLmax && waitRelease != k){
            waitRelease = k;
            isVis[k] = (isVis[k] == -1 ? 0 : -1);
          }
        } else
          break;
    }
  } else
    waitRelease = -1;
  
  if (mousePressed) // assegno target
  {
    xDes = mouseX - sizeX / 2;
    yDes = sizeY / 2 - mouseY;
    
    t0 = millis(); // inizio conteggio del tempo di missione
  }
  
  for(int indLandmark = 0; indLandmark < nLmax; indLandmark++){
    // SE IL LANDMARK E' SPENTO, NON VERIFICARE CHE SIA VISIBILE
    if(isVis[indLandmark] != -1){
      DeltaY = Landmark[indLandmark][1] - y;
      DeltaX = Landmark[indLandmark][0] - x;
      DeltaXY = pow(DeltaX, 2) + pow(DeltaY, 2);
      
      // VERIFICO CHE AMBE DISTANZA DAL LANDMARK E ANGOLO TRA LANDMARK E ROBOT
      // SIANO RISPETTIVAMENTE MINORI DEL RAGGIO E COMPRESI TRA THETA - .5 * VISBETA E
      // THETA + .5 * VISBETA (LANDMARK VISIBILE)
      if(DeltaXY <= pow(.5 * visR, 2) && abs(acos(cos((theta - atan2(DeltaY, DeltaX))))) <= .5 * visBeta)
        isVis[indLandmark] = 1;
      else
        isVis[indLandmark] = 0;
    }
  }
  
  // Calcolo errore e controllo basandomi sul valore stimato (xHat,yHat,thetaHat) delle variabili dell'uniciclo
  e_p = sqrt(pow(xDes - xHat, 2) + pow(yDes - yHat, 2));
  
  if (e_p > 1) // mi muovo solo se l'errore è maggiore di una certa quantità
  {
    tempo = (millis() - t0) / 1000;  // tempo missione in secondi

    // assegno velocità secondo legge proporzionale (in termini delle quantità stimate!)
    v1 = - kv1 * ((xHat - xDes) * cos(thetaHat) + (yHat - yDes) * sin(thetaHat));

    // Calcolo l'angolo verso il target: scelgo il multiplo di 2PI 
    // più vicino all'orientamento corrente ***stimato*** del robot
    thetaDes = atan2(yDes - yHat, xDes - xHat) + nGiri * TAU;
    
    if (abs(thetaDes + TAU - thetaHat) < abs(thetaDes - thetaHat))
    {
      thetaDes += TAU;
      nGiri += 1;
    }else
      if (abs(thetaDes - TAU - thetaHat) < abs(thetaDes - thetaHat)){
        thetaDes -= TAU;
        nGiri -= 1;
      }
      
   // assegno velocità angolare secondo legge proporzionale sempre in funzione delle quantità stimate   
    v2 = kv2 * (thetaDes - thetaHat);
  }
  else // se penso di essere vicino al target mi fermo
    v1 = v2 = 0;
  
  // Calcolo i movimenti da impartire alle ruote in base alle v1 e v2 trovate
  omegaR = (v1 + v2 * d / 2) / r;
  omegaL = (v1 - v2 * d / 2) / r;
  
  uRe = r * omegaR * dt; // spostamento comandato alla ruota destra (da considerarsi anche come informazione odometrica disponibile sul suo spostamento in (t,t+dt))
  uLe = r * omegaL * dt; // spostamento comandato alla ruota sinistra (da considerarsi anche come informazione odometrica disponibile sul suo spostamento in (t,t+dt))

  // Perturbo i due movimenti: gli spostamenti reali delle ruote non saranno mai esattamente uguali a quelli comandati 
  stDevR = sqrt(KR * abs(uRe));
  stDevL = sqrt(KL * abs(uLe));  
  
  uR = uRe + Gaussian(0, stDevR); // Spostamento vero ruota destra
  uL = uLe + Gaussian(0, stDevL); // Spostamento vero ruota sinistra
  
  // Dinamica effettiva dell'uniciclo
  x += ((uR + uL) / 2) * cos(theta);
  y += ((uR + uL) / 2) * sin(theta);
  theta += (uR - uL) / d;
  
  // STIMA FILTRO KALMAN ESTESO: PASSO di PREDIZIONE
  xHatMeno = xHat + ((uRe + uLe) / 2) * cos(thetaHat);
  yHatMeno = yHat + ((uRe + uLe) / 2) * sin(thetaHat);
  thetaHatMeno = thetaHat + (uRe - uLe) / d;

  //Aggiorno la giacobiana F (solo gli elementi variabili)
  F[0][2] = - (uRe + uLe) * sin(thetaHat) / 2;
  F[1][2] = (uRe + uLe) * cos(thetaHat) / 2;
  
  // Aggiorno W (solo gli elementi variabili)
  W[0][0] = .5 * cos(thetaHat);
  W[0][1] = .5 * cos(thetaHat);
  W[1][0] = .5 * sin(thetaHat);
  W[1][1] = .5 * sin(thetaHat);

  //Aggiorno Q (solo gli elementi variabili)
  Q[0][0] = KR * abs(uRe);
  Q[1][1] = KL * abs(uLe);
  
  // Calcolo matrice covarianza a priori
  Pmeno = mSum(mProd(mProd(F, P), trasposta(F)), mProd(mProd(W, Q), trasposta(W)));
   
  nL = countVisibleLandmarks();
  
  if(nL != nLprev && nL != 0){
    // DEFINISCO I VETTORI E LE MATRICI CHE DIPENDONO
    // IN TAGLIA DAL NUMERO DEI LANDMARKS 
    misureLandmark = new float[nL]; // vettore con le misure vere di distanza dagli nL landmark
    misureAtteseLandmark = new float[nL]; // vettore con le misure attese di distanza dagli nL landmark
    
    H = new float[nL][3]; // matrice giacobiana H = dh/dx
    K = new float[3][nL]; // guadagno di Kalman
    
    Rs = new float[nL][nL];
    Rs = idMat(nL, pow(sigmaLandmark, 2)); // matrice di covarianza errore misura dai landmark
    
    innovazione = new float[nL][1]; // innovazione EKF
  }
  
  // STIMA FILTRO DI KALMAN ESTESO: PASSO di CORREZIONE
  if (millis() - tempoUltimaMisura >= tStep && nL != 0) // attuo la correzione solo se ho le misure (che arrivano ogni tStep ms)
  {
    tempoUltimaMisura = millis(); // memorizzo il tempo in cui ho fatto l'ultima misura
    nStime++; // incremento il contatore delle stime fatte
     
    int indAux = 0, exit = 0;
    for (int indLandmark = 0; indLandmark < nL && exit != 1; indLandmark++) 
    {
      // SICCOME SO CHE, PER QUESTO CICLO DI DRAW, SONO VISIBILI E ATTIVI
      // NL LANDMARKS MA NON SO QUALI DI ESSI LO SIANO IMPOSTO 2 CONTATORI
      // INDAUX SERVIRA' PER CICLARE SUL VETTORE ISVIS E QUINDI TROVERA'
      // IL PROSSIMO (QUELLO CON INDICE INDLANDMARK) LANDMARK VISIBILE
      while(isVis[indAux] != 1){
        indAux++;
        if(indAux >= nLmax - 1){
          exit = 1;
          break;
        }
      }
      
      misureLandmark[indLandmark] = h(x, y, theta, indAux) + Gaussian(0, sigmaLandmark);
      misureAtteseLandmark[indLandmark] = h(xHatMeno, yHatMeno, thetaHatMeno, indAux);
      
      DeltaX = Landmark[indAux][0] - xHatMeno;
      DeltaY = Landmark[indAux][1] - yHatMeno;
      DeltaXY = pow(DeltaX, 2) + pow(DeltaY, 2);
      
      H[indLandmark][0] = DeltaY / DeltaXY;
      H[indLandmark][1] = - DeltaX / DeltaXY;  
      H[indLandmark][2] = - 1;
      
      float itemp = misureLandmark[indLandmark] - misureAtteseLandmark[indLandmark];
      innovazione[indLandmark][0] = atan2(sin(itemp), cos(itemp));
    }
    
    // Calcolo guadagno Kalman e aggiorno covarianza
    K = mProd(mProd(Pmeno, trasposta(H)), invMat(mSum(mProd(mProd(H, Pmeno), trasposta(H)), Rs)));
    P = mProd(mSum(idMat(3, 1), mProd(idMat(3, - 1), mProd(K, H))), Pmeno);
    
    // Correggo la stima    
    correzione = mProd(K, innovazione);
    
    // PROBLEMA SU CORREZIONE
    xHat = xHatMeno + correzione[0][0];
    yHat = yHatMeno + correzione[1][0];
    thetaHat = thetaHatMeno + correzione[2][0];
  }
  else  // se non ho misure non correggo nulla
  {
    xHat = xHatMeno;
    yHat = yHatMeno;
    thetaHat = thetaHatMeno;
    P = Pmeno;
  }
  // FINE EKF
  
  // SALVO IL VALORE PRECEDENTE DI NL COSI' CHE,
  // SE NON DOVESSE VARIARE TRA UN CICLO E QUELLO SUCCESSIVO,
  // POSSO RISPARMIARMI DI RE-DEFINIRE LE MATRICI CHE IN TAGLIA DIPENDONO DA NL
  nLprev = nL; 
  
  // DISEGNO ROBOT REALE, ROBOT STIMATO, I LANDMARKS E STAMPO 
  // A SCHERMO TUTTE LE INFORMAZIONI DEI ROBOT NELLA SCENA
  robot(x, y, theta, 1);
  robot(xHat, yHat, thetaHat, 0);
  drawLandmarks();
  printAllInfo();
}

void drawLandmarks(){ 
  stroke(255, 100, 0);
  strokeWeight(2);
  
  for (int indLandmark = 0; indLandmark < nLmax; indLandmark++)
  {
    switch(isVis[indLandmark]){
      case 0:
        fill(lInv);
        break;
      case -1:
        fill(lOff);
        break;
      default:
        fill(lVis);
    }
    
    triangle(Landmark[indLandmark][0] - 15, - Landmark[indLandmark][1] + 15, Landmark[indLandmark][0] + 15, - Landmark[indLandmark][1] + 15, Landmark[indLandmark][0], - Landmark[indLandmark][1] - 15);
    
    if(isVis[indLandmark] != -1){
      textSize(10);
      fill(0, 0, 0);
      text("L", Landmark[indLandmark][0] - 5, - Landmark[indLandmark][1] + 8);
      text(indLandmark, Landmark[indLandmark][0] + 1, - Landmark[indLandmark][1] + 8);
    }  
  }
}

void printAllInfo(){  
  stroke(0);
  strokeWeight(1);
  
  popMatrix();

  textSize(20);
  fill(0, 100, 255);
  text("v1 (pixel/s) = ", 10, 20); 
  text(v1, 200, 20);
  text("v2 (gradi/s) = ", 10, 50); 
  text(degrees(v1), 200, 50);
  
  fill(255, 100, 0);  
  text("x = ", 10, 160); 
  text(x, 80, 160);
  text("y = ", 10, 190); 
  text(y, 80, 190);
  text("theta = ", 10, 220); 
  text(degrees(theta), 100, 220);  

  fill(255, 255, 255);
  text("tempo = ", 10, 110); 
  text(tempo, 120, 110);  
  text("nGiri = ", 700, 110); 
  text(nGiri, 800, 110);  

  fill(0, 100, 255);
  text("omegaR (gradi/s) = ", 700, 20); 
  text(degrees(omegaL), 900, 20);
  text("omegaL (gradi/s) = ", 700, 50); 
  text(degrees(omegaL), 900, 50);
  
  fill(255, 100, 0);
  text("xDes = ", 700, 160); 
  text(xDes, 800, 160);
  text("yDes = ", 700, 190); 
  text(yDes, 800, 190);
  text("thetaDes = ", 700, 220);
  text(degrees(thetaDes), 800, 220);
  
  fill(255, 255, 100);  
  text("xHat = ", 10, 280); 
  text(xHat, 120, 280);
  text("yHat = ", 10, 310); 
  text(yHat, 120, 310);
  text("thetaHat = ", 10, 340); 
  text(degrees(thetaHat), 160, 340);  

  fill(255, 255, 255);
  text("nStime = ", 10, 390); 
  text(nStime, 120, 390);  

  fill(255, 255, 255);
  text("tStep (ms) = ", 10, 420); 
  text(tStep, 150, 420);  

  fill(255, 255, 100);  
  text("P = ", 10, 460); 
  text(P[0][0], 10, 490); text(P[0][1], 100, 490); text(P[0][2], 190, 490);
  text(P[1][0], 10, 520); text(P[1][1], 100, 520); text(P[1][2], 190, 520); 
  text(P[2][0], 10, 550); text(P[2][1], 100, 550); text(P[2][2], 190, 550);   
}

float h(float xAtt, float yAtt, float thetaAtt, int indLandmark){
  // I TERMINI + PI E + NGIRI * TAU VENGONO USATI PER AGGIUSTANRE IL VALORE DELL'ANGOLO RITORNATO COSI' CHE
  // ESSO NON VADA TROPPO OLTRE 2 * PI E - 2 * PI E RENDERE LA MISURA RITORNATA AL LANDMARK PIU' COERENTE
  // CON QUELLA RICHIESTA DA SPECIFICA CIOE' L'ANGOLO CHE IL ROBOT HA CON RISPETTO AL LANDMARK (E NON IL VICEVERSA
  // COME INVECE ALLUDEREBBE LA FORMULA SOTTO RIPORTATA SENZA I TERMINI AGGIUNTIVI)
  return(atan2(Landmark[indLandmark][1] - yAtt, Landmark[indLandmark][0] - xAtt) - thetaAtt);
}

int countVisibleLandmarks(){
  int nLs = 0;
  for(int i = 0; i < nLmax; i++)
    if(isVis[i] == 1)
      nLs++;
  
  return nLs;
}

void robot(float x, float y, float theta, int colore)
{ 
  pushMatrix();
  translate(x, - y);
  rotate(- theta);
  
  if (colore == 1){
    // DISEGNO L'AREA DI VISIBILITA' PER LA POSA REALE DELL'UNICICLO (ROSSO)
    fill(100, 255, 0, 100);
    arc(0, 0, visR, visR, - .5 * visBeta, .5 * visBeta);
    
    fill(255, 100, 0);
  } else
    fill(255, 255, 100);
  
  ellipse(0, 0, 2 * R, 2 * R);
  fill(0, 100, 255);  
  rect(- r, - d / 2 - w / 2, 2 * r, w);
  rect(- r, d / 2 - w / 2, 2 * r, w);
  fill(255);
  ellipse(.8 * R , 0, .2 * R, .2 * R);
  ellipse(- .8 * R, 0, .2 * R, .2 * R);  
  fill(100, 255, 0);
  triangle(- .1 * R, .3 * R, - .1 * R, - .3 * R, .5 * R, 0);
  popMatrix();
}

float Gaussian(float media, float stDev) // Restituisce variabile N(media, stDev^2) approssimata sfruttando il teorema del limite centrale
{
  float somma = 0;
  for (int k = 0; k < 27; k++) // 27 in modo che sqrt(3/27)=1/3
    somma += random(- stDev / 3, stDev / 3);

  return media + somma;
}

float[][] mProd(float[][] A, float[][] B) // Calcola prodotto di due matrici A e B (si assume, senza controllarlo, che numero colonne A = numero righe B!)
{
  int nA = A.length;
  int nAB = A[0].length;
  int nB = B[0].length;

  float[][] C = new float[nA][nB]; 

  for (int i=0; i < nA; i++) 
    for (int j=0; j < nB; j++)  
      for (int k=0; k < nAB; k++)
        C[i][j] += A[i][k] * B[k][j];
  return C;
}

float[][] mSum(float[][] A,float[][] B) // Calcola la somma di due matrici A e B (si assume, senza controllarlo, che A e B abbiano stesse dimensioni!)
{
  int nA = A.length;
  int nB = A[0].length;
  
  float[][] C = new float[nA][nB]; 

  for (int i=0; i < nA; i++) 
    for (int j=0; j < nB; j++) 
      C[i][j] = A[i][j] + B[i][j];
      
  return C;
}

float[][] trasposta(float[][] A) // Calcola la trasposta di una matrice A
{
  // HO NOTATO CHE, NELLA FUNZIONE TRASPOSTA TROVATA NEL FILE KALMANUNICICLO.PDE
  // C'ERA UN CONTROLLO SUL NUMERO DI RIGHE DI A
  // NELLO SPECIFICO
  // SE NR <= 1 ALLORA RITORNA A, IL CHE E' NON VERO PER VETTORI RIGA
  // O VETTORI COLONNA (PRESUMO CHE L'IDEA DI QUESTA FUNZIONE FOSSE
  // CALCOLARE LA TRASPOSTA PER MATRICI QUADRATE)
  int nR = A.length;
  int nC = A[0].length; 
  
  float[][] C = new float[nC][nR]; 

  for (int i=0; i < nC; i++) 
    for (int j=0; j < nR; j++) 
      C[i][j] = A[j][i];

  return C;
}


float[][] minore(float[][] A, int i, int j) // Determina il minore (i,j) di una matrice A (si assume, senza controllarlo, che A sia quadrata!)
{
  int nA = A.length;
  float[][] C = new float[nA-1][nA-1];
  
  for (int iM = 0; iM < i; iM++)
  {
    for (int jM = 0; jM < j; jM++)
      C[iM][jM] = A[iM][jM];
    for (int jM = j; jM < nA-1; jM++)
      C[iM][jM] = A[iM][jM+1];
  }
  for (int iM = i; iM < nA-1; iM++)
  {
    for (int jM = 0; jM < j; jM++)
      C[iM][jM] = A[iM+1][jM];
    for (int jM = j; jM < nA-1; jM++)
      C[iM][jM] = A[iM+1][jM+1]; 
  }
  return C;
}


float det(float[][] A) // Calcola il determinante di A (si assume, senza controllarlo, che A sia quadrata!)
{
  int nA = A.length;
  float determinante = 0;
  
  if (nA == 1)
    determinante = A[0][0];
  else
    for (int j=0; j < nA; j++) 
      determinante = determinante + A[0][j] * pow(- 1, j) * det(minore(A, 0, j));
      
  return determinante;
}


float[][] invMat(float[][] A) // Calcola l'inversa di una matrice A (si assume, senza controllarlo, che A sia quadrata!)
{
  int nA = A.length;
  float[][] C = new float[nA][nA];
  float detA = det(A);
  
  if (nA == 1)
    C[0][0] = 1/detA;
  else
    for (int i=0; i < nA; i++) 
      for (int j=0; j < nA; j++) 
        C[j][i] = pow(-1, i + j) * det(minore(A, i, j)) / detA;

  return C;
}

float[][] idMat(int nA, float sigma) // Assegna una matrice identità di ordine nA moltiplicata per una costante sigma
{
  float[][] I = new float[nA][nA]; 

  for (int i=0; i < nA; i++) {
    for (int j=0; j < nA; j++)  
      I[i][j] = 0;

    I[i][i] = sigma;
  }
  return I;
}
