/*
  NOME: LORENZO
  COGNOME: CASAVECCHIA
  MATRICOLA: 0291937
  
  ISTRUZIONI PER L'USO:
  
  COME DA SPECIFICA CON QUESTO SCRIPT È POSSIBILE
    * MODIFICARE LA POSIZIONE DESIDERATA
    DELLA PINZA CON I TASTI 'x', 'X', 'y', 'Y', 'z', 'Z' RISPETTIVAMENTE
    PER LE COORDINATE X, Y, Z (MINUSCOLO PER DIMINUIRNE IL VALORE, MAIUSCOLO
    PER AUMENTARLO)
    * MODIFICARE GLI ANGOLI DI AZIMUTH, ELEVAZIONE E ROLLIO DELLA PINZA
    CON I TASTI 'a', 'A', 'b', 'B', 't', 'T'
    (RISPETTIVAMENTE PER DIMINUIRE ED AUMENTARE LA MISURA DEGLI ANGOLI
    ALFA, BETA E THETA)
    * MUOVERE LA BASE DEL ROBOT ALLA POSIZIONE DEL MOUSE SUL CLICK
    * SCEGLIERE LA CONFIGURAZIONE A GOMITO BASSO E GOMITO ALTO PREMENDO
    RISPETTIVAMENTE '-' E '+'
  
  ALTRI PARAMETRI MODIFICABILI DALL'UTENTE SONO
    * IL VALORE DELLA COSTANTE KP RISPETTIVAMENTE CON TASTI 'k' (DECREMENTO)
    E 'K' (INCREMENTO)
    * L'ALTEZZA DELLA CAMERA RISPETTO ALLA BASE DEL ROBOT CON LA FRECCIA VERSO L'ALTO
    E VERSO IL BASSO (PER ALZARLA ED ABBASSARLA)
    * LA ROTAZIONE ATTORNO L'ASSE PARALLELO AD Y PASSANTE PER IL CENTRO DELLA SCENA
    DELLA CAMERA (A DIFFERENZA DAGLI SKETCHES MOSTRATI A LEZIONE LA CAMERA
    SI MUOVE SU UN CILINDRO DI RAGGIO EYER CENTRATO NEL CENTRO DELLA SCENA (WIDTH / 2,
    HEIGHT / 2, 0))
    * LA MODIFICA DELLA VARIABILE EYER TRAMITE TASTI 'q' E 'Q' (AVVICINAMENTO ED
    ALLONTANAMENTO DAL CENTRO DELA SCENA RISPETTIVAMENTE)
  
  PREMENDO TASTI 'v' O 'V' È POSSIBILE INOLTRE MUOVERE LA CAMERA E LA
  BASE DEL ROBOT ALLA CONFIGURAZIONE INIZIALE
  (QUELLA AL PRIMO AVVIO DELLO SKETCH)
  
  PREMENDO TASTI 'r' O 'R' È POSSIBILE REIMPOSTARE LA SOLA BASE DEL
  ROBOT (PRESERVANDO QUINDI ANGOLI DESIDERATI E POSIZIONE DELLA PINZA
  
  TUTTI I LINK SONO BIANCHI CON OPACITÀ 100
  
  TUTTI I GIUNTI (TRANNE LA BASE E LA PINZA) SONO BIANCHI CON OPACITÀ 255
  
  IL GIUNTO ASSOCIATO ALLA BASE È GIALLO MENTRE QUELLO ALLA PINZA È FUCSIA
  
  GLI ASSI X, Y, Z SONO COLORATI RISPETTIVAMENTE ROSSO, VERDE E BLU
  
  LO SFONDO È NERO 0 O GRIGIO 100 IN CASO DI IRRAGGIUNGIBILITÀ DELLA PINZA
  
  BUG IMPLEMENTATIVI:
    * PER CERTI VALORI DELLA POSIZIONE DESIDERATA E ANGOLI DI ROTAZIONE, MINIME
    VARIAZIONI DEI PARAMETRI PORTANO A SCATTI MOVIMENTI A SCATTI NEI LINK DEL ROBOT
    (LA FUNZIONE ATAN2 NON È CONTINUA SUL DOMINIO DEI POSSIBILI ANGOLI)
    * QUESTO PROBLEMA PUÒ ESSERE MITIGATO NORMALIZZANDO DI VOLTA IN VOLTA
    I VALORI DI ALPHA, BETA E THETA IN MODO CHE RISULTINO TRA - 2 * PI E 2 * PI
    * UN ALTRO MODO PER POTERLO MITIGARE POTREBBE ESSERE CALCOLARE AMBE
    SOLUZIONI DERIVATE DALLA SCELTA DI SEGNI + O - NELLA CINEMATICA INVERSA
    DEL POLSO E VALUTARE QUALE PRODUCA MINIMA VARIAZIONE DEGLI ANGOLI DI GIUNTO
    * IN OGNI CASO NON RIESCO A VEDERE UNA VARIAZIONE CHE RIESCA A RISOLVERE
    EFFETTIVAMENTE IL PROBLEMA, PERTANTO PER QUESTO SCRIPT HO ADOPERATO LA PRIMA
    ALTERNATIVA ESPOSTA
*/

// RISPETTIVAMENTE LE POSIZIONI RELATIVE DELLA PINZA, DELLA BASE E DEL POLSO
float []p6 = new float[3];
float []p0 = new float[3];
float []p3 = new float[3];

// VARIABILI FISSE DEL ROBOT (OMONIME AI PARAMETRI D I ED A I DELLA TABELLA DEI PARAMETRI DI D-H)
float d1 = 30;
float d4 = 150;
float d6 = 170;

float l1 = 20;
float l2 = 200;

// SPESSORE DEI LINK E LUNGHEZZA DEGLI ASSI DI RIFERIMENTO
float dl = 10, ax = 60;

// VARIABILI DELLA POSIZIONE DELLA CAMERA
float eyeX, eyeY, eyeR;

// VARIABILE DELLA CONFIGURAZIONE A BRACCIO ALTO (1) O BASSO (-1)
int elb = 1;

// ANGOLI DELLA PARAMETRIZZAZIONE ZYZ
float alph, beta, thet;

// RISPETTIVAMENTE VARIABILE DI CONTROLLO E PRECISIONE
float kp = .05, eps = .001;

// VARIABILI UTILI A SEMPLIFICARE IL CALCOLO PER LA CINEMATICA INVERSA
float A1, A2, a1, a2, c1, c23, s1, s23, a, t;

// MATRICI DI ROTAZIONE DELLA PINZA RISPETTO AL POLSO E ALLA BASE RISPETTIVAMENTE (LA TRASPOSTA DI R03, R30 È DEFINITA LOCALMENTE IN CALCPARM) 
float [][]R36 = new float[3][3];
float [][]R06 = new float[3][3];

// THETA I
float []q = new float[6];

// THETA I ATTUALE RISPETTO ALLO STATO DELLA LEGGE DI CONTROLLO KP
float []qA = new float[6];

// COLORI DEGLI ASSI X, Y, Z
color colorX = color(255, 0, 0);
color colorY = color(0, 255, 0);
color colorZ = color(0, 0, 255);

// COLORE DELLO SFONDO (100 IN CASO DI IRRAGGIUNGIBILITÀ DELLA PINZA)
color colorB = color(0, 0, 0);

// VETTORE DI STRINGHE USATE PER STAMPARE I VALORI DEGLI ANGOLI DI GIUNTO THETA I
String []strs = new String[6];

void setup() 
{
  size(1200, 500, P3D);
  stroke(0);
  
  p0[0] = width / 2;
  p0[1] = height / 2;
  p0[2] = 0;
  
  eyeR = height;
  
  for(int i = 0; i < 3; i++)
    p6[i] = 0;
  
  for(int i = 0; i < 6; i++)
    strs[i] = "Theta " + nf(i + 1) + " =";
  
  noStroke();
}

void draw()
{
  // SE IL MOUSE È STATO PREMUTO MODIFICA STATO DI ULTIMA MODIFICA
  if(mousePressed){
    p0[0] = mouseX;
    p0[1] = mouseY;
  }
  
  // SE UNA CHIAVE È STATA PREMUTA MODIFICA STATO DI ULTIMA MODIFICA
  if(keyPressed){
    switch(keyCode){
      case UP:
        eyeY += 5;
        break;
      case DOWN:
        eyeY -= 5;
        break;
      case LEFT:
        eyeX += .1;
        break;
      case RIGHT:
        eyeX -= .1;
        break;
      default:    
        marmoreif(key);
    }
  }
  
  background(colorB);
  
  // LA POSIZIONE DELLA CAMERA COINCIDE CON IL PUNTO DI UN CILINDRO DI RAGGIO EYER CON ASSE PARALLELO A X
  camera(width / 2 + eyeR * sin(eyeX), height / 2 - eyeY, eyeR * cos(eyeX), width / 2, height / 2, 0, 0, 1, 0);
  
  if(abs(alph) > TAU)
    alph += (alph > 0 ? -1 : 1) * TAU;
  
  if(abs(beta) > TAU)
    beta += (beta > 0 ? -1 : 1) * TAU;
    
  if(abs(thet) > TAU)
    thet += (thet > 0 ? -1 : 1) * TAU;
    
  // STAMPA I PARAMETRI E DISEGNA IL ROBOT
  printParm();
  drawAntr();
}

// DISEGNA A SCHERMO IL ROBOT
void drawAntr(){
  calcParm();  // CALCOLA I PARAMETRI DELLA CINEMATICA INVERSA
  
  translate(p0[0], p0[1], p0[2]);
  rotateX(PI / 2); // RUOTA IL SISTEMA DI RIFERIMENTO COSÌ CHE COINCIDA CON QUELLO DELLA BASE
  rotateZ(PI / 2);
  pushMatrix();
  
  // APPLICA LA CINEMATICA DIRETTA PER DISEGNARE IL ROBOT
  drawAxes();
  fill(255, 255, 0);
  
  // GIUNTO 0
  translate(0, 0, 1.5 * dl);
  box(dl, dl, 3 * dl);
  translate(0, 0, - 1.5 * dl);
  fill(255, 100);
  
  // GIUNTO 1, LINK 0, LINK 1
  rotateZ(qA[0]);
  translate(0, 0, d1 / 2);
  box(2 * l1, 2 * l1, d1);
  translate(0, 0, d1 / 2);
  
  // GIUNTO 2, LINK 2
  rotateX(PI / 2);
  translate(l1, 0, 0);
  rotateZ(qA[1]);
  fill(255);
  box(dl, dl, 3 * dl);
  fill(255, 100);
  
  // LINK 2
  translate(l2 / 2, 0, 0);
  box(l2, dl, dl);
  translate(l2 / 2, 0, 0);
  
  // GIUNTO 3
  rotateZ(qA[2]);
  rotateX(PI / 2);
  fill(255);
  box(dl, 3 * dl, dl);
  fill(255, 100);
  
  // GIUNTO 4
  rotateZ(qA[3]);
  fill(255);
  box(dl, dl, 3 * dl);
  fill(255, 100);
  
  // LINK 3
  translate(0, 0, d4 / 2);
  box(dl, dl, d4);
  translate(0, 0, d4 / 2);
  
  // GIUNTO 4
  rotateX(- PI / 2);
  fill(255);
  box(dl, dl, 3 * dl);
  fill(255, 100);
  
  // LINK 5, LINK 6
  rotateZ(qA[4]);
  rotateX(PI / 2);
  translate(0, 0, d6 / 2);
  box(dl, dl, d6);
  translate(0, 0, d6 / 2);
  
  // GIUNTO 5
  rotateZ(qA[5]);
  fill(255);
  box(3 * dl, dl, dl);
  fill(255, 100);
  
  // GIUNTO 6
  fill(255, 0, 255);
  box(dl, dl, 3 * dl);
  fill(255, 100);
  
  // DISEGNA ASSI DI RIFERIMENTO DELLA PINZA ATTUALI
  drawAxes();
  
  popMatrix();
  /*
  // DISEGNA ASSI DI RIFERIMENTO DELLA PINZA ASPETTATI
  translate(p6[0], p6[1], p6[2]);
  rotateZ(alph);
  rotateY(PI / 2 - beta);
  rotateZ(thet);
  
  drawAxes();
  */
}

// STAMPA A SCHERMO LA MATRICE DI ROTAZIONE DESIDERATA, LE MISURE DEGLI ANGOLI ALPH, BETA E THET E LA POSIZIONE DESIDERATA
void printParm(){
  mPrint("End-effector's rotation matrix", R06, 0, 0);
  fill(255); 
  
  textSize(30);
  text("End-effector's position", 480, 0);
  fill(colorX);
  text(p6[0], 480, 30);
  fill(colorY);
  text(p6[1], 480, 60);
  fill(colorZ);
  text(p6[2], 480, 90);
  fill(255);
  
  text("Angle measures", 840, 0);
  text("Alpha =", 840, 30);
  text(degrees(alph), 1000, 30);
  text("Beta =", 840, 60);
  text(degrees(beta), 1000, 60);
  text("Theta =", 840, 90);
  text(degrees(thet), 1000, 90);
  
  text("Joints' angles", 1120, 0);
  for(int i = 0; i < 6; i ++){
    text(strs[i], 1120, 30 * (i + 1));
    text(degrees(q[i]), 1240, 30 * (i + 1));
  }
  
  text("Kp = ", 0, height - 30);
  text(kp, 120, height - 30);
}

// CALCOLA LA SOLUZIONE ALLA CINEMATICA INVERSA APPLICANDO LE FORMULE SUI PARAMETRI P6, ALPH, BETA E THET
void calcParm(){
  R06 = mProd(mRotZ(- alph), mProd(mRotY(PI / 2 - beta), mRotZ(- thet)));
  
  p3[0] = p6[0] - d6 * R06[0][2];
  p3[1] = - p6[1] - d6 * R06[1][2];
  p3[2] = p6[2] - d6 * R06[2][2];
  
  t = atan2(p3[1], p3[0]); // SALVA TEMPORANEAMENTE Q0 IN T (NEL CASO IN CUI L'ARGOMENTO DELL'ARCOSENO SIA NON LECITO NON AGGIORNA LA POSA DEL ROBOT)
  
  A1 = p3[0] * cos(t) + p3[1] * sin(t) - l1;
  A2 = d1 - p3[2];
  
  a = (A1 * A1 + A2 * A2 - l2 * l2 - d4 * d4) / (2 * l2 * d4);
  
  if(abs(a) <= 1){
    q[0] = t;
    
    q[2] = asin(a);
    if(elb != 1)
      q[2] = PI - q[2];
    
    a1 = d4 * cos(q[2]);
    a2 = d4 * sin(q[2]) + l2;
    q[1] = atan2(a1 * A1 - a2 * A2, a2 * A1 + a1 * A2);
    
    c1 = cos(q[0]);
    s1 = sin(q[0]);
    c23 = cos(q[1] + q[2]);
    s23 = sin(q[1] + q[2]); 
    float [][]R30 = {{c1 * c23, s1, c1 * s23}, {s1 * c23, - c1, s1 * s23}, {s23, 0, - c23}};
    
    R30 = mTrans(R30);
    
    // CALCOLO LA MATRICE DELLA ROTAZIONE DELLA PINZA RISPETTO AL POLSO
    R36 = mProd(R30, R06);
    
    q[3] = atan2(R36[1][2], R36[0][2]);
    q[4] = atan2(sqrt(pow(R36[1][2], 2) + pow(R36[0][2], 2)), R36[2][2]);
    q[5] = atan2(R36[2][1], - R36[2][0]);
    
    colorB = color(0);
  } else
    colorB = color(100);
  
  for(int i = 0; i < 6; i++)
    if(abs(qA[i] - q[i]) > eps)
      qA[i] += kp * (q[i] - qA[i]);
}

// CONDENSA TUTTE LE ASSOCIAZIONI TRA TASTIERA E PARAMETRI IN UNA FUNZIONE
// IL COSTRUTTO SWITCH VIENE TALVOLTA CHIAMATO IF A CASCATA
// E QUINDI LA FUNZIONE L'HO CHIAMATA MARMOREIF
void marmoreif(char keypressed){
  switch(keypressed){
      case 'x':
        p6[0] -= 1;
        break;
      case 'X':
            p6[0] += 1;
            break;
          case 'y':
            p6[1] -= 1;
            break;
          case 'Y':
            p6[1] += 1;
            break;
          case 'z':
            p6[2] -= 1;
            break;
          case 'Z':
            p6[2] += 1;
            break;
          case 'a':
            alph -= .1;
            break;
          case 'A':
            alph += .1;  
            break;
          case 'b':
            beta -= .1;
            break;
          case 'B':
            beta += .1;
            break;
          case 't':
            thet -= .1;
            break;
          case 'T':
            thet += .1;
            break;
          case 'k':
            kp -= .01;
            if(kp < eps)
              kp += .01;
            break;
          case 'K':
            kp += .01;
            if(kp > .5)
              kp -= .01;
            break;
          case 'q':
            eyeR += 5;
            break;
          case 'Q':
            eyeR -= 5;
            break;
            
          // 'v' IMPLICA 'V' IMPLICA 'r' CHE IMPLICA 'R' (NOTARE LA MANCANZA DI BREAK)
          case 'v':
          case 'V':
            eyeY = eyeX = 0;
            eyeR = height;
          case 'r':
          case 'R':
            p0[0] = width / 2;
            p0[1] = height / 2;
            break;
          case '+':
            elb = 1;
            break;
          case '-':
      elb = -1;
    }
}

// METODI PRESI IN PRESTITO DALLA DISPENSA DI F. MARTINELLI E OPPORTUNAMENTE MODIFICATI 
void mPrint(String s, float[][] M, int x, int y)
{
  textSize(30);
  
  fill(255);
  text(s,x,y);
  
  fill(colorX);
  text(M[0][0],x,y+30);
  text(M[1][0],x,y+60);
  text(M[2][0],x,y+90);
  
  fill(colorY);
  text(M[0][1],x+120,y+30);
  text(M[1][1],x+120,y+60);
  text(M[2][1],x+120,y+90);
  
  fill(colorZ);
  text(M[0][2],x+240,y+30);
  text(M[1][2],x+240,y+60); 
  text(M[2][2],x+240,y+90);
}

float[][] mProd(float[][] A,float[][] B)
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

float[][] mTrans(float[][] A)
{
  int nR = A.length;
  int nC = A[0].length; 
  
  float[][] C = new float[nC][nR]; 

  for (int i=0; i < nC; i++) 
    for (int j=0; j < nR; j++) 
      C[i][j] = A[j][i];
      
  return C;
}

// RESTITUISCONO UNA MATRICE 3 X 3 DI ROTAZIONE ATTORNO L'ASSE Z E Y RISPETTIVAMENTE

float[][] mRotZ(float ang){  
  float [][]R = {{cos(ang), -sin(ang), 0}, {sin(ang), cos(ang), 0}, {0, 0, 1}};
  return(R);
}

float[][] mRotY(float ang){  
  float [][]R = {{cos(ang), 0, sin(ang)}, {0, 1, 0}, {-sin(ang), 0, cos(ang)}};
  return(R);
}

// DISEGNA I 3 ASSI [X, Y, Z] DI RIFERIMENTO
void drawAxes(){
  fill(colorX);
  translate(ax / 2 + dl / 2, 0, 0);
  box(ax, dl / 2, dl / 2);
  fill(colorY);
  translate(- ax / 2 - dl / 2, - ax / 2 - dl / 2, 0);
  box(dl / 2, ax, dl / 2);
  fill(colorZ);
  translate(0, ax / 2 + dl / 2, ax / 2 + dl / 2);
  box(dl / 2, dl / 2, ax);
  translate(0, 0, - ax / 2 - dl / 2);
}
