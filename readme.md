# Robot Manipolatore Planare 4R con Gravità

Progetto di modellazione cinematica, dinamica, controllo e simulazione in MATLAB di un robot manipolatore planare a 4 gradi di libertà (4R) soggetto a gravità lungo l'asse $-y$.

## Descrizione del Progetto

Il sistema modellato è un manipolatore planare composto da 4 link rigidi collegati da 4 giunti rotoidali i cui assi di rotazione sono paralleli tra loro e ortogonali al piano di lavoro $XY$. Il progetto copre l'intera pipeline di sviluppo:

1. **Cinematica Diretta:** Assegnazione delle terne secondo la convenzione Denavit-Hartenberg (DH) standard, calcolo delle matrici di trasformazione omogenea $A_i$ e della trasformazione complessiva $T = A_1 A_2 A_3 A_4$.
2. **Cinematica Inversa:** Disaccoppiamento tramite equazione del polso, risoluzione analitica in forma chiusa per il sottosistema planare e gestione delle soluzioni multiple (configurazione gomito alto / gomito basso).
3. **Modellazione Dinamica:** Derivazione delle equazioni del moto tramite approccio energetico di Eulero-Lagrange ($M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$), sia in forma simbolica che numerica.
4. **Progetto del Controllore:** Implementazione di un controllore a coppia calcolata (*Computed Torque Control*) nello spazio dei giunti per l'inseguimento di traiettorie circolari nel piano $XY$ e traiettorie polinomiali cubiche.
5. **Simulazione e Animazione:** Integrazione numerica della dinamica a ciclo chiuso tramite `ode45`, generazione dei grafici prestazionali e animazione in tempo reale del manipolatore con visualizzazione delle terne DH e della traiettoria target.

---

## Struttura del Repository

* `movimentazione_braccio.m` — Script principale che esegue i test cinematici, l'integrazione dinamica tramite `ode45`, la generazione dei grafici e l'animazione del robot.
* `fk_4R.m` — Funzione per il calcolo della cinematica diretta.
* `ifun_4R.m` — Funzione per il calcolo della cinematica inversa analitica.
* `dynamics_4R.m` — Funzione per il calcolo numerico delle matrici dinamiche $M(q)$, $C(q,\dot{q})$ e del vettore gravitazionale $G(q)$.

---

## Documentazione delle Funzioni

### 1. Cinematica Diretta (`fk_4R.m`)
```matlab
[T, x, y, phi] = fk_4R(theta, L)
```
Calcola la posa dell'organo terminale (End-Effector) a partire dalla configurazione dei giunti.
* **Input:**
  * `theta`: vettore $1 \times 4$ degli angoli di giunto $[\theta_1, \theta_2, \theta_3, \theta_4]$ in radianti.
  * `L`: vettore $1 \times 4$ delle lunghezze dei link $[L_1, L_2, L_3, L_4]$ in metri.
* **Output:**
  * `T`: matrice di trasformazione omogenea $4 \times 4$ dalla terna base $\{0\}$ alla terna $\{4\}$.
  * `x, y`: coordinate cartesiane dell'End-Effector nel piano $XY$.
  * `phi`: orientamento complessivo dell'End-Effector ($\phi = \theta_1 + \theta_2 + \theta_3 + \theta_4$).

### 2. Cinematica Inversa (`ifun_4R.m`)
```matlab
theta = ifun_4R(x, y, phi, L, elbow)
```
Calcola gli angoli di giunto necessari per raggiungere una posa desiderata $(x, y, \phi)$. Risolve la ridondanza cinematica fissando $\theta_3 = 0$ e disaccoppiando la posizione del polso.
* **Input:**
  * `x, y`: coordinate desiderate dell'End-Effector in metri.
  * `phi`: orientamento desiderato in radianti.
  * `L`: vettore delle lunghezze dei link.
  * `elbow`: parametro opzionale per la scelta della configurazione (`1` per gomito basso, `-1` per gomito alto).
* **Output:**
  * `theta`: vettore $1 \times 4$ degli angoli di giunto calcolati in radianti.

### 3. Dinamica (`dynamics_4R.m`)
```matlab
[M, C, G] = dynamics_4R(theta, dtheta, params)
```
Calcola i termini del modello dinamico di Eulero-Lagrange utilizzando gli Jacobiani dei baricentri e i simboli di Christoffel.
* **Input:**
  * `theta`: vettore delle posizioni dei giunti.
  * `dtheta`: vettore delle velocità angolari dei giunti.
  * `params`: struttura contenente i parametri fisici del robot (`L`, `lc`, `m`, `I`, `g`).
* **Output:**
  * `M`: matrice di inerzia $4 \times 4$ simmetrica e definita positiva.
  * `C`: matrice $4 \times 4$ dei termini centrifughi e di Coriolis.
  * `G`: vettore $4 \times 1$ delle coppie gravitazionali.

---

## Funzionalità della Simulazione

All'interno dello script principale `movimentazione_braccio.m` sono implementate le seguenti funzionalità:

* **Controllo Computed Torque:** Compensazione preventiva dei termini non lineari ($M, C, G$) abbinata a una retroazione PD sui giunti per garantire errore di inseguimento nullo a regime.
* **Pianificazione di Traiettoria:** Supporto sia per traiettorie circolari nello spazio operativo (campionate tramite cinematica inversa) sia per traiettorie polinomiali cubiche nello spazio dei giunti.
* **Analisi Grafica:** Tracciamento automatico dell'andamento temporale delle variabili di giunto $q(t)$, dell'errore di inseguimento $e(t) = q_d - q$ e delle coppie erogate dai motori $\tau(t)$.
* **Animazione 2D:** Rappresentazione grafica dinamica tramite `drawnow` che mostra il movimento dei link, la traiettoria target, la scia dell'End-Effector e le terne di riferimento DH locali su ciascun giunto.

---

## Requisiti ed Esecuzione

### Requisiti
* MATLAB (testato su R2022b e versioni successive)
* Symbolic Math Toolbox

### Come eseguire il codice
1. Clonare il repository o scaricare tutti i file `.m` nella stessa cartella di lavoro.
2. Aprire MATLAB e impostare la cartella del progetto come *Current Folder*.
3. Eseguire lo script principale dalla Command Window:
```matlab
movimentazione_braccio
```
