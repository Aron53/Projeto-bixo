// ===============================
// PINOS DOS ENCODERS
// ===============================

#define ENC_ESQ_A 0
#define ENC_ESQ_B 1

#define ENC_DIR_A 2
#define ENC_DIR_B 3


// ===============================
// DADOS FISICOS DO ROBO
// ===============================

const float RAIO_RODA = 0.033;       // metros
const float DISTANCIA_RODAS = 0.16;  // metros
const int TICKS_POR_VOLTA = 330;


// ===============================
// CONTADORES DOS ENCODERS
// ===============================

volatile long ticks_esq = 0;
volatile long ticks_dir = 0;

long ticks_esq_anterior = 0;
long ticks_dir_anterior = 0;


// ===============================
// POSE DO ROBO
// ===============================

float x = 0.0;
float y = 0.0;
float theta = 0.0;


// ===============================
// TEMPO
// ===============================

unsigned long tempo_anterior = 0;

const unsigned long PERIODO = 50; // 50 ms = 20 Hz


// ===============================
// INTERRUPCOES
// ===============================

void IRAM_ATTR encoderEsquerdo() {

    if (digitalRead(ENC_ESQ_B) == HIGH) {
        ticks_esq++;
    } else {
        ticks_esq--;
    }
}


void IRAM_ATTR encoderDireito() {

    if (digitalRead(ENC_DIR_B) == HIGH) {
        ticks_dir++;
    } else {
        ticks_dir--;
    }
}


// ===============================
// SETUP
// ===============================

void setup() {

    Serial.begin(115200);

    pinMode(ENC_ESQ_A, INPUT_PULLUP);
    pinMode(ENC_ESQ_B, INPUT_PULLUP);

    pinMode(ENC_DIR_A, INPUT_PULLUP);
    pinMode(ENC_DIR_B, INPUT_PULLUP);

    attachInterrupt(
        digitalPinToInterrupt(ENC_ESQ_A),
        encoderEsquerdo,
        RISING
    );

    attachInterrupt(
        digitalPinToInterrupt(ENC_DIR_A),
        encoderDireito,
        RISING
    );
}


// ===============================
// LOOP
// ===============================

void loop() {

    unsigned long agora = millis();

    if (agora - tempo_anterior >= PERIODO) {

        tempo_anterior = agora;


        // Faz uma copia segura dos ticks

        noInterrupts();

        long ticks_esq_atual = ticks_esq;
        long ticks_dir_atual = ticks_dir;

        interrupts();


        // Quantos ticks aconteceram desde
        // a ultima atualizacao?

        long delta_ticks_esq =
            ticks_esq_atual - ticks_esq_anterior;

        long delta_ticks_dir =
            ticks_dir_atual - ticks_dir_anterior;


        ticks_esq_anterior = ticks_esq_atual;
        ticks_dir_anterior = ticks_dir_atual;


        // Ticks -> distancia da roda

        float distancia_esq =
            ((float)delta_ticks_esq / TICKS_POR_VOLTA)
            * 2.0 * PI * RAIO_RODA;

        float distancia_dir =
            ((float)delta_ticks_dir / TICKS_POR_VOLTA)
            * 2.0 * PI * RAIO_RODA;


        // Cinematica diferencial

        float delta_s =
            (distancia_dir + distancia_esq) / 2.0;

        float delta_theta =
            (distancia_dir - distancia_esq)
            / DISTANCIA_RODAS;


        // Atualiza a pose

        x += delta_s * cos(theta);
        y += delta_s * sin(theta);

        theta += delta_theta;


        // Saida serial

        Serial.print("Ticks_E:");
        Serial.print(ticks_esq_atual);

        Serial.print("\tTicks_D:");
        Serial.print(ticks_dir_atual);

        Serial.print("\tX:");
        Serial.print(x, 4);

        Serial.print("\tY:");
        Serial.print(y, 4);

        Serial.print("\tTheta:");
        Serial.println(theta, 4);
    }
}