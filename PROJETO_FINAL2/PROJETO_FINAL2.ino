#include <Arduino.h>               // Inclui a biblioteca principal para programação Arduino
#include <AccelStepper.h>          // Inclui a biblioteca para controle de motores de passo
#include <BluetoothSerial.h>       // Inclui a biblioteca para comunicação serial via Bluetooth

String dispositivo = "32P";        // Define o nome do dispositivo Bluetooth como "32P"
BluetoothSerial SerialBT;          // Cria um objeto para comunicação serial Bluetooth

// Definição dos pinos do motor de passo
#define e1 21                      // Define o pino 21 como a primeira bobina do motor de passo
#define e2 19                      // Define o pino 19 como a segunda bobina do motor de passo
#define e3 18                      // Define o pino 18 como a terceira bobina do motor de passo
#define e4 5                       // Define o pino 5 como a quarta bobina do motor de passo

// Configurações do buzzer
#define BUZZER_PIN 25              // Define o pino 25 para o buzzer
#define BUZZER_CHANNEL 0           // Define o canal PWM 0 para o buzzer
#define BUZZER_FREQUENCY 2000      // Define a frequência do buzzer em 2000 Hz
#define BUZZER_RESOLUTION 8        // Define a resolução PWM do buzzer em 8 bits (0 a 255)
int buzzerVolumeVermelho = 128;    // Volume do buzzer (0 a 255) para o alerta vermelho
int buzzerVolumeAmarelo = 64;      // Volume do buzzer (0 a 255) para o alerta amarelo
int buzzerFreqVermelho = 2000;     // Frequência do buzzer em 2000 Hz para o alerta vermelho
int buzzerFreqAmarelo = 1000;      // Frequência do buzzer em 1000 Hz para o alerta amarelo
unsigned long tempoAnteriorBuzzer = 0; // Armazena o tempo anterior para controle do buzzer
const unsigned long intervaloBuzzer = 10; // Intervalo de tempo para atualizar o buzzer em milissegundos

// Definição dos pinos do sensor ultrassônico
#define TRIG_PIN 26                // Define o pino 26 como TRIG para o sensor ultrassônico
#define ECHO_PIN 27                // Define o pino 27 como ECHO para o sensor ultrassônico

// Definição do pino do botão
#define BUTTON_PIN 4               // Define o pino 4 como o pino do botão

// Definição dos pinos dos LEDs
#define LED_VERDE 23               // Define o pino 23 para o LED verde
#define LED_AMARELO 32             // Define o pino 32 para o LED amarelo
#define LED_VERMELHO 33            // Define o pino 33 para o LED vermelho

// Variáveis para controle do efeito de fade dos LEDs
int brilhoVermelho = 0;           // Nível de brilho atual do LED vermelho (0 a 255)
int incrementoVermelho = 30;      // Incremento do brilho do LED vermelho para efeito de fade

int brilhoAmarelo = 0;            // Nível de brilho atual do LED amarelo (0 a 255)
int incrementoAmarelo = 3;        // Incremento do brilho do LED amarelo para efeito de fade

unsigned long tempoAnteriorVermelho = 0; // Armazena o tempo anterior para controle do fade do LED vermelho
unsigned long tempoAnteriorAmarelo = 0;  // Armazena o tempo anterior para controle do fade do LED amarelo
const unsigned long intervaloPWMVermelho = 15; // Intervalo de tempo para o efeito de fade do LED vermelho em milissegundos
const unsigned long intervaloPWMAmarelo = 20;  // Intervalo de tempo para o efeito de fade do LED amarelo em milissegundos

bool alertaVermelho = false;      // Flag para saber se o alerta vermelho está ativo
bool alertaAmarelo = false;       // Flag para saber se o alerta amarelo está ativo

// Criação do objeto AccelStepper para controle do motor de passo
AccelStepper motorPasso(AccelStepper::FULL4WIRE, e1, e3, e2, e4); // Configura motor em modo FULL4WIRE

// Variáveis globais para o sensor ultrassônico e controle do motor
unsigned long tempoAnteriorSensor = 0;  // Armazena o tempo anterior para controle do sensor ultrassônico
const int passosTotais = 1024;          // Define o número total de passos que o motor deve dar em cada direção
bool sentidoHorario = true;             // Define o sentido de rotação inicial do motor (horário)

// Variáveis para controle do botão
bool sistemaLigado = false;             // Estado inicial do sistema (desligado)
bool estadoBotaoAnterior = LOW;         // Armazena o estado anterior do botão (pressionado ou não)
unsigned long tempoDebounce = 50;       // Tempo de debounce do botão em milissegundos (para evitar múltiplas leituras)
unsigned long ultimoTempoBotao = 0;     // Armazena o último tempo em que o botão foi pressionado

// Variáveis para controle do buzzer (LED vermelho)
bool buzzerAtivoVermelho = false;       // Flag para saber se o buzzer está ativo para o LED vermelho
unsigned long tempoBuzzerVermelho = 0;  // Armazena o tempo anterior para o buzzer vermelho

// Variáveis para controle do buzzer (LED amarelo)
bool buzzerAtivo = false;               // Flag para saber se o buzzer está ativo para o LED amarelo
unsigned long tempoBuzzer = 0;          // Armazena o tempo anterior para o buzzer amarelo

// Variáveis adicionais para controlar o retorno do motor à posição inicial
bool sistemaLigadoAnterior = false;     // Armazena o estado anterior do sistema (ligado/desligado)
const long initialPosition = 0;         // Define a posição inicial desejada para o motor de passo

// Função para medir a distância usando o sensor ultrassônico
long medirDistancia() {
    digitalWrite(TRIG_PIN, LOW);   // Garante que o pino TRIG esteja em estado baixo para começar
    delayMicroseconds(2);          // Espera por 2 microssegundos para garantir um pulso limpo
    digitalWrite(TRIG_PIN, HIGH);  // Define o pino TRIG como alto para enviar o pulso ultrassônico
    delayMicroseconds(10);         // Mantém o TRIG alto por 10 microssegundos para enviar o pulso
    digitalWrite(TRIG_PIN, LOW);   // Define o TRIG como baixo novamente após enviar o pulso

    // Usa a função pulseIn para medir o tempo (em microssegundos) que o pulso leva para retornar
    long duracao = pulseIn(ECHO_PIN, HIGH);
    
    // Converte o tempo medido em distância em centímetros
    // (velocidade do som é aproximadamente 0.034 cm/µs, e divide por 2 porque o som vai e volta)
    long distancia = (duracao * 0.034) / 2;

    return distancia; // Retorna a distância calculada
}

// Função para ler o sensor ultrassônico e imprimir a distância via Bluetooth
void lerSensor() {
    unsigned long tempoAtual = millis(); // Obtém o tempo atual em milissegundos desde o início do programa
    
    // Verifica se 1 segundo (1000 ms) se passou desde a última leitura do sensor
    if (tempoAtual - tempoAnteriorSensor >= 1000) {
        tempoAnteriorSensor = tempoAtual; // Atualiza o tempo anterior com o tempo atual
        
        // Chama a função medirDistancia() e armazena o valor da distância
        long distancia = medirDistancia();
        
        // Envia a distância medida via Bluetooth
        SerialBT.print("Distância: ");
        SerialBT.print(distancia);
        SerialBT.println(" cm");
    
        // Lógica de controle para LEDs e alertas com base na distância medida

        // Se a distância for menor que 10 cm
        if (distancia < 10) {
            alertaVermelho = true;       // Ativa o alerta vermelho
            alertaAmarelo = false;       // Desativa o alerta amarelo
            digitalWrite(LED_VERDE, LOW); // Desliga o LED verde
        }
        // Se a distância estiver entre 10 cm e 30 cm
        else if (10 <= distancia && distancia <= 30) {
            alertaVermelho = false;      // Desativa o alerta vermelho
            alertaAmarelo = true;        // Ativa o alerta amarelo
            digitalWrite(LED_VERDE, LOW); // Desliga o LED verde
        }
        // Se a distância for maior que 30 cm
        else {
            alertaVermelho = false;      // Desativa o alerta vermelho
            alertaAmarelo = false;       // Desativa o alerta amarelo
            digitalWrite(LED_VERDE, HIGH); // Liga o LED verde
        }
    }
}


// Função responsável por controlar o efeito de fade no LED vermelho
void fadeLedVermelho() {
    unsigned long tempoAtual = millis(); // Obtém o tempo atual em milissegundos
    
    // Se o alerta vermelho estiver ativo
    if (alertaVermelho == true) {
        digitalWrite(LED_AMARELO, LOW); // Garante que o LED amarelo esteja apagado
    }

    // Verifica se o alerta vermelho está ativo e se já passou o intervalo necessário para atualizar o LED
    if (alertaVermelho && tempoAtual - tempoAnteriorVermelho >= intervaloPWMVermelho) {
        tempoAnteriorVermelho = tempoAtual; // Atualiza o tempo anterior com o tempo atual
        
        // Ajusta o brilho do LED vermelho usando PWM (0-255)
        analogWrite(LED_VERMELHO, brilhoVermelho);
        
        // Incrementa o brilho do LED vermelho
        brilhoVermelho += incrementoVermelho;

        // Inverte a direção do fade (clarear ou escurecer) ao atingir os limites
        if (brilhoVermelho <= 0 || brilhoVermelho >= 255) {
            incrementoVermelho = -incrementoVermelho; // Muda o sentido do incremento
        }

        // Aciona o buzzer por 50ms quando o brilho atingir o máximo (255)
        if (brilhoVermelho >= 255 && !buzzerAtivoVermelho) {
            digitalWrite(BUZZER_PIN, HIGH); // Liga o buzzer
            buzzerAtivoVermelho = true; // Marca que o buzzer está ativo
            tempoBuzzerVermelho = millis(); // Registra o tempo de início do buzzer
        }
    } 
    // Caso o alerta vermelho esteja desativado
    else if (!alertaVermelho) {
        analogWrite(LED_VERMELHO, 0); // Garante que o LED vermelho esteja apagado
    }

    // Verifica se o buzzer já está ativo há 50ms
    if (buzzerAtivoVermelho && (millis() - tempoBuzzerVermelho >= 50)) {
        digitalWrite(BUZZER_PIN, LOW); // Desliga o buzzer
        buzzerAtivoVermelho = false;   // Reseta a flag do buzzer ativo
    }
}

// Função responsável por controlar o efeito de fade no LED amarelo
void fadeLedAmarelo() {
    unsigned long tempoAtual = millis(); // Obtém o tempo atual em milissegundos
    
    // Verifica se o alerta amarelo está ativo e se já passou o intervalo necessário para atualizar o LED
    if (alertaAmarelo && tempoAtual - tempoAnteriorAmarelo >= intervaloPWMAmarelo) {
        tempoAnteriorAmarelo = tempoAtual; // Atualiza o tempo anterior com o tempo atual
        
        // Ajusta o brilho do LED amarelo usando PWM (0-255)
        analogWrite(LED_AMARELO, brilhoAmarelo);
        
        // Incrementa o brilho do LED amarelo
        brilhoAmarelo += incrementoAmarelo;

        // Inverte a direção do fade (clarear ou escurecer) ao atingir os limites
        if (brilhoAmarelo <= 0 || brilhoAmarelo >= 255) {
            incrementoAmarelo = -incrementoAmarelo; // Muda o sentido do incremento
        }

        // Aciona o buzzer por 50ms quando o brilho atingir o máximo (255)
        if (brilhoAmarelo >= 255 && !buzzerAtivo) {
            digitalWrite(BUZZER_PIN, HIGH); // Liga o buzzer
            buzzerAtivo = true; // Marca que o buzzer está ativo
            tempoBuzzer = millis(); // Registra o tempo de início do buzzer
        }
    } 
    // Caso o alerta amarelo esteja desativado
    else if (!alertaAmarelo) {
        analogWrite(LED_AMARELO, 0); // Garante que o LED amarelo esteja apagado
    }

    // Verifica se o buzzer já está ativo há 50ms
    if (buzzerAtivo && (millis() - tempoBuzzer >= 50)) {
        digitalWrite(BUZZER_PIN, LOW); // Desliga o buzzer
        buzzerAtivo = false; // Reseta a flag do buzzer ativo
    }
}

// Função para configurar o motor de passo para mover 1024 passos em uma direção
void configurarMotor() {
    // Verifica se o sentido atual é horário
    if (sentidoHorario) {
        // Configura o motor para se mover 1024 passos à frente
        motorPasso.moveTo(motorPasso.currentPosition() + passosTotais);
    } else {
        // Configura o motor para se mover 1024 passos para trás
        motorPasso.moveTo(motorPasso.currentPosition() - passosTotais);
    }
    // Inverte o sentido do movimento para a próxima vez que a função for chamada
    sentidoHorario = !sentidoHorario;
}

// Função para atualizar o estado do motor de passo
void atualizarMotor() {
    // Verifica se o motor chegou à posição alvo
    if (motorPasso.distanceToGo() == 0) {
        // Se sim, configura o próximo movimento
        configurarMotor();
    }
}

// Função para ler o estado do botão e alternar o estado do sistema (ligado/desligado)
void lerBotao() {
    // Lê o estado atual do botão (pressionado ou não)
    bool estadoBotaoAtual = digitalRead(BUTTON_PIN);
    unsigned long tempoAtual = millis(); // Obtém o tempo atual em milissegundos

    // Verifica se o botão mudou de estado e se o tempo de debounce foi respeitado
    if (estadoBotaoAtual != estadoBotaoAnterior && (tempoAtual - ultimoTempoBotao) > tempoDebounce) {
        ultimoTempoBotao = tempoAtual; // Atualiza o tempo da última leitura válida do botão

        // Se o botão foi pressionado (estado HIGH)
        if (estadoBotaoAtual == HIGH) {
            // Alterna o estado do sistema entre ligado e desligado
            sistemaLigado = !sistemaLigado;

            // Se o sistema foi ligado
            if (sistemaLigado == true) {
                SerialBT.println("Sistema Ligado!"); // Envia mensagem via Bluetooth
                // Imprime uma linha decorativa no monitor serial Bluetooth
                for (int i = 0; i < 15; i++) {
                    SerialBT.print("-=");
                }
                SerialBT.println(); // Quebra de linha
            } 
            // Se o sistema foi desligado
            else {
                // Imprime uma linha decorativa no monitor serial Bluetooth
                for (int i = 0; i < 15; i++) {
                    SerialBT.print("-=");
                }
                SerialBT.println(); // Quebra de linha
                SerialBT.println("Sistema Desligado!"); // Envia mensagem via Bluetooth
                // Imprime outra linha decorativa
                for (int i = 0; i < 15; i++) {
                    SerialBT.print("-=");
                }
                SerialBT.println(); // Quebra de linha
            }
        }
    }
    // Atualiza o estado anterior do botão para o estado atual
    estadoBotaoAnterior = estadoBotaoAtual;
}

// Função de configuração inicial do Arduino (executada uma vez)
void setup() {
    // Inicia a comunicação serial com uma taxa de transmissão de 115200 bits por segundo
    Serial.begin(115200);

    // Inicia a comunicação Bluetooth com o nome do dispositivo definido na variável "dispositivo"
    SerialBT.begin(dispositivo);

    // Configurações iniciais para o motor de passo:
    // Define a velocidade máxima do motor para 100 passos por segundo
    motorPasso.setMaxSpeed(100.0);
    // Define a aceleração do motor para 100 passos por segundo ao quadrado
    motorPasso.setAcceleration(100.0);

    // Configura os pinos do sensor ultrassônico
    pinMode(TRIG_PIN, OUTPUT); // Define o pino TRIG como saída
    pinMode(ECHO_PIN, INPUT);  // Define o pino ECHO como entrada

    // Configura o pino do botão
    pinMode(BUTTON_PIN, INPUT); // Define o pino do botão como entrada

    // Configura os pinos dos LEDs
    pinMode(LED_VERDE, OUTPUT);    // Define o pino do LED verde como saída
    pinMode(LED_AMARELO, OUTPUT);  // Define o pino do LED amarelo como saída
    pinMode(LED_VERMELHO, OUTPUT); // Define o pino do LED vermelho como saída

    // Configura o pino do buzzer
    pinMode(BUZZER_PIN, OUTPUT); // Define o pino do buzzer como saída

    // Inicializa os LEDs amarelo e vermelho apagados (brilho 0)
    analogWrite(LED_AMARELO, 0);  // Desliga o LED amarelo
    analogWrite(LED_VERMELHO, 0); // Desliga o LED vermelho

    // Imprime uma mensagem no monitor serial para indicar que o sistema foi iniciado
    Serial.println("Sistema Iniciado");

    // Armazena o estado inicial do sistema (desligado) em uma variável
    sistemaLigadoAnterior = sistemaLigado;
}

// Função principal do Arduino que executa repetidamente após a função setup()
void loop() {
    // Lê o estado atual do botão e atualiza a variável de controle do sistema
    lerBotao();  

    // Verifica se o estado do sistema (ligado/desligado) mudou
    if (sistemaLigado != sistemaLigadoAnterior) {
        // Se o sistema foi desligado
        if (!sistemaLigado) {
            // Move o motor para a posição inicial (0)
            motorPasso.moveTo(initialPosition);
        } else {
            // Se o sistema foi ligado, configura o próximo movimento do motor
            configurarMotor();
        }
        // Atualiza a variável que guarda o estado anterior do sistema
        sistemaLigadoAnterior = sistemaLigado;
    }

    // Executa o motor para garantir que ele se mova em direção à posição alvo
    motorPasso.run();

    // Se o sistema está ligado
    if (sistemaLigado) {
        // Atualiza o estado do motor de acordo com o movimento
        atualizarMotor();  
        // Lê o valor do sensor ultrassônico e atualiza o sistema de alerta
        lerSensor();     
        // Controla o efeito de fade no LED vermelho
        fadeLedVermelho();
        // Controla o efeito de fade no LED amarelo
        fadeLedAmarelo();
        
    } else {
        // Se o sistema está desligado, realiza a limpeza dos estados
        // Desliga o buzzer
        digitalWrite(BUZZER_PIN, LOW);
        // Desativa os alertas dos LEDs
        alertaVermelho = false;
        alertaAmarelo = false;
        // Desliga o LED verde
        digitalWrite(LED_VERDE, LOW);
    }
}
