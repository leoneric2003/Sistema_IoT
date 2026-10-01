/*
 * ================================================================
 * VF - SISTEMA DE DETECCIÓN DE GAS CON LÓGICA DIFUSA MAMDANI
 *      Y PROCESAMIENTO EN EL BORDE
 *
 * Basado en:
 * V0 - Código original del proyecto
 * V1 - Implementación Mamdani + centroide
 * V2 - Registro experimental detallado
 *
 * VERSIÓN FINAL (VF):
 * Conserva la lógica y los parámetros de V2 utilizados en las pruebas,
 * con nomenclatura final consistente con el experimento de gas.
 *
 * ESTÍMULO EXPERIMENTAL:
 * GAS
 *
 * DOMINIO DE ENTRADA:
 * SensorGas = 0 - 1023
 *
 * DOMINIO DE SALIDA:
 * VentiladorLED = 0 - 500
 *
 * PWM:
 * 0 - 255
 *
 * DATOS REGISTRADOS:
 * Tiempo_ms
 * SensorGas
 * MuBajo
 * MuMedio
 * MuAlto
 * R1
 * R2
 * R3
 * VentiladorLED
 * PWM
 * ================================================================
 */

#include "Arduino_LED_Matrix.h"
#include <WiFiS3.h>


// ================================================================
// CONFIGURACIÓN DE RED Y THINGSPEAK
// ================================================================

char ssid[] = "TU_RED_WIFI";
char pass[] = "TU_CONTRASENA_WIFI";

const char* server = "api.thingspeak.com";
String apiKey = "TU_API_KEY_THINGSPEAK";


// ================================================================
// INSTANCIAS
// ================================================================

ArduinoLEDMatrix matrix;
WiFiClient client;


// ================================================================
// PINES
// ================================================================

const int pinSensor = A0;
const int pinLED = 11;


// ================================================================
// VARIABLE DE SALIDA
// ================================================================

float VentiladorLED = 0.0;


// ================================================================
// TEMPORIZACIÓN
// ================================================================

unsigned long ultimoEnvio = 0;

const unsigned long intervaloEnvio = 30000;


// ================================================================
// FRAMES DE LA MATRIZ LED
// ================================================================

uint8_t frame_X_ALTA[8][12] = {

    {1,0,0,0,0,0,0,0,0,0,0,1},
    {0,1,1,0,0,0,0,0,0,1,1,0},
    {0,0,1,1,0,0,0,0,1,1,0,0},
    {0,0,0,1,1,1,1,1,1,0,0,0},
    {0,0,0,1,1,1,1,1,1,0,0,0},
    {0,0,1,1,0,0,0,0,1,1,0,0},
    {0,1,1,0,0,0,0,0,0,1,1,0},
    {1,0,0,0,0,0,0,0,0,0,0,1}
};


uint8_t frame_X_MEDIA[8][12] = {

    {1,0,0,0,0,0,0,0,0,0,0,0},
    {0,1,0,0,0,0,0,0,0,1,0,0},
    {0,0,1,0,0,0,0,0,1,0,0,0},
    {0,0,0,1,1,0,0,1,1,0,0,0},
    {0,0,0,1,1,0,0,1,1,0,0,0},
    {0,0,1,0,0,0,0,0,1,0,0,0},
    {0,1,0,0,0,0,0,0,0,1,0,0},
    {1,0,0,0,0,0,0,0,0,0,0,0}
};


uint8_t frame_X_BAJA[8][12] = {

    {1,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,1,0,0,0,0,0,0,0,0,0},
    {0,0,0,1,0,0,0,0,1,0,0,0},
    {0,0,0,0,1,0,0,1,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,1}
};


// ================================================================
// FUNCIONES DE PERTENENCIA - ENTRADA
// SensorGas: 0 - 1023
// ================================================================

float membresiaBajo(float valor) {

    if (valor <= 10.0)
        return 1.0;

    if (valor >= 200.0)
        return 0.0;

    return (200.0 - valor) / 190.0;
}


float membresiaMedio(float valor) {

    if (valor <= 150.0 || valor >= 350.0)
        return 0.0;

    if (valor <= 250.0)
        return (valor - 150.0) / 100.0;

    return (350.0 - valor) / 100.0;
}


float membresiaAlto(float valor) {

    if (valor <= 300.0)
        return 0.0;

    if (valor >= 450.0)
        return 1.0;

    return (valor - 300.0) / 150.0;
}


// ================================================================
// FUNCIONES DE PERTENENCIA - SALIDA
// VentiladorLED: 0 - 500
// ================================================================

float salidaBajo(float valor) {

    if (valor <= 10.0)
        return 1.0;

    if (valor >= 200.0)
        return 0.0;

    return (200.0 - valor) / 190.0;
}


float salidaMedio(float valor) {

    if (valor <= 150.0 || valor >= 350.0)
        return 0.0;

    if (valor <= 250.0)
        return (valor - 150.0) / 100.0;

    return (350.0 - valor) / 100.0;
}


float salidaAlto(float valor) {

    if (valor <= 300.0)
        return 0.0;

    if (valor >= 450.0)
        return 1.0;

    return (valor - 300.0) / 150.0;
}


// ================================================================
// INFERENCIA MAMDANI
//
// R1: SI SensorGas es Bajo -> VentiladorLED Bajo
// R2: SI SensorGas es Medio -> VentiladorLED Medio
// R3: SI SensorGas es Alto -> VentiladorLED Alto
//
// Implicación: mínimo
// Agregación: máximo
// Desfusificación: centroide
// ================================================================

float calcularVentiladorLED(

    float SensorGas,

    float &muBajo,
    float &muMedio,
    float &muAlto,

    float &R1,
    float &R2,
    float &R3

) {


    // ============================================================
    // 1. FUSIFICACIÓN
    // ============================================================

    muBajo = membresiaBajo(SensorGas);
    muMedio = membresiaMedio(SensorGas);
    muAlto = membresiaAlto(SensorGas);


    // ============================================================
    // 2. EVALUACIÓN DE REGLAS
    // ============================================================

    R1 = muBajo;
    R2 = muMedio;
    R3 = muAlto;


    // ============================================================
    // 3. AGREGACIÓN + CENTROIDE
    // ============================================================

    float sumaNumerador = 0.0;
    float sumaDenominador = 0.0;


    for (int z = 0; z <= 500; z++) {

        float salidaB = salidaBajo(z);
        float salidaM = salidaMedio(z);
        float salidaA = salidaAlto(z);


        // --------------------------------------------------------
        // Implicación Mamdani
        // --------------------------------------------------------

        float implicacionBajo =
            min(R1, salidaB);

        float implicacionMedio =
            min(R2, salidaM);

        float implicacionAlto =
            min(R3, salidaA);


        // --------------------------------------------------------
        // Agregación
        // --------------------------------------------------------

        float agregado = max(

            implicacionBajo,

            max(
                implicacionMedio,
                implicacionAlto
            )
        );


        // --------------------------------------------------------
        // Centroide
        // --------------------------------------------------------

        sumaNumerador += z * agregado;

        sumaDenominador += agregado;
    }


    // ============================================================
    // 4. CONTROL DE SEGURIDAD
    // ============================================================

    if (sumaDenominador == 0.0)
        return 0.0;


    // ============================================================
    // 5. RESULTADO DEFUSIFICADO
    // ============================================================

    return sumaNumerador / sumaDenominador;
}


// ================================================================
// CONVERSIÓN A PWM
//
// VentiladorLED: 0 - 500
// PWM: 0 - 255
// ================================================================

int convertirAPWM(float valorDifuso) {

    valorDifuso =
        constrain(valorDifuso, 0.0, 500.0);

    return (int)round(
        (valorDifuso / 500.0) * 255.0
    );
}


// ================================================================
// CONTROL DE MATRIZ LED
// ================================================================

void mostrarMatrizDifusa(float intensidad) {

    if (intensidad < 167.0) {

        matrix.renderBitmap(
            frame_X_BAJA,
            8,
            12
        );

    }

    else if (intensidad < 333.0) {

        matrix.renderBitmap(
            frame_X_MEDIA,
            8,
            12
        );

    }

    else {

        matrix.renderBitmap(
            frame_X_ALTA,
            8,
            12
        );
    }
}


// ================================================================
// REGISTRO EXPERIMENTAL
// ================================================================
//
// Se genera una línea CSV para facilitar la captura de datos.
//
// Formato:
//
// Tiempo_ms,
// SensorGas,
// MuBajo,
// MuMedio,
// MuAlto,
// R1,
// R2,
// R3,
// VentiladorLED,
// PWM
// ================================================================

void registrarDatos(

    unsigned long tiempo,

    int SensorGas,

    float muBajo,
    float muMedio,
    float muAlto,

    float R1,
    float R2,
    float R3,

    float VentiladorLED,

    int PWM

) {

    Serial.print(tiempo);
    Serial.print(",");

    Serial.print(SensorGas);
    Serial.print(",");

    Serial.print(muBajo, 4);
    Serial.print(",");

    Serial.print(muMedio, 4);
    Serial.print(",");

    Serial.print(muAlto, 4);
    Serial.print(",");

    Serial.print(R1, 4);
    Serial.print(",");

    Serial.print(R2, 4);
    Serial.print(",");

    Serial.print(R3, 4);
    Serial.print(",");

    Serial.print(VentiladorLED, 2);
    Serial.print(",");

    Serial.println(PWM);
}


// ================================================================
// SETUP
// ================================================================

void setup() {

    Serial.begin(9600);

    pinMode(pinLED, OUTPUT);

    matrix.begin();


    // ------------------------------------------------------------
    // ENCABEZADO DEL REGISTRO
    // ------------------------------------------------------------

    Serial.println();
    Serial.println(
        "Tiempo_ms,SensorGas,MuBajo,MuMedio,MuAlto,R1,R2,R3,VentiladorLED,PWM"
    );

    Serial.println(
        "Inicio de VF - Registro experimental"
    );


    // ------------------------------------------------------------
    // CONEXIÓN WiFi
    // ------------------------------------------------------------

    WiFi.begin(ssid, pass);

    while (WiFi.status() != WL_CONNECTED) {

        delay(500);

        Serial.print(".");
    }

    Serial.println();

    Serial.println(
        "Sistema Online y Conectado"
    );
}


// ================================================================
// LOOP PRINCIPAL
// ================================================================

void loop() {


    // ============================================================
    // 1. ADQUISICIÓN DEL SENSOR
    // ============================================================

    int SensorGas =
        analogRead(pinSensor);


    SensorGas =
        constrain(
            SensorGas,
            0,
            1023
        );


    // ============================================================
    // 2. VARIABLES DE INFERENCIA
    // ============================================================

    float muBajo = 0.0;
    float muMedio = 0.0;
    float muAlto = 0.0;

    float R1 = 0.0;
    float R2 = 0.0;
    float R3 = 0.0;


    // ============================================================
    // 3. INFERENCIA MAMDANI
    // ============================================================

    VentiladorLED =
        calcularVentiladorLED(

            SensorGas,

            muBajo,
            muMedio,
            muAlto,

            R1,
            R2,
            R3
        );


    // ============================================================
    // 4. CONVERSIÓN A PWM
    // ============================================================

    int PWM =
        convertirAPWM(
            VentiladorLED
        );


    // ============================================================
    // 5. ACTUACIÓN
    // ============================================================

    analogWrite(
        pinLED,
        PWM
    );


    mostrarMatrizDifusa(
        VentiladorLED
    );


    // ============================================================
    // 6. REGISTRO DE DATOS
    // ============================================================

    registrarDatos(

        millis(),

        SensorGas,

        muBajo,
        muMedio,
        muAlto,

        R1,
        R2,
        R3,

        VentiladorLED,

        PWM
    );


    // ============================================================
    // 7. TELEMETRÍA IoT
    // ============================================================

    if (
        millis() - ultimoEnvio >
        intervaloEnvio
    ) {

        enviarAThingSpeak(

            SensorGas,

            (int)VentiladorLED
        );

        ultimoEnvio =
            millis();
    }


    // ============================================================
    // 8. PERIODO DE MUESTREO
    // ============================================================

    delay(250);
}


// ================================================================
// THINGSPEAK
// ================================================================

void enviarAThingSpeak(

    int valorSensor,

    int intensidad

) {

    if (client.connect(server, 80)) {

        String url =
            "/update?api_key=" + apiKey +
            "&field1=" + String(valorSensor) +
            "&field2=" + String(intensidad);


        client.print(

            String("GET ") +
            url +
            " HTTP/1.1\r\n" +

            "Host: " +
            server +
            "\r\n" +

            "Connection: close\r\n\r\n"
        );


        Serial.println(
            "Datos sincronizados con la nube."
        );


        client.stop();

    }

    else {

        Serial.println(
            "Fallo de conexión al servidor IoT"
        );
    }
}