#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"
#include <stdio.h>
#include <math.h>
#include "pid_ctrl.h"

// Prototipos de funciones
void init_gpio_interrupts(void);
void init_adc(void);
void init_pwm(void);
//void init_spiffs(void);
void leer_archivo_csv(void);
void actualizar_pwm(float duty_1, float duty_2, float duty_l);
double calcular_pid(double K_obj, double K_actual, double duty_1);
double calcular_pid2(double p1, double p2, double duty_2);
double filtrar_voltaje(double voltaje_actual);
double leer_potenciometro_ajustado(adc_channel_t canal);
double leer_presion(adc_channel_t canal);
void mostrar_datos(double w1, double w2, double wl, double p1, double p2, double k_obj, double k_actual, float duty1, float duty2, float dutyl, float error);

// Definir una etiqueta de log para identificar los mensajes en el terminal
static const char *TAG = "ProyectoESP32";

// Pines de sensores de velocidad (efecto Hall)
#define PIN_VELOCIDAD_W1   GPIO_NUM_26
#define PIN_VELOCIDAD_W2   GPIO_NUM_25
#define PIN_VELOCIDAD_WL   GPIO_NUM_33

// Pines de sensores de presión (ADC)
#define PIN_SENSOR_P1      ADC_CHANNEL_7  // GPIO35
#define PIN_SENSOR_P2      ADC_CHANNEL_4  // GPIO32
#define PIN_SENSOR_PL      ADC_CHANNEL_6  // GPIO34

// Canal ADC para el potenciómetro que regula K_obj
#define PIN_POTENCIOMETRO  ADC_CHANNEL_3  // GPIO39 (VN)

// Pines para señales PWM
#define PIN_PWM_VAL1       GPIO_NUM_19  // PWM dinámico controlado por PID
#define PIN_PWM_VAL2       GPIO_NUM_18  // PWM fijo con ciclo de trabajo constante
#define PIN_PWM_VALL       GPIO_NUM_22  // Nuevo PWM con ciclo de trabajo fijo al 50%

#define WINDOW_SIZE 20


// Variables globales para velocidades, presiones y PID
volatile uint64_t last_time_w1 = 0, last_time_w2 = 0, last_time_wl = 0;
volatile double velocidad_actual_w1 = 0, velocidad_actual_w2 = 0, velocidad_actual_wl = 0;
double presion_p1 = 0, presion_p2 = 0, presion_pl = 0;
double K_obj = 0;  // Valor objetivo
double K_actual = 0;  // Valor actual calculado como w2 / w1
float p_obj = 0.82;
double error_anterior = 0;  // Error del PID anterior
float duty_act1 = 0.26;  // Duty cycle inicial (50%)
float duty_act2 = 0.43;  // Duty cycle inicial (0.35%)
float duty_actl = 0.1;  // Duty cycle inicial (50%)

pid_ctrl_block_handle_t pid_k_obj;  // Bloque PID para K_obj
pid_ctrl_block_handle_t pid_presion;  // Bloque PID para presión

static double velocidades_w1[WINDOW_SIZE] = {0};
static double velocidades_w2[WINDOW_SIZE] = {0};
static double velocidades_wl[WINDOW_SIZE] = {0};

// Índices para cada filtro
static int index_w1 = 0, index_w2 = 0, index_wl = 0;


// Umbral para el filtro de histeresis
#define UMBRAL 0.1

// Handle para el ADC OneShot
adc_oneshot_unit_handle_t adc1_handle;


// Función para mapear la lectura del potenciómetro entre 0.4 y 2.5
double mapear_rango(double valor, double min_src, double max_src, double min_dest, double max_dest) {
    return min_dest + (valor - min_src) * (max_dest - min_dest) / (max_src - min_src);
}

double calcular_promedio(const double *array) {
    double suma = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        suma += array[i];
    }
    return suma / WINDOW_SIZE;
}

void mostrar_datos(double w1, double w2, double wl, double p1, double p2, double k_obj, double k_actual, float duty1, float duty2, float dutyl, float error) {
   
    // Imprimir los datos en formato CSV
    printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f\n", w1, w2, wl, p1, p2, k_obj, k_actual, duty1, duty2, dutyl, error);
}


// Función para leer presión desde un canal ADC y convertirla a bares
double leer_presion(adc_channel_t canal) {
    int valor_adc;
    esp_err_t ret = adc_oneshot_read(adc1_handle, canal, &valor_adc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer el ADC para presión: %s", esp_err_to_name(ret));
        return 0.0; 
    }
    double voltaje = (valor_adc * 3.3) / 4095.0;  /
    return voltaje;
}

// Función para leer el valor del potenciómetro ajustado al rango 0.4 - 2.5
double leer_potenciometro_ajustado(adc_channel_t canal) {
    int valor_adc;
    esp_err_t ret = adc_oneshot_read(adc1_handle, canal, &valor_adc);  // Leer el ADC
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer el ADC: %s", esp_err_to_name(ret));
        return 0.0; r
    }
    double voltaje = (valor_adc * 3.3) / 4095.0;  
    return mapear_rango(voltaje, 0.0, 3.3, 0.4, 2.5); 
}

// Función para aplicar el filtro de histeresis
double filtrar_voltaje(double voltaje_actual) {
    static double K_obj_prev = 0;
    if (fabs(voltaje_actual - K_obj_prev) > UMBRAL) {
        K_obj_prev = voltaje_actual;  // Actualizar solo si supera el umbral
    }
    return K_obj_prev;
}

// ISR para sensores de velocidad (efecto Hall)
// ISR para W1
static void IRAM_ATTR isr_handler_w1(void* arg) {
    uint64_t now = esp_timer_get_time();
    if (last_time_w1 != 0) {
        uint64_t elapsed_time = now - last_time_w1;
        double nueva_velocidad = (1.0 / (elapsed_time / 1000000.0)) / 36 * 60; // Calcular velocidad en RPS

        // Almacenar el valor en la ventana deslizante
        velocidades_w1[index_w1] = nueva_velocidad;
        index_w1 = (index_w1 + 1) % WINDOW_SIZE;  // Índice circular

        // Actualizar con el promedio de la ventana
        velocidad_actual_w1 = calcular_promedio(velocidades_w1);
    }
    last_time_w1 = now;
}

// ISR para W2
static void IRAM_ATTR isr_handler_w2(void* arg) {
    uint64_t now = esp_timer_get_time();
    if (last_time_w2 != 0) {
        uint64_t elapsed_time = now - last_time_w2;
        double nueva_velocidad = (1.0 / (elapsed_time / 1000000.0)) / 48 * 60; // Calcular velocidad en RPS

        // Almacenar el valor en la ventana deslizante
        velocidades_w2[index_w2] = nueva_velocidad;
        index_w2 = (index_w2 + 1) % WINDOW_SIZE;  // Índice circular

        // Actualizar con el promedio de la ventana
        velocidad_actual_w2 = calcular_promedio(velocidades_w2);
    }
    last_time_w2 = now;
}

// ISR para WL
static void IRAM_ATTR isr_handler_wl(void* arg) {
    uint64_t now = esp_timer_get_time();
    if (last_time_wl != 0) {
        uint64_t elapsed_time = now - last_time_wl;
        double nueva_velocidad = (1.0 / (elapsed_time / 1000000.0)) / 49 * 60; // Calcular velocidad en RPS

        // Almacenar el valor en la ventana deslizante
        velocidades_wl[index_wl] = nueva_velocidad;
        index_wl = (index_wl + 1) % WINDOW_SIZE;  // Índice circular

        // Actualizar con el promedio de la ventana
        velocidad_actual_wl = calcular_promedio(velocidades_wl);
    }
    last_time_wl = now;
}


// Inicialización del ADC usando el controlador OneShot
void init_adc(void) {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config, &adc1_handle);

    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    adc_oneshot_config_channel(adc1_handle, PIN_SENSOR_P1, &channel_config);
    adc_oneshot_config_channel(adc1_handle, PIN_SENSOR_P2, &channel_config);
    adc_oneshot_config_channel(adc1_handle, PIN_SENSOR_PL, &channel_config);
    adc_oneshot_config_channel(adc1_handle, PIN_POTENCIOMETRO, &channel_config);
}

// Inicialización de GPIO e interrupciones para sensores de velocidad
void init_gpio_interrupts(void) {
    gpio_install_isr_service(0);

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 1,
    };

    // Configuración de W1
    io_conf.pin_bit_mask = (1ULL << PIN_VELOCIDAD_W1);
    gpio_config(&io_conf);
    gpio_isr_handler_add(PIN_VELOCIDAD_W1, isr_handler_w1, NULL);

    // Configuración de W2
    io_conf.pin_bit_mask = (1ULL << PIN_VELOCIDAD_W2);
    gpio_config(&io_conf);
    gpio_isr_handler_add(PIN_VELOCIDAD_W2, isr_handler_w2, NULL);

    // Configuración de Wl
    io_conf.pin_bit_mask = (1ULL << PIN_VELOCIDAD_WL);
    gpio_config(&io_conf);
    gpio_isr_handler_add(PIN_VELOCIDAD_WL, isr_handler_wl, NULL);
}

// Inicialización de PWM para VAL1, VAL2, vall
void init_pwm(void) {
    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer);

    ledc_channel_config_t pwm_channel_val1 = {
        .gpio_num = PIN_PWM_VAL1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = (uint32_t)( duty_act1 * 1023),
        .hpoint = 0
    };
    ledc_channel_config(&pwm_channel_val1);

    ledc_channel_config_t pwm_channel_val2 = {
        .gpio_num = PIN_PWM_VAL2,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .duty = (uint32_t)(duty_act2 * 1023),
        .hpoint = 0
    };
    ledc_channel_config(&pwm_channel_val2);

    // Configuración del canal PWM para VALL
    ledc_channel_config_t pwm_channel_vall = {
        .gpio_num = PIN_PWM_VALL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_TIMER_0,
        .duty = (uint32_t)(duty_actl * 1023),
        .hpoint = 0
    };
    ledc_channel_config(&pwm_channel_vall);
}

// Actualización del duty cycle en VAL1
void actualizar_pwm(float duty_1, float duty_2, float duty_l) {
    uint32_t duty1 = (uint32_t)(duty_1 * 1023);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    uint32_t duty2 = (uint32_t)(duty_2 * 1023);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

    uint32_t dutyl = (uint32_t)(duty_l * 1023);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, dutyl);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

}


// Función principal
void app_main(void) {
    // Inicialización de periféricos
    init_gpio_interrupts();
    init_adc();
    init_pwm();
    int contador_log = 0;
    float vel1=0;
    float vel2= 0;
    
    // Configuración inicial de los bloques PID
    pid_ctrl_config_t config_k_obj = {
        .init_param = {
            .kp = 0.33,
            .ki = 0.004,
            .kd = 4,
            .max_output = 0.35,
            .min_output = 0.1,
            .max_integral = 50,
            .min_integral = 25,
            .cal_type = PID_CAL_TYPE_POSITIONAL
        }
    };
    pid_new_control_block(&config_k_obj, &pid_k_obj);

    pid_ctrl_config_t config_presion = {
        .init_param = {
            .kp = 0.1,
            .ki = 0.002,
            .kd = 1,
            .max_output = 0.5,
            .min_output = 0.35,
            .max_integral = 250,
            .min_integral = 50,
            .cal_type = PID_CAL_TYPE_POSITIONAL
        }
    };
    pid_new_control_block(&config_presion, &pid_presion);

    while (1) {
        // Leer y filtrar el valor objetivo desde el potenciómetro
        K_obj = filtrar_voltaje(leer_potenciometro_ajustado(PIN_POTENCIOMETRO));
        vel1 = velocidad_actual_w1;
        vel2 = velocidad_actual_w2;

        // Calcular la relación actual (K_actual) entre W1 y W2
        K_actual = (vel1 != 0) ? (vel2 / vel1) : 0;

        // Actualizar las presiones medidas
        presion_p1 = leer_presion(PIN_SENSOR_P1);
        presion_p2 = leer_presion(PIN_SENSOR_P2);
        
        float error_k_obj = K_actual - K_obj;
        pid_compute(pid_k_obj, error_k_obj, &duty_act1);
        
        float error_p_obj = presion_p2 - p_obj;
        pid_compute(pid_presion, error_p_obj, &duty_act2);

        duty_actl = 0.8* (fmin(duty_act1, duty_act2));
       
        // Actualizar el PWM dinámico
        actualizar_pwm(duty_act1, duty_act2, duty_actl);

        // Mostrar datos en el log
         // Controlar la frecuencia de los logs
        if (contador_log % 20 == 0) {
            mostrar_datos(vel1, vel2, velocidad_actual_wl, 
                              presion_p1, presion_p2, K_obj, K_actual, duty_act1, duty_act2, duty_actl, error_p_obj);
            
        }
        contador_log++;  // Incrementar el contador de logs

        // Esperar 10 ms antes de la siguiente iteración
        vTaskDelay(pdMS_TO_TICKS(10));

    }
    pid_del_control_block(pid_k_obj);
    pid_del_control_block(pid_presion);

}