/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Smart House (Boutons + LCD + Capteurs)
  *
  * Fonctionnement :
  *  - ON/OFF : démarre/arrête le système
  *  - Au démarrage : "Bonjour / Smart House" (2s) puis affichage mesures (2s)
  *  - Réglage seuil Température (avec + / -) puis OK
  *  - Réglage seuil Gaz (avec + / -) puis OK
  *  - Mode normal : affiche T, G et le seuil température en haut
  *  - Alarmes : Temp / Gaz / Temp+Gaz avec LEDs, buzzer, ventilateur
  *
  * Particularités ajoutées :
  *  - Anti-rebond NON bloquant (sans HAL_Delay)
  *  - Ajustement du seuil température "à chaud" en ST_NORMAL, ST_TEMP_ALARM, ST_BOTH_ALARM
  *  - Limite du seuil température : 0..60°C
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_lcd.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */

/* ===================== 1) ETATS / LOGIQUE ===================== */

/* Machine d'états du système */
typedef enum {
  ST_OFF = 0,       // Système éteint
  ST_BOOT_MSG,      // Message "Bonjour / Smart House" (2s)
  ST_SHOW_MEAS,     // Affichage mesures (2s)
  ST_SET_TEMP,      // Réglage seuil température
  ST_SET_GAS,       // Réglage seuil gaz
  ST_NORMAL,        // Fonctionnement normal
  ST_TEMP_ALARM,    // Alarme température
  ST_GAS_ALARM,     // Alarme gaz
  ST_BOTH_ALARM     // Alarme combinée temp + gaz
} AppState;

/* Etat courant et précédent (utile si plus tard tu veux envoyer des events UART, log, etc.) */
static AppState state      = ST_OFF;
static AppState prev_state = ST_OFF;

/* drapeau ON/OFF global */
static uint8_t system_on = 0;

/* ===================== 2) BOUTONS ===================== */
/*
   Boutons physiques (Pull-up activé) :
   - Niveau = 1 : non appuyé
   - Niveau = 0 : appuyé
*/
static uint8_t last_btn   = 1;  // dernier niveau ON/OFF
static uint8_t last_plus  = 1;  // dernier niveau +
static uint8_t last_minus = 1;  // dernier niveau -
static uint8_t last_ok    = 1;  // dernier niveau OK

/* Anti-rebond non bloquant : "prochaine date autorisée" pour accepter un nouvel appui */
static uint32_t next_on_ms    = 0;
static uint32_t next_plus_ms  = 0;
static uint32_t next_minus_ms = 0;
static uint32_t next_ok_ms    = 0;

/* Durées anti-rebond (ms) */
#define DEB_ON_MS     250u
#define DEB_PLUS_MS   120u
#define DEB_MINUS_MS  120u
#define DEB_OK_MS     150u

/* ===================== 3) SEUILS / MESURES ===================== */

/* Seuils */
static int16_t  temp_seuil = 32;   // seuil température (°C) modifiable
static uint16_t gaz_seuil  = 150;  // seuil gaz (ADC) modifiable (à calibrer)

/* Limites du seuil température */
#define TEMP_SEUIL_MIN  0
#define TEMP_SEUIL_MAX  60

/* Mesures capteurs */
static float    tempC        = 0.0f;  // température en °C (float)
static int16_t  temp_mesuree = 0;     // température arrondie (int)
static uint16_t gaz_adc      = 0;     // valeur ADC MQ-2
static uint8_t  gaz_detecte  = 0;     // 1 si gaz détecté, 0 sinon (avec hystérésis)

/* Hystérésis température :
   - pour éviter oscillations autour du seuil */
#define TEMP_HYST_ON   0.2f
#define TEMP_HYST_OFF  0.2f

/* Hystérésis gaz autour du seuil ADC */
#define GAS_HYST       30u

/* ===================== 4) TIMING ===================== */
static uint32_t last_sense_ms = 0;  // dernière lecture capteurs
static uint32_t state_t0_ms   = 0;  // début d’un état (pour les 2s)

/* ===================== 5) BUZZER ===================== */
/* Buzzer non-bloquant : on l’allume et on planifie une extinction */
static uint8_t  buzzer_done      = 0;  // évite de relancer le beep plusieurs fois
static uint8_t  buzzer_active    = 0;  // buzzer actuellement ON ?
static uint32_t buzzer_until_ms  = 0;  // date d’extinction buzzer

/* ===================== 6) LCD ANTI-FLICKER ===================== */
/* Cache des deux lignes (16 chars) pour éviter de ré-écrire si c’est identique */
static char lcd_l0_last[17] = {0};
static char lcd_l1_last[17] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN 0 */

/* ============================================================
   A) BOUTONS : Détection d’un appui (front descendant)
   + Anti-rebond non bloquant via HAL_GetTick()
   ------------------------------------------------------------
   last     : dernier niveau lu (0/1)
   now_lvl  : niveau actuel lu (0/1)
   next_ms  : prochaine date (ms) où un appui sera accepté
   deb_ms   : durée anti-rebond
   Retour   : 1 si nouvel appui détecté, 0 sinon
   ============================================================ */
static uint8_t Btn_FallingEdge(uint8_t *last, uint8_t now_lvl,
                               uint32_t *next_ms, uint32_t deb_ms)
{
  uint32_t now = HAL_GetTick();
  uint8_t pressed = 0;

  if (now >= *next_ms) {
    if ((*last == 1u) && (now_lvl == 0u)) {
      pressed = 1u;
      *next_ms = now + deb_ms; // bloque les nouveaux appuis pendant deb_ms
    }
  }

  *last = now_lvl;
  return pressed;
}

/* ============================================================
   B) LCD : affichage sans flicker
   - on compare avec cache (lcd_l0_last / lcd_l1_last)
   - si même texte => on n'écrit pas
   ============================================================ */
static void LCD_PrintLine(uint8_t row, const char *s16)
{
  char buf[17];
  snprintf(buf, sizeof(buf), "%-16s", s16); // pad à 16 chars

  if (row == 0) {
    if (strncmp(buf, lcd_l0_last, 16) == 0) return;
    memcpy(lcd_l0_last, buf, 16); lcd_l0_last[16] = 0;
  } else {
    if (strncmp(buf, lcd_l1_last, 16) == 0) return;
    memcpy(lcd_l1_last, buf, 16); lcd_l1_last[16] = 0;
  }

  LCD_SetCursor(row, 0);
  LCD_Print(buf);
}

static void LCD_ClearCache(void)
{
  memset(lcd_l0_last, 0, sizeof(lcd_l0_last));
  memset(lcd_l1_last, 0, sizeof(lcd_l1_last));
  LCD_Clear();
}

/* ============================================================
   C) BUZZER : beep non bloquant
   ============================================================ */
static void Buzzer_Beep(uint32_t ms)
{
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
  buzzer_active   = 1;
  buzzer_until_ms = HAL_GetTick() + ms;
}

static void Buzzer_Task(void)
{
  if (buzzer_active && (HAL_GetTick() >= buzzer_until_ms)) {
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
    buzzer_active = 0;
  }
}

/* ============================================================
   D) CAPTEUR HTS221 : lecture température (I2C2)
   ============================================================ */
#define HTS221_ADDR_7BIT   (0x5F)
#define HTS221_ADDR_8BIT   (HTS221_ADDR_7BIT << 1)

static uint8_t hts221_read8(I2C_HandleTypeDef *hi2c, uint8_t reg)
{
  uint8_t v = 0;
  (void)HAL_I2C_Mem_Read(hi2c, HTS221_ADDR_8BIT, reg, I2C_MEMADD_SIZE_8BIT, &v, 1, 100);
  return v;
}

static void hts221_write8(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
  (void)HAL_I2C_Mem_Write(hi2c, HTS221_ADDR_8BIT, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

static void hts221_power_on(I2C_HandleTypeDef *hi2c)
{
  hts221_write8(hi2c, 0x20, 0x81); // CTRL_REG1: PD=1, ODR=1Hz
}

static float hts221_get_tempC(I2C_HandleTypeDef *hi2c)
{
  /* Calibration T0/T1 (datasheet) */
  uint8_t t0_x8  = hts221_read8(hi2c, 0x32);
  uint8_t t1_x8  = hts221_read8(hi2c, 0x33);
  uint8_t msb    = hts221_read8(hi2c, 0x35);

  uint16_t T0_x8 = (uint16_t)(t0_x8 | ((msb & 0x03) << 8));
  uint16_t T1_x8 = (uint16_t)(t1_x8 | ((msb & 0x0C) << 6));

  float T0 = (float)T0_x8 / 8.0f;
  float T1 = (float)T1_x8 / 8.0f;

  int16_t T0_OUT = (int16_t)((uint16_t)hts221_read8(hi2c, 0x3C) | ((uint16_t)hts221_read8(hi2c, 0x3D) << 8));
  int16_t T1_OUT = (int16_t)((uint16_t)hts221_read8(hi2c, 0x3E) | ((uint16_t)hts221_read8(hi2c, 0x3F) << 8));
  int16_t T_OUT  = (int16_t)((uint16_t)hts221_read8(hi2c, 0x2A) | ((uint16_t)hts221_read8(hi2c, 0x2B) << 8));

  if (T1_OUT == T0_OUT) return T0;
  return T0 + ((float)(T_OUT - T0_OUT)) * (T1 - T0) / (float)(T1_OUT - T0_OUT);
}

/* ============================================================
   E) MQ-2 : lecture ADC1
   ============================================================ */
static uint16_t MQ2_ReadADC(void)
{
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 20);
  uint16_t v = (uint16_t)HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  return v;
}

/* ============================================================
   F) Lecture capteurs périodique (toutes les 50ms)
   - met à jour tempC, temp_mesuree, gaz_adc, gaz_detecte
   ============================================================ */
static void Sensors_Task(void)
{
  if (!system_on) return;

  if (HAL_GetTick() - last_sense_ms >= 50u) {
    last_sense_ms = HAL_GetTick();

    tempC = hts221_get_tempC(&hi2c2);
    temp_mesuree = (int16_t)lroundf(tempC);

    gaz_adc = MQ2_ReadADC();

    /* Hystérésis gaz pour éviter oscillations */
    uint16_t th_on  = (uint16_t)(gaz_seuil + GAS_HYST);
    uint16_t th_off = (gaz_seuil > GAS_HYST) ? (uint16_t)(gaz_seuil - GAS_HYST) : 0u;

    if (!gaz_detecte && (gaz_adc >= th_on)) gaz_detecte = 1;
    else if (gaz_detecte && (gaz_adc <= th_off)) gaz_detecte = 0;
  }
}

/* ============================================================
   G) Format affichage température (xx.xx) sans float printf
   ============================================================ */
static void FormatTemp2Dec(char out16[17], float t)
{
  int t100 = (int)lroundf(t * 100.0f);
  int sign = 0;
  if (t100 < 0) { sign = 1; t100 = -t100; }

  int whole = t100 / 100;
  int frac  = t100 % 100;

  if (sign) snprintf(out16, 17, "-%d.%02d", whole, frac);
  else      snprintf(out16, 17, "%d.%02d", whole, frac);
}

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  LCD_Init(&hi2c1, 0x27);
  hts221_power_on(&hi2c2);

  /* Etat initial : OFF */
  LCD_ClearCache();
  LCD_PrintLine(0, "System OFF");
  LCD_PrintLine(1, "Press ON/OFF");

  /* Synchroniser les boutons pour éviter un faux appui au démarrage */
  last_btn   = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
  last_plus  = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
  last_minus = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
  last_ok    = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);

  /* Interdire les appuis pendant une courte période après boot */
  uint32_t now0 = HAL_GetTick();
  next_on_ms    = now0 + DEB_ON_MS;
  next_plus_ms  = now0 + DEB_PLUS_MS;
  next_minus_ms = now0 + DEB_MINUS_MS;
  next_ok_ms    = now0 + DEB_OK_MS;
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */

    /* 1) tâches périodiques */
    Sensors_Task();
    Buzzer_Task();

    /* 2) lecture niveau boutons */
    uint8_t btn_on    = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
    uint8_t btn_plus  = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
    uint8_t btn_minus = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
    uint8_t btn_ok    = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);

    /* 3) transformer en évènements (appui) */
    uint8_t ev_on    = Btn_FallingEdge(&last_btn,   btn_on,    &next_on_ms,    DEB_ON_MS);
    uint8_t ev_plus  = Btn_FallingEdge(&last_plus,  btn_plus,  &next_plus_ms,  DEB_PLUS_MS);
    uint8_t ev_minus = Btn_FallingEdge(&last_minus, btn_minus, &next_minus_ms, DEB_MINUS_MS);
    uint8_t ev_ok    = Btn_FallingEdge(&last_ok,    btn_ok,    &next_ok_ms,    DEB_OK_MS);

    /* ==================== ON/OFF ==================== */
    if (ev_on)
    {
      system_on = !system_on;

      if (!system_on)
      {
        /* Passage OFF : couper tout */
        state = ST_OFF;
        buzzer_done = 0;
        buzzer_active = 0;
        HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(led_verte_GPIO_Port, led_verte_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(led_jaune_GPIO_Port, led_jaune_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(led_rouge_GPIO_Port, led_rouge_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(led_bleu_GPIO_Port, led_bleu_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ventilateur_GPIO_Port, ventilateur_Pin, GPIO_PIN_RESET);

        LCD_ClearCache();
        LCD_PrintLine(0, "System OFF");
        LCD_PrintLine(1, "Press ON/OFF");
      }
      else
      {
        /* Passage ON : message boot */
        state = ST_BOOT_MSG;
        state_t0_ms = HAL_GetTick();
        buzzer_done = 0;

        LCD_ClearCache();
        LCD_PrintLine(0, "Bonjour");
        LCD_PrintLine(1, "EMS_SYSTEM");
      }
    }

    /* ==================== MACHINE D’ETATS ==================== */
    prev_state = state;

    if (state == ST_OFF)
    {
      /* rien */
    }

    else if (state == ST_BOOT_MSG)
    {
      if (HAL_GetTick() - state_t0_ms >= 2000u) {
        state = ST_SHOW_MEAS;
        state_t0_ms = HAL_GetTick();
        LCD_ClearCache();
      }
    }

    else if (state == ST_SHOW_MEAS)
    {
      char t[17], l0[17], l1[17];
      FormatTemp2Dec(t, tempC);
      snprintf(l0, sizeof(l0), "Temp:%s C", t);
      snprintf(l1, sizeof(l1), "Gaz:%4u", gaz_adc);
      LCD_PrintLine(0, l0);
      LCD_PrintLine(1, l1);

      if (HAL_GetTick() - state_t0_ms >= 2000u) {
        state = ST_SET_TEMP;
        LCD_ClearCache();
      }
    }

    else if (state == ST_SET_TEMP)
    {
      char t[17], l1[17];
      FormatTemp2Dec(t, tempC);

      LCD_PrintLine(0, "Seuil TEMP (C)");
      snprintf(l1, sizeof(l1), "M:%s S:%2d", t, (int)temp_seuil);
      LCD_PrintLine(1, l1);

      /* +/- change le seuil température */
      if (ev_plus)  { if (temp_seuil < TEMP_SEUIL_MAX) temp_seuil++; }
      if (ev_minus) { if (temp_seuil > TEMP_SEUIL_MIN) temp_seuil--; }

      /* OK -> réglage gaz */
      if (ev_ok) {
        state = ST_SET_GAS;
        LCD_ClearCache();
      }
    }

    else if (state == ST_SET_GAS)
    {
      char l1[17];
      LCD_PrintLine(0, "Seuil GAZ (ADC)");
      snprintf(l1, sizeof(l1), "M:%4u S:%4u", gaz_adc, gaz_seuil);
      LCD_PrintLine(1, l1);

      /* +/- change le seuil gaz */
      if (ev_plus)  { if (gaz_seuil < 4095u - 10u) gaz_seuil += 10u; else gaz_seuil = 4095u; }
      if (ev_minus) { if (gaz_seuil >= 10u)        gaz_seuil -= 10u; else gaz_seuil = 0u;      }

      /* OK -> mode normal */
      if (ev_ok) {
        state = ST_NORMAL;
        buzzer_done = 0;
        LCD_ClearCache();
      }
    }

    else if (state == ST_NORMAL)
    {
      /* ✅ ajuster seuil température à chaud */
      if (ev_plus)  { if (temp_seuil < TEMP_SEUIL_MAX) temp_seuil++; }
      if (ev_minus) { if (temp_seuil > TEMP_SEUIL_MIN) temp_seuil--; }

      /* Affichage normal : ligne0 montre le seuil */
      char t[17], l0[17], l1[17];
      FormatTemp2Dec(t, tempC);

      snprintf(l0, sizeof(l0), "F normal|S=%2d", (int)temp_seuil);
      snprintf(l1, sizeof(l1), "T=%s G=%4u", t, gaz_adc);
      LCD_PrintLine(0, l0);
      LCD_PrintLine(1, l1);

      /* Sorties normal */
      HAL_GPIO_WritePin(led_verte_GPIO_Port, led_verte_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(led_jaune_GPIO_Port, led_jaune_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(led_rouge_GPIO_Port, led_rouge_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(led_bleu_GPIO_Port, led_bleu_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ventilateur_GPIO_Port, ventilateur_Pin, GPIO_PIN_RESET);

      /* Passage en alarmes selon conditions */
      uint8_t temp_high = (tempC > ((float)temp_seuil + TEMP_HYST_ON)) ? 1u : 0u;

      if (gaz_detecte && temp_high)      { state = ST_BOTH_ALARM; buzzer_done = 0; LCD_ClearCache(); }
      else if (gaz_detecte)              { state = ST_GAS_ALARM;  buzzer_done = 0; LCD_ClearCache(); }
      else if (temp_high)                { state = ST_TEMP_ALARM; buzzer_done = 0; LCD_ClearCache(); }
    }

    else if (state == ST_TEMP_ALARM)
    {
      /* ✅ ajuster seuil température à chaud même en alarme */
      if (ev_plus)  { if (temp_seuil < TEMP_SEUIL_MAX) temp_seuil++; }
      if (ev_minus) { if (temp_seuil > TEMP_SEUIL_MIN) temp_seuil--; }

      /* Sorties alarme temp */
      HAL_GPIO_WritePin(led_verte_GPIO_Port, led_verte_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(led_jaune_GPIO_Port, led_jaune_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(led_rouge_GPIO_Port, led_rouge_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(led_bleu_GPIO_Port, led_bleu_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ventilateur_GPIO_Port, ventilateur_Pin, GPIO_PIN_RESET);

      /* LCD */
      char l1[17];
      LCD_PrintLine(0, "Temp > seuil");
      snprintf(l1, sizeof(l1), "T=%2d S=%2d", (int)temp_mesuree, (int)temp_seuil);
      LCD_PrintLine(1, l1);

      /* Buzzer 2s une seule fois */
      if (!buzzer_done) { Buzzer_Beep(2000u); buzzer_done = 1; }

      /* Si gaz arrive => alarme combinée */
      if (gaz_detecte) { state = ST_BOTH_ALARM; buzzer_done = 0; LCD_ClearCache(); }

      /* Retour normal si temp redescend */
      if (tempC <= ((float)temp_seuil - TEMP_HYST_OFF) && !gaz_detecte) {
        state = ST_NORMAL; buzzer_done = 0; LCD_ClearCache();
      }
    }

    else if (state == ST_GAS_ALARM)
    {
      /* Sorties alarme gaz */
      HAL_GPIO_WritePin(led_verte_GPIO_Port, led_verte_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(led_jaune_GPIO_Port, led_jaune_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(led_rouge_GPIO_Port, led_rouge_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(led_bleu_GPIO_Port, led_bleu_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ventilateur_GPIO_Port, ventilateur_Pin, GPIO_PIN_SET);

      /* LCD */
      char l1[17];
      LCD_PrintLine(0, "Detection gaz");
      snprintf(l1, sizeof(l1), "ADC=%4u S=%4u", gaz_adc, gaz_seuil);
      LCD_PrintLine(1, l1);

      if (!buzzer_done) { Buzzer_Beep(2000u); buzzer_done = 1; }

      /* Si temp devient haute => alarme combinée */
      if (tempC > ((float)temp_seuil + TEMP_HYST_ON)) {
        state = ST_BOTH_ALARM; buzzer_done = 0; LCD_ClearCache();
      }

      /* Retour normal si gaz disparaît */
      if (!gaz_detecte) {
        HAL_GPIO_WritePin(ventilateur_GPIO_Port, ventilateur_Pin, GPIO_PIN_RESET);
        state = ST_NORMAL; buzzer_done = 0; LCD_ClearCache();
      }
    }

    else if (state == ST_BOTH_ALARM)
    {
      /* ✅ ajuster seuil température à chaud */
      if (ev_plus)  { if (temp_seuil < TEMP_SEUIL_MAX) temp_seuil++; }
      if (ev_minus) { if (temp_seuil > TEMP_SEUIL_MIN) temp_seuil--; }

      /* Sorties combinées */
      HAL_GPIO_WritePin(led_verte_GPIO_Port, led_verte_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(led_jaune_GPIO_Port, led_jaune_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(led_rouge_GPIO_Port, led_rouge_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(led_bleu_GPIO_Port, led_bleu_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(ventilateur_GPIO_Port, ventilateur_Pin, GPIO_PIN_SET);

      /* LCD */
      char l1[17];
      LCD_PrintLine(0, "Temp + Gaz !");
      snprintf(l1, sizeof(l1), "T=%2d G=%4u", (int)temp_mesuree, gaz_adc);
      LCD_PrintLine(1, l1);

      if (!buzzer_done) { Buzzer_Beep(2000u); buzzer_done = 1; }

      /* Gestion transitions */
      if (!gaz_detecte && (tempC > ((float)temp_seuil + TEMP_HYST_ON))) {
        state = ST_TEMP_ALARM; buzzer_done = 0; LCD_ClearCache();
      }
      else if (gaz_detecte && (tempC <= ((float)temp_seuil - TEMP_HYST_OFF))) {
        state = ST_GAS_ALARM;  buzzer_done = 0; LCD_ClearCache();
      }
      else if (!gaz_detecte && (tempC <= ((float)temp_seuil - TEMP_HYST_OFF))) {
        HAL_GPIO_WritePin(ventilateur_GPIO_Port, ventilateur_Pin, GPIO_PIN_RESET);
        state = ST_NORMAL; buzzer_done = 0; LCD_ClearCache();
      }
    }

    /* USER CODE END 3 */
  }
}



/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10D19CE4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ventilateur_Pin|Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, led_bleu_Pin|led_rouge_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_jaune_GPIO_Port, led_jaune_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_verte_GPIO_Port, led_verte_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ventilateur_Pin Buzzer_Pin */
  GPIO_InitStruct.Pin = ventilateur_Pin|Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : button__Pin on_off_Pin valid_temp_Pin */
  GPIO_InitStruct.Pin = button__Pin|on_off_Pin|valid_temp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : led_bleu_Pin led_rouge_Pin */
  GPIO_InitStruct.Pin = led_bleu_Pin|led_rouge_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : led_jaune_Pin */
  GPIO_InitStruct.Pin = led_jaune_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_jaune_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : led_verte_Pin */
  GPIO_InitStruct.Pin = led_verte_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_verte_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : button_B4_Pin */
  GPIO_InitStruct.Pin = button_B4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(button_B4_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
