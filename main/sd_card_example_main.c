#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif_sntp.h"
#include "lwip/apps/sntp.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

/* ==================== Конфигурация Wi‑Fi ==================== */
#define WIFI_SSID           "majorpack"          // Замените на свой SSID
#define WIFI_PASS           "majorpack2023!"      // Замените на свой пароль
#define WIFI_MAX_RETRY      5

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

static const char *TAG = "time_logger";
static int s_retry_num = 0;

/* Обработчик событий Wi‑Fi (стандартный) */
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* Инициализация Wi‑Fi в режиме станции */
static void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Ожидание подключения или ошибки */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID: %s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID: %s", WIFI_SSID);
        /* В реальном проекте можно добавить перезагрузку или повторную попытку */
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

/* ==================== Получение времени по NTP (однократно) ==================== */
static void obtain_time(void) {
    ESP_LOGI(TAG, "Initializing SNTP with server: ntp.msk-ix.ru");

    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("ntp.msk-ix.ru");
    config.sync_cb = NULL;
    esp_netif_sntp_init(&config);

    int retry = 0;
    const int retry_count = 15;
    while (esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for NTP time sync... (%d/%d)", retry, retry_count);
    }

    if (retry < retry_count) {
        ESP_LOGI(TAG, "Time synchronized successfully");
    } else {
        ESP_LOGE(TAG, "Failed to synchronize time");
    }

    esp_netif_sntp_deinit();
}

/* ==================== Работа с SD‑картой ==================== */
// Пины SPI – можно задать через menuconfig или прямо здесь
#define PIN_NUM_MISO   20   
#define PIN_NUM_MOSI   19  
#define PIN_NUM_CLK    18   
#define PIN_NUM_CS     15     

#define MOUNT_POINT     "/sdcard"
#define MIN_FREE_BYTES  (1024 * 1024)             // Минимум 1 МБ свободно

static sdmmc_card_t *s_card = NULL;
static bool s_sd_mounted = false;

/* Монтирование SD‑карты. Возвращает ESP_OK при успехе. */
static esp_err_t mount_sd_card(void) {
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,   // Не форматировать автоматически
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &s_card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. Card may not be formatted.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card: %s", esp_err_to_name(ret));
        }
        spi_bus_free(host.slot);
        return ret;
    }

    s_sd_mounted = true;
    sdmmc_card_print_info(stdout, s_card);
    return ESP_OK;
}

/* Проверка свободного места на SD‑карте */
static bool check_free_space(void) {
    uint64_t total_bytes, free_bytes;   // используем uint64_t
    esp_err_t ret = esp_vfs_fat_info(MOUNT_POINT, &total_bytes, &free_bytes);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_vfs_fat_info failed: %s", esp_err_to_name(ret));
        return false;
    }
    if (free_bytes < MIN_FREE_BYTES) {
        ESP_LOGW(TAG, "Low free space: %llu bytes (< %d)", free_bytes, MIN_FREE_BYTES);
        return false;
    }
    return true;
}

/* ==================== Задача записи меток времени ==================== */
// Интервал записи 0.2 с = 200 мс
#define WRITE_INTERVAL_MS    200
// Интервал смены файла 10 минут = 600 секунд
#define FILE_ROTATION_SEC    600

// Массив сокращений дней недели (на английском)
static const char *day_names[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

/* Формирование имени файла на основе текущего времени */
static void make_filename(char *buf, size_t len, const struct tm *tm_info) {
    int wday = tm_info->tm_wday;  // 0 = воскресенье
    if (wday < 0 || wday > 6) wday = 0;
    snprintf(buf, len, MOUNT_POINT "/%s_%02d%02d.txt",
             day_names[wday], tm_info->tm_hour, tm_info->tm_min);
}

/* Задача: бесконечный цикл записи */
static void write_task(void *pvParameters) {
    FILE *f = NULL;
    time_t last_open_time = 0;          // время открытия текущего файла (сек)
    char filename[64];

    while (1) {
        // Получаем текущее время с микросекундами
        struct timeval tv;
        gettimeofday(&tv, NULL);
        time_t now = tv.tv_sec;
        struct tm tm_info;
        localtime_r(&now, &tm_info);

        // Проверка, что время установлено (год не 1970)
        if (tm_info.tm_year < (2024 - 1900)) {
            ESP_LOGW(TAG, "System time not yet valid, waiting...");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        // Проверка свободного места на карте
        if (!check_free_space()) {
            ESP_LOGE(TAG, "Insufficient free space. Stopping writes.");
            if (f) {
                fclose(f);
                f = NULL;
            }
            break;  // Выход из задачи
        }

        // Проверка, нужно ли сменить файл (каждые FILE_ROTATION_SEC секунд)
        if (f == NULL || (now - last_open_time) >= FILE_ROTATION_SEC) {
            // Закрываем старый файл, если открыт
            if (f) {
                fclose(f);
                f = NULL;
            }
            // Формируем имя нового файла
            make_filename(filename, sizeof(filename), &tm_info);
            ESP_LOGI(TAG, "Opening new file: %s", filename);

            // Открываем файл для записи (перезапись, если уже существует – но такого не должно быть)
            f = fopen(filename, "w");
            if (f == NULL) {
                ESP_LOGE(TAG, "Failed to open file %s", filename);
                // Если не удалось открыть, пробуем снова через некоторое время
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }
            last_open_time = now;
        }

        // Формируем строку с меткой времени (до миллисекунд)
        char time_str[64];
        int ms = tv.tv_usec / 1000;  // миллисекунды
        snprintf(time_str, sizeof(time_str),
                 "%04d-%02d-%02d %02d:%02d:%02d.%03d\n",
                 tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
                 tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec, ms);

        // Запись в файл
        int written = fprintf(f, "%s", time_str);
        if (written < 0) {
            ESP_LOGE(TAG, "Write error: %s", strerror(errno));
            fclose(f);
            f = NULL;
            // Можно попытаться переоткрыть файл на следующей итерации
        } else {
            // Сбрасываем буферы на карту (для надёжности)
            fflush(f);
        }

        // Ждём следующий интервал
        vTaskDelay(pdMS_TO_TICKS(WRITE_INTERVAL_MS));
    }

    // При выходе закрываем файл, если открыт
    if (f) {
        fclose(f);
    }
    ESP_LOGI(TAG, "Write task ended.");
    vTaskDelete(NULL);
}

/* ==================== Точка входа ==================== */
void app_main(void) {
    // Инициализация NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Подключаемся к Wi‑Fi (блокируется до подключения или ошибки)
    wifi_init_sta();

    // Получаем время по NTP (однократно)
    obtain_time();

    // Устанавливаем часовой пояс Москвы (UTC+3)
    setenv("TZ", "MSK-3", 1);
    tzset();

    // Проверяем, что время действительно установлено
    time_t now;
    time(&now);
    struct tm tm_info;
    localtime_r(&now, &tm_info);
    if (tm_info.tm_year < (2024 - 1900)) {
        ESP_LOGE(TAG, "Time not set correctly. Stopping.");
        return;
    }

    // Монтируем SD‑карту
    if (mount_sd_card() != ESP_OK) {
        ESP_LOGE(TAG, "SD card mount failed. Exiting.");
        return;
    }

    // Запускаем задачу записи
    xTaskCreate(write_task, "write_task", 4096, NULL, 5, NULL);

    // app_main может завершиться, задача продолжит работу
    ESP_LOGI(TAG, "Main task finished. Write task running.");
}