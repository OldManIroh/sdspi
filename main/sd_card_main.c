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
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_sntp.h"

/* ==================== Конфигурация Wi‑Fi ==================== */
#define WIFI_SSID "TP-Link_BDF2"          // Имя Wi‑Fi сети
#define WIFI_PASS "12182541"               // Пароль
#define WIFI_MAX_RETRY 5                    // Макс. количество попыток подключения

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "time_logger";
static int s_retry_num = 0;

/* Обработчик событий Wi‑Fi */
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Повторная попытка подключения к точке доступа");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Не удалось подключиться к точке доступа");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Получен IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* Инициализация Wi‑Fi в режиме станции */
static void wifi_init_sta(void)
{
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

    ESP_LOGI(TAG, "Инициализация Wi-Fi завершена, ожидание подключения...");

    /* Ожидание подключения или ошибки */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Подключено к SSID: %s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Не удалось подключиться к SSID: %s", WIFI_SSID);
        /* Здесь можно добавить перезагрузку или повторную попытку */
    } else {
        ESP_LOGE(TAG, "Непредвиденное событие");
    }
}

/* ==================== Получение времени по NTP ==================== */

/**
 * @brief Колбэк, вызываемый при успешной синхронизации времени.
 */
void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "=== NTP СИНХРОНИЗАЦИЯ УСПЕШНА! ===");
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "Текущее время: %s", strftime_buf);
}

/**
 * @brief Инициализация SNTP и ожидание синхронизации.
 *        Используются только статические NTP-серверы (DHCP не запрашивается).
 */
static void obtain_time(void)
{
    ESP_LOGI(TAG, "Инициализация SNTP со статическими серверами");

    // Конфигурация SNTP с резервными серверами
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG();
    config.num_of_servers = 4;                      // Количество резервных серверов
    config.servers[0] = "ntp.msk-ix.ru";
    config.servers[1] = "ru.pool.ntp.org";
    config.servers[2] = "pool.ntp.org";
    config.servers[3] = "time.google.com";
    config.sync_cb = time_sync_notification_cb;      // Колбэк при успехе

    esp_netif_sntp_init(&config);

    // Ожидание синхронизации (таймаут 60 секунд)
    int retry = 0;
    const int retry_count = 30;   // 30 попыток по 5 секунды
    while (esp_netif_sntp_sync_wait(5000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT && ++retry < retry_count) {
        ESP_LOGI(TAG, "Ожидание синхронизации NTP... (%d/%d)", retry, retry_count);
    }

    if (retry < retry_count) {
        ESP_LOGI(TAG, "Время успешно синхронизировано");
    } else {
        ESP_LOGE(TAG, "Не удалось синхронизировать время - проверьте сеть или NTP-серверы");
    }
}

/* ==================== Работа с SD‑картой ==================== */
#define PIN_NUM_MISO 20
#define PIN_NUM_MOSI 19
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   15

#define MOUNT_POINT      "/sdcard"
#define MIN_FREE_BYTES   (1024 * 1024)   // Минимум 1 МБ свободно

static sdmmc_card_t *s_card = NULL;

/* Монтирование SD‑карты. Возвращает ESP_OK при успехе. */
static esp_err_t mount_sd_card(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,   // не форматировать, если не удалось примонтировать
        .max_files = 5,                     // макс. количество одновременно открытых файлов
        .allocation_unit_size = 16 * 1024   // размер кластера
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
        ESP_LOGE(TAG, "Не удалось инициализировать SPI: %s", esp_err_to_name(ret));
        return ret;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &s_card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Не удалось примонтировать файловую систему. Возможно, карта не отформатирована.");
        } else {
            ESP_LOGE(TAG, "Ошибка инициализации карты: %s", esp_err_to_name(ret));
        }
        spi_bus_free(host.slot);
        return ret;
    }

    sdmmc_card_print_info(stdout, s_card);
    return ESP_OK;
}

/* Проверка свободного места на SD‑карте */
static bool check_free_space(void)
{
    uint64_t total_bytes, free_bytes;
    esp_err_t ret = esp_vfs_fat_info(MOUNT_POINT, &total_bytes, &free_bytes);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка esp_vfs_fat_info: %s", esp_err_to_name(ret));
        return false;
    }
    if (free_bytes < MIN_FREE_BYTES) {
        ESP_LOGW(TAG, "Мало свободного места: %llu байт (< %d)", free_bytes, MIN_FREE_BYTES);
        return false;
    }
    return true;
}

/* ==================== Задача записи меток времени ==================== */
#define WRITE_INTERVAL_MS      200      // интервал между замерами (мс)
#define FILE_ROTATION_SEC      600      // смена файла каждые 10 минут

// Массив названий дней (английские, чтобы избежать проблем с кодировкой FAT)
static const char *day_names[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

/**
 * @brief Формирует имя файла на основе текущего времени.
 *        Формат: /sdcard/День_ЧЧММ.txt, например /sdcard/Mon_1435.txt
 */
static void make_filename(char *buf, size_t len, const struct tm *tm_info)
{
    int wday = tm_info->tm_wday;
    if (wday < 0 || wday > 6) wday = 0;
    snprintf(buf, len, MOUNT_POINT "/%s_%02d%02d.txt",
             day_names[wday], tm_info->tm_hour, tm_info->tm_min);
}

#define BUFFER_FLUSH_INTERVAL_MS  60000   // сбрасывать буфер на карту каждую минуту
#define BUFFER_MAX_SIZE            4096   // максимальный размер буфера (4 КБ)

/**
 * @brief Задача, которая каждые 200 мс получает временную метку,
 *        накапливает данные в буфере и периодически сбрасывает на SD-карту.
 *        Буферизация нужна для уменьшения количества операций записи на карту,
 *        что продлевает срок её службы.
 */
static void write_task(void *pvParameters)
{
    FILE *f = NULL;
    char *buffer = malloc(BUFFER_MAX_SIZE);   // буфер в RAM
    size_t buffer_len = 0;                     // текущая заполненность буфера
    time_t last_open_time = 0;                  // время последнего открытия файла
    char filename[64];
    TickType_t last_flush_tick = xTaskGetTickCount(); // время последнего сброса по таймеру

    if (!buffer) {
        ESP_LOGE(TAG, "Не удалось выделить память под буфер");
        vTaskDelete(NULL);
    }

    while (1) {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        time_t now = tv.tv_sec;
        struct tm tm_info;
        localtime_r(&now, &tm_info);

        // Проверка, что время установлено (год >= 2024)
        if (tm_info.tm_year < (2024 - 1900)) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        // Проверка свободного места на карте
        if (!check_free_space()) {
            ESP_LOGE(TAG, "Нет свободного места. Остановка записи.");
            break;
        }

        // Ротация файла: каждые FILE_ROTATION_SEC секунд создаём новый файл
        if (f == NULL || (now - last_open_time) >= FILE_ROTATION_SEC) {
            // Перед закрытием старого файла сбрасываем буфер
            if (buffer_len > 0 && f) {
                size_t written = fwrite(buffer, 1, buffer_len, f);
                if (written != buffer_len) {
                    ESP_LOGE(TAG, "Ошибка записи буфера при ротации файла");
                }
                fflush(f);
                fsync(fileno(f));   // гарантируем запись на физический носитель
                buffer_len = 0;
            }
            if (f) {
                fclose(f);
                f = NULL;
            }
            make_filename(filename, sizeof(filename), &tm_info);
            // Открываем в режиме "a" (добавление в конец), чтобы не потерять данные,
            // если файл уже существует (например, после перезагрузки)
            f = fopen(filename, "a");
            if (!f) {
                ESP_LOGE(TAG, "Не удалось открыть файл %s", filename);
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }
            last_open_time = now;
            ESP_LOGI(TAG, "Открыт новый файл: %s", filename);
        }

        // Формируем строку с временем (включая миллисекунды)
        char time_str[64];
        int ms = tv.tv_usec / 1000;
        int len = snprintf(time_str, sizeof(time_str),
                           "%04d-%02d-%02d %02d:%02d:%02d.%03d\n",
                           tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
                           tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec, ms);

        // Если строка помещается в буфер, добавляем её
        if (buffer_len + len < BUFFER_MAX_SIZE) {
            memcpy(buffer + buffer_len, time_str, len);
            buffer_len += len;
        } else {
            // Буфер переполнен – сбрасываем всё на карту
            if (f) {
                size_t written = fwrite(buffer, 1, buffer_len, f);
                if (written != buffer_len) {
                    ESP_LOGE(TAG, "Ошибка записи буфера (переполнение)");
                }
                // Записываем текущую строку отдельно
                written = fwrite(time_str, 1, len, f);
                if (written != len) {
                    ESP_LOGE(TAG, "Ошибка записи текущей строки");
                }
                fflush(f);
                fsync(fileno(f));
            }
            buffer_len = 0;
        }

        // Проверяем, не пора ли сбросить буфер по времени (раз в минуту)
        TickType_t now_ticks = xTaskGetTickCount();
        if ((now_ticks - last_flush_tick) >= pdMS_TO_TICKS(BUFFER_FLUSH_INTERVAL_MS)) {
            if (buffer_len > 0 && f) {
                size_t written = fwrite(buffer, 1, buffer_len, f);
                if (written != buffer_len) {
                    ESP_LOGE(TAG, "Ошибка записи буфера по таймеру");
                }
                fflush(f);
                fsync(fileno(f));
                buffer_len = 0;
            }
            last_flush_tick = now_ticks;
        }

        vTaskDelay(pdMS_TO_TICKS(WRITE_INTERVAL_MS));
    }

    // Выход из цикла по ошибке – пытаемся сохранить остатки буфера
    if (buffer_len > 0 && f) {
        size_t written = fwrite(buffer, 1, buffer_len, f);
        if (written != buffer_len) {
            ESP_LOGE(TAG, "Ошибка записи остатка буфера");
        }
        fflush(f);
        fsync(fileno(f));
    }
    if (f) fclose(f);
    free(buffer);
    ESP_LOGI(TAG, "Задача записи завершена.");
    vTaskDelete(NULL);
}

/* ==================== Точка входа ==================== */
void app_main(void)
{
    // Инициализация NVS (необходимо для работы Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Подключаемся к Wi-Fi (функция ждёт подключения или ошибки)
    wifi_init_sta();

    // Получаем время по NTP
    obtain_time();

    // Устанавливаем часовой пояс (Москва, UTC+3)
    setenv("TZ", "MSK-3", 1);
    tzset();

    // Проверяем, что время действительно установлено (год >= 2024)
    time_t now;
    time(&now);
    struct tm tm_info;
    localtime_r(&now, &tm_info);
    if (tm_info.tm_year < (2024 - 1900)) {
        ESP_LOGE(TAG, "Время не установлено корректно. Остановка.");
        return;
    }

    // Монтируем SD‑карту
    if (mount_sd_card() != ESP_OK) {
        ESP_LOGE(TAG, "Не удалось примонтировать SD-карту. Выход.");
        return;
    }

    // Запускаем задачу записи
    xTaskCreate(write_task, "write_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Главная задача завершена. Задача записи выполняется.");
}