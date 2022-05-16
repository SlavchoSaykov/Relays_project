/* Simple HTTP Server Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_wifi.h>
#include <esp_event.h>
#include "esp_system.h"
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "protocol_examples_common.h"
#include <esp_http_server.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "spi_relays.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include <string.h>
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_hidh.h"
#include "esp_hid_gap.h"

//#include "semaphore.h"

//#define DEBUG
//#define CONFIG_IDF_TARGET_ESP32S2
/*
 This code demonstrates how to use the SPI master half duplex mode to drive relay matrix (32 relays)).
*/
#define CONFIG_BT_BLE_ENABLED 1

//static xSemaphoreHandle ble_semaphore = NULL;

#ifdef CONFIG_IDF_TARGET_ESP32
#  ifdef CONFIG_EXAMPLE_USE_SPI1_PINS
#    define RELAY_HOST    SPI1_HOST
// Use default pins, same as the flash chip.
#    define PIN_NUM_MISO 7
#    define PIN_NUM_MOSI 8
#    define PIN_NUM_CLK  6
#  else
#    define RELAY_HOST    HSPI_HOST
#    define PIN_NUM_MISO 18
#    define PIN_NUM_MOSI 23
#    define PIN_NUM_CLK  19
#  endif

#  define PIN_NUM_CS   13
#elif defined CONFIG_IDF_TARGET_ESP32S2
#  define RELAY_HOST    SPI2_HOST

#  define PIN_NUM_MISO 37
#  define PIN_NUM_MOSI 35
#  define PIN_NUM_CLK  36
#  define PIN_NUM_CS   34
#elif defined CONFIG_IDF_TARGET_ESP32C3
#  define RELAY_HOST    SPI2_HOST

#  define PIN_NUM_MISO 2
#  define PIN_NUM_MOSI 7
#  define PIN_NUM_CLK  6
#  define PIN_NUM_CS   10

#elif CONFIG_IDF_TARGET_ESP32S3
#  define RELAY_HOST    SPI2_HOST

#  define PIN_NUM_MISO 11
#  define PIN_NUM_MOSI 13
#  define PIN_NUM_CLK  12
#  define PIN_NUM_CS   10
#endif


/* Global variables */
relays_handle_t relays_handle;
static const char *TAG = "Relays";
static const char *TAG_BLE = "Bluetooth";

void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    printf("****************************************************");


    switch (event) {
    case ESP_HIDH_OPEN_EVENT: {
        if (param->open.status == ESP_OK) {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
            ESP_LOGI(TAG_BLE, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
            esp_hidh_dev_dump(param->open.dev, stdout);
        } else {
            ESP_LOGE(TAG_BLE, " OPEN failed!");
        }
        break;
    }
    case ESP_HIDH_BATTERY_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
        ESP_LOGI(TAG_BLE, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
        break;
    }
    case ESP_HIDH_INPUT_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
        ESP_LOGI(TAG_BLE, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
        ESP_LOG_BUFFER_HEX(TAG_BLE, param->input.data, param->input.length);
        break;
    }
    case ESP_HIDH_FEATURE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
        ESP_LOGI(TAG_BLE, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda),
                 esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id,
                 param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG_BLE, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDH_CLOSE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
        ESP_LOGI(TAG_BLE, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
        break;
    }
    default:
        ESP_LOGI(TAG_BLE, "EVENT: %d", event);
        break;
    }
}

#define SCAN_DURATION_SECONDS 5

void hid_demo_task(void *pvParameters)
{
    size_t results_len = 0;                  //SlaSay
    esp_hid_scan_result_t *results = NULL;   //SlaSay

//Dani begin code
    // while(1)
    // {

    //Semaphor wait
//    xSemaphoreTake(ble_semaphore, portMAX_DELAY);

//Dani end code

    // esp_hidh_config_t config_ble = {
    //     .callback = hidh_callback,
    //     .event_stack_size = 4096,
    //     .callback_arg = NULL,
    // };
    // ESP_ERROR_CHECK( esp_hidh_init(&config_ble) );

    ESP_LOGI(TAG_BLE, "SCAN...");
    //start scan for HID devices
    esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results);
//     ESP_LOGI(TAG_BLE, "SCAN: %u results", results_len);
//     if (results_len) {
//         esp_hid_scan_result_t *r = results;
//         esp_hid_scan_result_t *cr = NULL;
//         while (r) {
//            printf("  %s: " ESP_BD_ADDR_STR ", ", (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ", ESP_BD_ADDR_HEX(r->bda));
//            printf("RSSI: %d, ", r->rssi);
//            printf("USAGE: %s, ", esp_hid_usage_str(r->usage));
// #if CONFIG_BT_BLE_ENABLED
//             if (r->transport == ESP_HID_TRANSPORT_BLE) {
//                 cr = r;
//                 printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
//                 printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));
//             }
// #endif /* CONFIG_BT_BLE_ENABLED */
// #if CONFIG_BT_HID_HOST_ENABLED
//             if (r->transport == ESP_HID_TRANSPORT_BT) {
//                 cr = r;
//                 printf("COD: %s[", esp_hid_cod_major_str(r->bt.cod.major));
//                 esp_hid_cod_minor_print(r->bt.cod.minor, stdout);
//                 printf("] srv 0x%03x, ", r->bt.cod.service);
//                 print_uuid(&r->bt.uuid);
//                 printf(", ");
//             }
// #endif /* CONFIG_BT_HID_HOST_ENABLED */
//             printf("NAME: %s ", r->name ? r->name : "");
//             printf("\n");

//             printf("\nMAC \n");

//             if(!strcmp(r->name, "ProSeries-488B7"))
//             {
//                 esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
//             }
//             r = r->next;
//         }
//         // if (cr) {
//         //     //open the last result
//         //     esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
//         // }

//         //free the results
//         esp_hid_scan_results_free(results);
//     }
    
    vTaskDelay(pdMS_TO_TICKS(5000));  
//    static esp_hidh_dev_t *dev = NULL;

    esp_bd_addr_t mac_addr = {0xc6,0x3b,0x60,0x6c,0xcc,0x82};

//    esp_hidh_dev_close(dev);
//    dev = esp_hidh_dev_open(mac_addr, ESP_HID_TRANSPORT_BLE, BLE_ADDR_TYPE_RANDOM);
    esp_hidh_dev_open(mac_addr, ESP_HID_TRANSPORT_BLE, BLE_ADDR_TYPE_RANDOM);

//Dani begin code

//   }
//Dani end code
//    vTaskDelete(NULL);
}


/* An HTTP GET handler */
static esp_err_t output_get_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;
    char resp_str[40];
    int ret;

    // Parse req->uri

    ESP_LOGI(TAG, "URI: %s", req->uri);

    /* Get header value string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        /* Copy null terminated value string into buffer */
        if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Host: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-2") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-2", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Test-Header-2: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-1") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-1", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Test-Header-1: %s", buf);
        }
        free(buf);
    }

    /* Read URL query string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found URL query => %s", buf);

            uint16_t state = strtol(&buf[3],NULL,10);
#ifdef DEBUG
            static uint8_t output_id = 0;
            output_id++;
            if(output_id > 32) output_id = 0;
#else
            uint8_t output_id = 10*(buf[0]-'0') + buf[1]-'0';
#endif
            if(output_id != 0 && output_id <= 32)
            {
                ESP_LOGI(TAG, "Set out%d => %d ms", output_id,state); 
                sprintf(resp_str, "Relay : %d %s for %d msec",output_id, "is SET", state);    

                ret = spi_relays_set(relays_handle, output_id);  
                ESP_ERROR_CHECK(ret);
                vTaskDelay(pdMS_TO_TICKS(state));  
                ret = spi_relays_reset(relays_handle);
                ESP_ERROR_CHECK(ret);
            }
            else
            {
                ESP_LOGI(TAG, "There is a BLE request. Start pairing.");



                xTaskCreate(&hid_demo_task, "hid_task", 6 * 1024, NULL, 2, NULL);
//Dani begin code                
                // Callin app_init xTaskCreate(&hid_demo_task, "hid_task", 6 * 1024, NULL, 2, NULL);

                // Call semaphor post/put/signal
//                xSemaphoreGive(ble_semaphore);
//Dani end code
                sprintf(resp_str, "BLE devices are paired.");    
            }
        }
        free(buf);
    }

    /* Set some custom headers */
    httpd_resp_set_hdr(req, "Custom-Header-1", "Custom-Value-1");
    httpd_resp_set_hdr(req, "Custom-Header-2", "Custom-Value-2");


    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

    /* After sending the HTTP response the old HTTP request
     * headers are lost. Check if HTTP request headers can be read now. */
    if (httpd_req_get_hdr_value_len(req, "Host") == 0) {
        ESP_LOGI(TAG, "Request headers lost");
    }
    return ESP_OK;
}

static const httpd_uri_t output = {
    .uri       = "/out",
    .method    = HTTP_GET,
    .handler   = output_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = "OK"
};

/* An HTTP POST handler */
static esp_err_t echo_post_handler(httpd_req_t *req)
{
    char buf[100];
    int ret, remaining = req->content_len;

    while (remaining > 0) {
        /* Read the data for the request */
        if ((ret = httpd_req_recv(req, buf,
                        MIN(remaining, sizeof(buf)))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }

        /* Send back the same data */
        httpd_resp_send_chunk(req, buf, ret);
        remaining -= ret;

        /* Log data received */
        ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
        ESP_LOGI(TAG, "%.*s", ret, buf);
        ESP_LOGI(TAG, "====================================");
    }

    // End response
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t echo = {
    .uri       = "/echo",
    .method    = HTTP_POST,
    .handler   = echo_post_handler,
    .user_ctx  = NULL
};

/* This handler allows the custom error handling functionality to be
 * tested from client side. For that, when a PUT request 0 is sent to
 * URI /ctrl, the /out and /echo URIs are unregistered and following
 * custom error handler http_404_error_handler() is registered.
 * Afterwards, when /out or /echo is requested, this custom error
 * handler is invoked which, after sending an error message to client,
 * either closes the underlying socket (when requested URI is /echo)
 * or keeps it open (when requested URI is /out). This allows the
 * client to infer if the custom error handler is functioning as expected
 * by observing the socket state.
 */
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    if (strcmp("/out", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/out URI is not available");
        /* Return ESP_OK to keep underlying socket open */
        return ESP_OK;
    } else if (strcmp("/echo", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/echo URI is not available");
        /* Return ESP_FAIL to close underlying socket */
        return ESP_FAIL;
    }
    /* For any other URI send 404 and close socket */
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Some 404 error message");
    return ESP_FAIL;
}

/* An HTTP PUT handler. This demonstrates realtime
 * registration and deregistration of URI handlers
 */
static esp_err_t ctrl_put_handler(httpd_req_t *req)
{
    char buf;
    int ret;

    if ((ret = httpd_req_recv(req, &buf, 1)) <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }

    if (buf == '0') {
        /* URI handlers can be unregistered using the uri string */
        ESP_LOGI(TAG, "Unregistering /out and /echo URIs");
        httpd_unregister_uri(req->handle, "/out");
        httpd_unregister_uri(req->handle, "/echo");
        /* Register the custom error handler */
        httpd_register_err_handler(req->handle, HTTPD_404_NOT_FOUND, http_404_error_handler);
    }
    else {
        ESP_LOGI(TAG, "Registering /out and /echo URIs");
        httpd_register_uri_handler(req->handle, &output);
        httpd_register_uri_handler(req->handle, &echo);
        /* Unregister custom error handler */
        httpd_register_err_handler(req->handle, HTTPD_404_NOT_FOUND, NULL);
    }

    /* Respond with empty body */
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t ctrl = {
    .uri       = "/ctrl",
    .method    = HTTP_PUT,
    .handler   = ctrl_put_handler,
    .user_ctx  = NULL
};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &output);
        httpd_register_uri_handler(server, &echo);
        httpd_register_uri_handler(server, &ctrl);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        stop_webserver(*server);
        *server = NULL;
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}

void app_main(void)
{
    static httpd_handle_t server = NULL;
    esp_err_t ret;

//  Bluetooth  Part Begin
#if HID_HOST_MODE == HIDH_IDLE_MODE
    ESP_LOGE(TAG_BLE, "Please turn on BT HID host or BLE!");
    return;
#endif

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK( ret );

    ESP_LOGI(TAG_BLE, "setting hid gap, mode:%d", HID_HOST_MODE);
    ESP_ERROR_CHECK( esp_hid_gap_init(HID_HOST_MODE) );
#if CONFIG_BT_BLE_ENABLED
    ESP_ERROR_CHECK( esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler) );
#endif /* CONFIG_BT_BLE_ENABLED */

    esp_hidh_config_t config_ble = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK( esp_hidh_init(&config_ble) );

//    xTaskCreate(&hid_demo_task, "hid_task", 6 * 1024, NULL, 2, NULL);

//    ble_semaphore = xSemaphoreCreateBinary();

// Bluetooth  Part End

#ifndef CONFIG_EXAMPLE_USE_SPI1_PINS
    ESP_LOGI(TAG, "Initializing bus SPI%d...", RELAY_HOST+1);
    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    //Initialize the SPI bus
    ret = spi_bus_initialize(RELAY_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
#else
    ESP_LOGI(TAG, "Attach to main flash bus...");
#endif
    relays_config_t relays_config = {
        .cs_io = PIN_NUM_CS,
        .host = RELAY_HOST,
        .miso_io = PIN_NUM_MISO,
    };
#ifdef CONFIG_EXAMPLE_INTR_USED
    relays_config.intr_used = true;
    gpio_install_isr_service(0);
#endif

    ESP_LOGI(TAG, "Initializing device...");
    ret = spi_relays_init(&relays_config, &relays_handle);
    ESP_ERROR_CHECK(ret);

    spi_relays_reset(relays_handle);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
    tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA ,"Relays");

    /* Register event handlers to stop the server when Wi-Fi or Ethernet is disconnected,
     * and re-start it upon connection.
     */
#ifdef CONFIG_EXAMPLE_CONNECT_WIFI
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_WIFI
#ifdef CONFIG_EXAMPLE_CONNECT_ETHERNET
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ETHERNET_EVENT_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_ETHERNET

    /* Start the server for the first time */
    server = start_webserver();
}
