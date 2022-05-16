/*
    This code demonstrates how to use the SPI master half duplex mode to drive relay matrix (32 relays)).

*/

#include "spi_relays.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include <unistd.h>
#include "esp_log.h"
#include <sys/param.h>
#include "sdkconfig.h"

#define RELAYS_BUSY_TIMEOUT_MS  5
#define RELAYS_CLK_FREQ         (1*1000*1000)   //When powered by 3.3V, RELAYS max freq is 1MHz
#define RELAYS_INPUT_DELAY_NS   ((1000*1000*1000/RELAYS_CLK_FREQ)/2+20)

/// Context (config and data) of the spi_relay
struct relays_context_t{
    relays_config_t cfg;        ///< Configuration by the caller.
    spi_device_handle_t spi;    ///< SPI device handle
    xSemaphoreHandle ready_sem; ///< Semaphore for ready signal
};

typedef struct relays_context_t relays_context_t;
static const char TAG[] = "relay";

// Workaround: The driver depends on some data in the flash and cannot be placed to DRAM easily for
// now. Using the version in LL instead.
#define gpio_set_level  gpio_set_level_patch
#include "hal/gpio_ll.h"
static inline esp_err_t gpio_set_level_patch(gpio_num_t gpio_num, uint32_t level)
{
    gpio_ll_set_level(&GPIO, gpio_num, level);
    return ESP_OK;
}

static esp_err_t relay_wait_done(relays_context_t* ctx)
{
    //have to keep cs low for 250ns
    usleep(1);
    //clear signal
    if (ctx->cfg.intr_used) {
        xSemaphoreTake(ctx->ready_sem, 0);
        gpio_set_level(ctx->cfg.cs_io, 1);
        gpio_intr_enable(ctx->cfg.miso_io);

        //Max processing time is 5ms, tick=1 may happen very soon, set to 2 at least
        uint32_t tick_to_wait = MAX(RELAYS_BUSY_TIMEOUT_MS / portTICK_PERIOD_MS, 2);
        BaseType_t ret = xSemaphoreTake(ctx->ready_sem, tick_to_wait);
        gpio_intr_disable(ctx->cfg.miso_io);
        gpio_set_level(ctx->cfg.cs_io, 0);

        if (ret != pdTRUE) return ESP_ERR_TIMEOUT;
    } else {
        bool timeout = true;
        gpio_set_level(ctx->cfg.cs_io, 1);
        for (int i = 0; i < RELAYS_BUSY_TIMEOUT_MS * 1000; i ++) {
            if (gpio_get_level(ctx->cfg.miso_io)) {
                timeout = false;
                break;
            }
            usleep(1);
        }
        gpio_set_level(ctx->cfg.cs_io, 0);
        if (timeout) return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static void cs_high(spi_transaction_t* t)
{
    ESP_EARLY_LOGV(TAG, "cs high %d.", ((relays_context_t*)t->user)->cfg.cs_io);
    gpio_set_level(((relays_context_t*)t->user)->cfg.cs_io, 1);
}

static void cs_low(spi_transaction_t* t)
{
    gpio_set_level(((relays_context_t*)t->user)->cfg.cs_io, 0);
    ESP_EARLY_LOGV(TAG, "cs low %d.", ((relays_context_t*)t->user)->cfg.cs_io);
}

void ready_rising_isr(void* arg)
{
    relays_context_t* ctx = (relays_context_t*)arg;
    xSemaphoreGive(ctx->ready_sem);
    ESP_EARLY_LOGV(TAG, "ready detected.");
}

esp_err_t spi_relay_deinit(relays_context_t* ctx)
{
    spi_bus_remove_device(ctx->spi);
    if (ctx->cfg.intr_used) {
        vSemaphoreDelete(ctx->ready_sem);
    }
    free(ctx);
    return ESP_OK;
}

esp_err_t spi_relays_init(const relays_config_t *cfg, relays_context_t** out_ctx)
{
    esp_err_t err = ESP_OK;
    if (cfg->intr_used && cfg->host == SPI1_HOST) {
        ESP_LOGE(TAG, "interrupt cannot be used on SPI1 host.");
        return ESP_ERR_INVALID_ARG;
    }

    relays_context_t* ctx = (relays_context_t*)malloc(sizeof(relays_context_t));
    if (!ctx) return ESP_ERR_NO_MEM;

    *ctx = (relays_context_t) {
        .cfg = *cfg,
    };

    spi_device_interface_config_t devcfg={
        .clock_speed_hz = RELAYS_CLK_FREQ,
        .mode = 0,          //SPI mode 0
        /*
         * The timing requirements to read the busy signal from the RELAYS cannot be easily emulated
         * by SPI transactions. We need to control CS pin by SW to check the busy signal manually.
         */
        .spics_io_num = -1,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_POSITIVE_CS,
        .pre_cb = cs_high,
        .post_cb = cs_low,
        .input_delay_ns = RELAYS_INPUT_DELAY_NS,  //the RELAYS output the data half a SPI clock behind.
    };
    //Attach the RELAYS to the SPI bus
    err = spi_bus_add_device(ctx->cfg.host, &devcfg, &ctx->spi);
    if  (err != ESP_OK) {
        goto cleanup;
    }

    gpio_set_level(ctx->cfg.cs_io, 0);
    gpio_config_t cs_cfg = {
        .pin_bit_mask = BIT64(ctx->cfg.cs_io),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cs_cfg);

    if (ctx->cfg.intr_used) {
        ctx->ready_sem = xSemaphoreCreateBinary();
        if (ctx->ready_sem == NULL) {
            err = ESP_ERR_NO_MEM;
            goto cleanup;
        }

        gpio_set_intr_type(ctx->cfg.miso_io, GPIO_INTR_POSEDGE);
        err = gpio_isr_handler_add(ctx->cfg.miso_io, ready_rising_isr, ctx);
        if (err != ESP_OK) {
            goto cleanup;
        }
        gpio_intr_disable(ctx->cfg.miso_io);
    }
    *out_ctx = ctx;
    return ESP_OK;

cleanup:
    if (ctx->spi) {
        spi_bus_remove_device(ctx->spi);
        ctx->spi = NULL;
    }
    if (ctx->ready_sem) {
        vSemaphoreDelete(ctx->ready_sem);
        ctx->ready_sem = NULL;
    }
    free(ctx);
    return err;
}

esp_err_t spi_relays_write(relays_context_t* ctx, uint8_t addr, uint8_t data)
{
    esp_err_t err;
    err = spi_device_acquire_bus(ctx->spi, portMAX_DELAY);
    if (err != ESP_OK) return err;

    spi_transaction_t t = {
        .length = 8,
        .flags = SPI_TRANS_USE_TXDATA,
        .tx_data = {data},
        .user = ctx,
    };
    err = spi_device_polling_transmit(ctx->spi, &t);

    if (err == ESP_OK) {
        err = relay_wait_done(ctx);
    }

    spi_device_release_bus(ctx->spi);
    return err;
}
esp_err_t spi_relays_set(relays_handle_t handle, uint8_t output_id)
{
    uint32_t relay_out = ~(RELAYS_BASE << (output_id-1));
    int ret;
    uint8_t byte3, byte2, byte1, byte0;
    byte3 = 0xFF&(relay_out >> 24);
    byte2 = 0xFF&(relay_out >> 16);
    byte1 = 0xFF&(relay_out >> 8);
    byte0 = 0xFF&relay_out;

    ret = spi_relays_write(handle, 0, byte3);
    ESP_ERROR_CHECK(ret);
    ret = spi_relays_write(handle, 0, byte2);
    ESP_ERROR_CHECK(ret);
    ret = spi_relays_write(handle, 0, byte1);
    ESP_ERROR_CHECK(ret);
    ret = spi_relays_write(handle, 0, byte0);
    ESP_ERROR_CHECK(ret);

    return ret;
}
esp_err_t spi_relays_reset(relays_handle_t handle)
{
    int ret; 
    uint8_t n_bytes_reset = 4;
    uint8_t relays_reset = 0xFF;

    for(int i = 0; i < n_bytes_reset;i++)
    {
        ret = spi_relays_write(handle, 0, relays_reset);
        ESP_ERROR_CHECK(ret);
    }

    return ret;
}