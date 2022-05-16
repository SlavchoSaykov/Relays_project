/*
    This code demonstrates how to use the SPI master half duplex mode to drive relay matrix (32 relays)).

*/

#pragma once
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#define RELAYS_BASE 0x00000001

/// Configurations of the spi_relay
typedef struct {
    spi_host_device_t host; ///< The SPI host used, set before calling `spi_relays_init()`
    gpio_num_t cs_io;       ///< CS gpio number, set before calling `spi_relays_init()`
    gpio_num_t miso_io;     ///< MISO gpio number, set before calling `spi_relays_init()`
    bool intr_used;         ///< Whether to use polling or interrupt when waiting for write to be done. Set before calling `spi_relays_init()`.
} relays_config_t;

typedef struct relays_context_t* relays_handle_t;

/**
 * @brief Initialize the hardware.
 *
 * @param config Configuration of the RELAY
 * @param out_handle Output context of RELAY communication.
 * @return
 *  - ESP_OK: on success
 *  - ESP_ERR_INVALID_ARG: If the configuration in the context is incorrect.
 *  - ESP_ERR_NO_MEM: if semaphore create failed.
 *  - or other return value from `spi_bus_add_device()` or `gpio_isr_handler_add()`.
 */
esp_err_t spi_relays_init(const relays_config_t *config, relays_handle_t* out_handle);

/**
 * @brief Release the resources used by the RELAY.
 *
 * @param handle Context of RELAY communication.
 * @return Always ESP_OK
 */
esp_err_t spi_relay_deinit(relays_handle_t handle);

/**
 * @brief Write a byte into the RELAY buffer
 *
 * @param handle Context of RELAY communication.
 * @param data  The byte to write.
 * @return 
 *  - ESP_OK: on success
 *  - ESP_ERR_TIMEOUT: if the RELAY is not able to be ready before the time in the spec. This may mean that the connection is not correct.
 *  - or return value from `spi_device_acquire_bus()` `spi_device_polling_transmit()`.
 */
esp_err_t spi_relays_write(relays_handle_t handle, uint8_t addr, uint8_t data);

/**
 * @brief Switching ON a relay from the matrix   
 *
 * @param handle Context of RELAY communication.
 * @param output_id The number of the relay in matrix that has to be switched ON
 * @return 
 *  - ESP_OK: on success
 *  - ESP_ERR_TIMEOUT: if the RELAY is not able to be ready before the time in the spec. This may mean that the connection is not correct.
 *  - or return value from `spi_device_acquire_bus()` `spi_device_polling_transmit()`.
 */

esp_err_t spi_relays_set(relays_handle_t handle, uint8_t output_id);

/**
 * @brief Switch OFF all the relays in the relay matrix
 * @param handle Context of RELAY communication.
 * @return Always ESP_OK
 *  - ESP_OK: on success
 */
esp_err_t spi_relays_reset(relays_handle_t handle);
