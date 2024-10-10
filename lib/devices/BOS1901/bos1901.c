/*
 * Copyright (c) 2024 Dennis Liu, dennis48161025@gmail.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "bos1901.h"

#ifdef LIB_USE_ESPIDF_PLATFORM
#pragma message("LIB_USE_ESPIDF_PLATFORM defined")
#elif defined LIB_USE_STM32_PLATFORM
#pragma message("LIB_USE_STM32_PLATFORM defined")
#else
#pragma message("LIB_USE_DUMMY_PLATFORM defined")
#endif

static const char *TAG = "bos1901";

/* static function declaration */

/**
 * @brief Updates the existing SPI interface with non-NULL pointers from the new
 * interface.
 *
 * This function only updates the function pointers that are non-NULL in the new
 * interface.
 *
 * @param existing_interface The existing SPI interface to be updated.
 * @param new_interface The new SPI interface containing the updates.
 */
static void bos1901_update_spi_interface(
    bos1901_spi_interface_t *existing_interface,
    bos1901_spi_interface_t new_interface);

static void bos1901_device_config(bos1901_dev_t *dev,
                                  bos1901_dev_config_t *dev_config);

/**
 * @brief Factory function to create and initialize a new SPI interface.
 *
 * @return Pointer to the created SPI interface, or NULL on failure.
 */
static bos1901_spi_interface_t *create_spi_interface();

/**
 * @brief Function to initialize the SPI device.
 *
 * @param dev Pointer to the BOS1901 device structure.
 * @param device_config Pointer to the SPI device configuration.
 */
static void bos1901_spi_device_init(bos1901_dev_t *dev,
                                    bos1901_spi_dev_config_t *device_config);

/**
 * @brief Function to deinitialize the SPI device.
 *
 * @param dev Pointer to the BOS1901 device structure.
 */
static void bos1901_spi_device_deinit(bos1901_dev_t *dev);

/**
 * @brief Reads and writes a single 16-bit word to the SPI device using the
 * SPI interface.
 *
 * This function performs a read/write operation on the SPI device. The data
 * is transmitted in a 16-bit word format, and the function will swap the byte
 * order to accommodate the endianness differences between the host and the
 * SPI device.
 *
 * @param dev Pointer to the BOS1901 device structure.
 * @param data The 16-bit word to be written to the SPI device.
 * @return The 16-bit word read from the SPI device.
 *
 * Example usage:
 * \code
 * uint16_t read_value = bos1901_spi_read_write_reg(device, 0x1234);
 * printf("Read value: 0x%04X\n", read_value);
 * \endcode
 */
static bos1901_err_t bos1901_spi_read_write_reg(bos1901_dev_t *dev,
                                                uint16_t data_tx, uint16_t *rx);

/**
 * @brief Reads and writes multiple 16-bit words to the SPI device using the
 * SPI interface.
 *
 * This function performs a read/write operation on the SPI device for an array
 * of 16-bit words. The data is transmitted in a 16-bit word format, and the
 * function will swap the byte order to accommodate the endianness differences
 * between the host and the SPI device.
 *
 * @param dev Pointer to the BOS1901 device structure.
 * @param data_tx Pointer to the array of 16-bit words to be written to the SPI
 * device.
 * @param data_rx Pointer to the array where the read data will be stored.
 * @param length The number of 16-bit words to be transmitted and received.
 * @return bos1901_err_t The status of the read/write operation.
 *
 * Example usage:
 * \code
 * uint16_t data_tx[] = {0x1234, 0x5678};
 * uint16_t data_rx[2];
 * bos1901_err_t err = bos1901_spi_device_read_write(device, data_tx, data_rx,
 * 2); if (err == BOS1901_OK)
 * {
 *     printf("Read values: 0x%04X, 0x%04X\n", data_rx[0], data_rx[1]);
 * }
 * else
 * {
 *     printf("Error during read/write: %d\n", err);
 * }
 * \endcode
 */
static bos1901_err_t bos1901_spi_device_read_write(bos1901_dev_t *dev,
                                                   uint16_t *data_tx,
                                                   uint16_t *data_rx,
                                                   uint32_t length);

/**
 * @brief Function to change the SPI interface for the BOS1901 device.
 *
 * This function updates the existing SPI interface of the BOS1901 device
 * with the new interface provided. Only the non-NULL parts of the new
 * interface will be used to update the existing interface, which means
 * that if any pointer in the new interface is NULL, the corresponding
 * pointer in the existing interface will remain unchanged.
 *
 * @param dev Pointer to the BOS1901 device structure.
 * @param new_interface The new SPI interface to be set. Only non-NULL
 * members will be used to update the existing interface.
 *
 * Example:
 * \code
 * bos1901_spi_interface_t custom_interface = {
 *     .name = "custom_interface",
 *     .description = "Custom SPI Interface",
 *     .init_device = custom_init_device, // Assuming non-NULL
 *     .deinit_device = custom_deinit_device, // Assuming non-NULL
 *     // Leave other members as NULL to retain the existing ones
 * };
 * bos1901_change_spi_interface(device, custom_interface);
 * \endcode
 */
static void bos1901_change_spi_interface(bos1901_dev_t *dev,
                                         bos1901_spi_interface_t new_interface);

/* public function definition */

/* Enter function */
bos1901_dev_t *bos1901_device_create(const char *dev_name)
{
  /* malloc bos1901_dev_t */
  bos1901_dev_t *dev = (bos1901_dev_t *)malloc(sizeof(bos1901_dev_t));
  if (dev == NULL)
  {
    BOS_LOGE(TAG, "Unable to allocate memory for bos1901_dev_t.");
    return NULL;
  }

  dev->dev_name = dev_name;

  /* malloc bos1901_dev_config_t */
  dev->dev_config =
      (bos1901_dev_config_t *)malloc(sizeof(bos1901_dev_config_t));
  if (dev->dev_config == NULL)
  {
    BOS_LOGE(TAG, "Unable to allocate memory for dev_config.");
    free(dev);
    return NULL;
  }

  /* malloc bos1901_spi_dev_t */
  dev->spi_dev = (bos1901_spi_dev_t *)malloc(sizeof(bos1901_spi_dev_t));
  if (dev->spi_dev == NULL)
  {
    BOS_LOGE(TAG, "Unable to allocate memory for spi_dev.");
    free(dev->dev_config);
    free(dev);
    return NULL;
  }

  dev->spi_dev->spi_interface_ptr = NULL;
  dev->spi_dev->device_config = NULL;

  BOS_LOGI(TAG, "Device {%s} created.", dev_name);

  return dev;
}

void bos1901_device_free(bos1901_dev_t *dev)
{
  if (dev == NULL) return;

  const char *dev_name = (dev->dev_name != NULL) ? dev->dev_name : "UNKNOWN";

  if (dev->spi_dev != NULL)
  {
    BOS_LOGI(TAG, "Freeing SPI device for dev {%s}.", dev_name);
    free(dev->spi_dev);
  }
  if (dev->dev_config != NULL)
  {
    BOS_LOGI(TAG, "Freeing device configuration for dev {%s}.", dev_name);
    free(dev->dev_config);
  }

  BOS_LOGI(TAG, "Freeing device {%s}.", dev_name);
  free(dev);
}

void bos1901_device_init(bos1901_dev_t *dev, bos1901_dev_config_t *dev_config,
                         bos1901_spi_dev_config_t *spi_device_config)
{
  const char *dev_name = (dev->dev_name != NULL) ? dev->dev_name : "UNKNOWN";
  BOS_LOGI(TAG, "Initializing device {%s}.", dev_name);

  /* spi device config */
  BOS_LOGI(TAG, "SPI device initializing...");
  bos1901_spi_device_init(dev, spi_device_config);
  BOS_LOGI(TAG, "SPI device config already completed");

  /* start device config */
  BOS_LOGI(TAG, "Start device config...");
  bos1901_device_config(dev, dev_config);
  BOS_LOGI(TAG, "Device config completed...");

  BOS_LOGI(TAG, "Device {%s} initialized.\n", dev_name);
}

void bos1901_device_deinit(bos1901_dev_t *dev)
{
  const char *dev_name = (dev->dev_name != NULL) ? dev->dev_name : "UNKNOWN";
  BOS_LOGI(TAG, "Deinitializing device {%s}.", dev_name);

  bos1901_spi_device_deinit(dev);
  bos1901_device_free(dev);

  BOS_LOGI(TAG, "Device {%s} deinitialized.\n", dev_name);
}

bos1901_err_t bos1901_device_write_reg(bos1901_dev_t *dev,
                                       bos1901_register_t reg, uint16_t data)
{
  const char *dev_name = (dev->dev_name != NULL) ? dev->dev_name : "UNKNOWN";
  /* TODO: v2, check reg is in range */
  const int address_reg_shift = 12;
  const int address_reg_mask = 0b1111;  // 4 bits

  /* clear address part */
  data &= ~(address_reg_mask << address_reg_shift);

  /* assign address reg part */
  data |= (reg << address_reg_shift);

  BOS_LOGV(TAG, "Send write command: {0x%04X}", data);

  bos1901_err_t err = bos1901_spi_read_write_reg(dev, data, NULL);

  if (err == BOS1901_OK)
  {
    BOS_LOGV(TAG, "Write to device {%s} successful.", dev_name);
  }
  else
  {
    BOS_LOGE(TAG, "Write to device {%s} failed with error code: %d.", dev_name,
             err);
  }

  return err;
}

bos1901_err_t bos1901_device_read_reg(bos1901_dev_t *dev,
                                      bos1901_register_t reg, uint16_t *data)
{
  const char *dev_name = (dev->dev_name != NULL) ? dev->dev_name : "UNKNOWN";

  /* usage: we need to set CONFIG to desire reg, then write one dummy byte to
   * read back data */

  /* TODO: v2, check reg is in range */

  /* define constants */
  const bos1901_register_t config_reg = BOS1901_REG_CONFIG;
  const bos1901_register_t bc = reg;

  // this fetch from cache (dev->private_status)
  const bos1901_oe_status_t oe = (dev->private_status.output_enable)
                                     ? BOS1901_OE_ENABLE
                                     : BOS1901_OE_DISABLE;

  const bos1901_lock_t lock = dev->dev_config->write_protect_lock;
  const bos1901_rst_t rst = BOS1901_RST_NORMAL;
  const bos1901_ds_mode_t ds = dev->dev_config->ds;
  const bos1901_play_sample_rate_t play_mode = dev->dev_config->play_mode;
  const uint16_t dummy = 0xC000;

  BOS_LOGV(TAG, "Detect oe status is: %s",
           (oe) ? "BOS1901_OE_ENABLE" : "BOS1901_OE_DISABLE");

  // // length
  // const uint32_t rw_length = 2;  // 2 words

  // shift
  const int address_reg_shift = 12;
  const int bc_shift = 7;
  const int lock_shift = 6;
  const int rst_shift = 5;
  const int oe_shift = 4;
  const int ds_shift = 3;
  const int play_shift = 0;

  // mask
  const int address_reg_mask = 0b1111;  // 4 bits
  const int bc_mask = 0b11111;          // 5 bits
  const int lock_mask = 0b1;            // 1 bit
  const int rst_mask = 0b1;             // 1 bit
  const int oe_mask = 0b1;              // 1 bit
  const int ds_mask = 0b1;              // 1 bit
  const int play_mask = 0b111;          // 3 bits

  /* format read register data_tx from reg */
  // uint16_t data_tx[2];
  uint16_t data_rx[2];

  /* format cmd */
  uint16_t cmd =
      ((config_reg & address_reg_mask) << address_reg_shift) |
      ((bc & bc_mask) << bc_shift) | ((lock & lock_mask) << lock_shift) |
      ((rst & rst_mask) << rst_shift) | ((oe & oe_mask) << oe_shift) |
      ((ds & ds_mask) << ds_shift) | ((play_mode & play_mask) << play_shift);

  /* assign cmd */
  // data_tx[0] = cmd;
  // data_tx[1] = dummy;

  BOS_LOGV(TAG, "Send cmd in `bos1901_device_read_reg`: 0x%04X", cmd);

  /* This function writes a command word to the BOS1901 device. Note that this
   * method cannot be used for reading data because BOS1901 requires additional
   * time to correctly push data to the SDO pin. This method is too fast for
   * reading accurate data, which is why adjustments were necessary.
   */
  // bos1901_err_t err = bos1901_device_read_write(dev, data_tx, data_rx,
  // rw_length);

  /* alternative, around 170 us in debug mode @ 35M */

  bos1901_device_read_write_reg(dev, cmd);
  data_rx[1] = bos1901_device_read_write_reg(dev, dummy);

  bos1901_err_t err = BOS1901_OK;

  if (err == BOS1901_OK)
  {
    BOS_LOGV(TAG, "Read from device {%s} successful.", dev_name);
    *data = data_rx[1];
  }
  else
  {
    BOS_LOGE(TAG, "Read from device {%s} failed with error code: %d.", dev_name,
             err);
  }

  return err;
}

bos1901_err_t bos1901_device_read_write(bos1901_dev_t *dev, uint16_t *data_tx,
                                        uint16_t *data_rx, uint32_t length)
{
  const char *dev_name = (dev->dev_name != NULL) ? dev->dev_name : "UNKNOWN";
  BOS_LOGV(TAG, "Read-write from device {%s}, data length: %u words.", dev_name,
           (unsigned int)length);

  if (length == 0)
  {
    return BOS1901_OK;
  }

  bos1901_err_t err =
      bos1901_spi_device_read_write(dev, data_tx, data_rx, length);

  if (err == BOS1901_OK)
  {
    BOS_LOGV(TAG, "Read-write from device {%s} successful.", dev_name);
  }
  else
  {
    BOS_LOGE(TAG, "Read-write from device {%s} failed with error code: %d.",
             dev_name, err);
  }

  return err;
}

bos1901_dev_config_t bos1901_get_device_default_config()
{
  bos1901_dev_config_t s = {0};
  return s;
}

uint16_t bos1901_device_read_write_reg(bos1901_dev_t *dev, uint16_t data_tx)
{
  uint16_t data_rx = 0;
  bos1901_err_t ret = bos1901_spi_read_write_reg(dev, data_tx, &data_rx);

  BOS_LOGV(TAG, "bos1901_spi_read_write_reg status result is: %d", ret);
  return data_rx;
}

void bos1901_device_change_interface(bos1901_dev_t *dev,
                                     bos1901_spi_interface_t new_interface)
{
  bos1901_change_spi_interface(dev, new_interface);
}

bos1901_err_t bos1901_device_output_enable(bos1901_dev_t *dev,
                                           bos1901_oe_status_t new_status)
{
  const char *dev_name = (dev->dev_name != NULL) ? dev->dev_name : "UNKNOWN";

  /* check status is in range */
  if (new_status >= BOS1901_OE_END)
  {
    BOS_LOGE(TAG,
             "ERR: new_status {%d} (bos1901_oe_status_t) in "
             "`bos1901_device_output_enable` out of range",
             new_status);
    return BOS1901_INVALID_ARG;
  }

  /* update private status */
  dev->private_status.output_enable = new_status;

  /* read reg */
  uint16_t config_value;
  bos1901_err_t ret =
      bos1901_device_read_reg(dev, BOS1901_REG_CONFIG, &config_value);

  if (ret != BOS1901_OK)
  {
    BOS_LOGE(TAG,
             "Execute `bos1901_device_output_enable` (read stage) failed. "
             "Error Code: {%d}",
             ret);
    return ret;
  }

  /* assign field */
  const int oe_shift = 4;
  uint16_t oe_mask = 1 << oe_shift;

  /* clear oe bit */
  config_value &= ~oe_mask;

  /* assign oe bit */
  config_value |= (new_status << oe_shift);

  BOS_LOGI(TAG, "Set bos1901 dev {%s} output {%d}", dev_name, config_value);

  ret = bos1901_device_write_reg(dev, BOS1901_REG_CONFIG, config_value);

  if (ret != BOS1901_OK)
  {
    BOS_LOGE(TAG, "Set bos1901 dev {%s} bos1901_device_output_enable failed!",
             dev_name);
  }
  else
  {
    BOS_LOGW(TAG, "Set bos1901 dev {%s} output {%s}", dev_name,
             (new_status) ? "ENABLE" : "DISABLE");
  }
  return ret;
}

bos1901_err_t bos1901_device_reset(bos1901_dev_t *dev)
{
  const char *dev_name = (dev->dev_name != NULL) ? dev->dev_name : "UNKNOWN";

  uint16_t reset_cmd = 0x5020;  // or input 0x0020 is ok too
  bos1901_err_t ret =
      bos1901_device_write_reg(dev, BOS1901_REG_CONFIG, reset_cmd);

  if (ret == BOS1901_OK)
  {
    /* clear private status */
    /* TODO: v1, write a function to reset the struct */
    bos1901_private_status_t p = {0};
    dev->private_status = p;

    BOS_LOGI(TAG, "BOS1901 dev {%s} reset ok!", dev_name);
  }
  else
  {
    BOS_LOGE(TAG, "BOS1901 dev {%s} reset failed with err status {%d}!",
             dev_name, ret);
  }

  return ret;
}

bos1901_err_t bos1901_device_sense_enable(bos1901_dev_t *dev,
                                          bos1901_sense_t new_status)
{
  const char *dev_name = (dev->dev_name != NULL) ? dev->dev_name : "UNKNOWN";

  /* check status is in range */
  if (new_status >= BOS1901_SENSE_END)
  {
    BOS_LOGE(TAG,
             "ERR: new_status {%d} (bos1901_sense_status_t) in "
             "`bos1901_device_sense_enable` out of range",
             new_status);
    return BOS1901_INVALID_ARG;
  }

  /* update private status */
  dev->private_status.sense_enable = new_status;

  /* read reg */
  uint16_t sup_rise_value;
  bos1901_err_t ret =
      bos1901_device_read_reg(dev, BOS1901_REG_SUP_RISE, &sup_rise_value);

  if (ret != BOS1901_OK)
  {
    BOS_LOGE(TAG,
             "Execute `bos1901_device_sense_enable` (read stage) failed. Error "
             "Code: {%d}",
             ret);
    return ret;
  }

  /* assign field */
  const int sense_shift = 11;
  uint16_t sense_mask = 1 << sense_shift;

  /* clear oe bit */
  sup_rise_value &= ~sense_mask;

  /* assign oe bit */
  sup_rise_value |= (new_status << sense_shift);

  ret = bos1901_device_write_reg(dev, BOS1901_REG_SUP_RISE, sup_rise_value);

  if (ret != BOS1901_OK)
  {
    BOS_LOGE(TAG, "Set bos1901 dev {%s} bos1901_device_sense_enable failed!",
             dev_name);
  }
  else
  {
    BOS_LOGW(TAG, "Set bos1901 dev {%s} sense {%s}", dev_name,
             (new_status) ? "ENABLE" : "DISABLE");
  }
  return ret;
}

/* private function definition */

static void bos1901_update_spi_interface(
    bos1901_spi_interface_t *existing_interface,
    bos1901_spi_interface_t new_interface)
{
  if (new_interface.name != NULL)
  {
    existing_interface->name = new_interface.name;
  }
  if (new_interface.description != NULL)
  {
    existing_interface->description = new_interface.description;
  }
  if (new_interface.init_device != NULL)
  {
    existing_interface->init_device = new_interface.init_device;
  }
  if (new_interface.deinit_device != NULL)
  {
    existing_interface->deinit_device = new_interface.deinit_device;
  }
  if (new_interface.read_write_word != NULL)
  {
    existing_interface->read_write_word = new_interface.read_write_word;
  }
  if (new_interface.read_write != NULL)
  {
    existing_interface->read_write = new_interface.read_write;
  }
}

static void bos1901_device_config(bos1901_dev_t *dev,
                                  bos1901_dev_config_t *dev_config)
{
  /* copy user dev_config to dev->dev_config */
  BOS_LOGI(TAG, "Copy user's device config to dev->dev_config");

  if (dev_config != NULL)
  {
    *(dev->dev_config) = *dev_config;
  }
  else
  {
    ESP_LOGW(TAG,
             "No device config provided by user! Using default device config");
    *(dev->dev_config) = bos1901_get_device_default_config();
  }

  /* TODO: v1 start to config device (read/write bos1901 registers) */
  BOS_LOGI(TAG, "Start to config device through spi bus");
  BOS_LOGW(TAG, "TODO...");
}

static bos1901_spi_interface_t *create_spi_interface()
{
  bos1901_spi_interface_t *spi_interface =
      (bos1901_spi_interface_t *)malloc(sizeof(bos1901_spi_interface_t));

  if (spi_interface == NULL)
  {
    BOS_LOGE(TAG, "Unable to allocate memory for spi_interface_ptr.");
    return NULL;
  }

  *spi_interface = bos1901_get_default_spi_interface();

  const char *interface_name =
      (spi_interface->name != NULL) ? spi_interface->name : "UNKNOWN";
  BOS_LOGI(TAG, "Using spi interface: {%s}", interface_name);

  return spi_interface;
}

// SPI device functions
static void bos1901_spi_device_init(bos1901_dev_t *dev,
                                    bos1901_spi_dev_config_t *device_config)
{
  bos1901_spi_dev_t *spi_dev = dev->spi_dev;

  /* malloc device_config */
  if (spi_dev->device_config == NULL)
  {
    spi_dev->device_config =
        (bos1901_spi_dev_config_t *)malloc(sizeof(bos1901_spi_dev_config_t));
    if (spi_dev->device_config == NULL)
    {
      BOS_LOGE(TAG, "Failed to create the SPI device config.");
      return;
    }

    BOS_LOGI(TAG, "Malloc bos1901_spi_dev_config_t");
  }

  /* FIXME: v2 Currently, request handle defined by user */
  *(spi_dev->device_config) = *(device_config);

  if (spi_dev->spi_interface_ptr == NULL)
  {
    /* malloc spi interface */
    spi_dev->spi_interface_ptr = create_spi_interface();
    if (spi_dev->spi_interface_ptr == NULL)
    {
      if (dev->dev_name != NULL)
      {
        BOS_LOGE(TAG, "In dev {%s}", dev->dev_name);
      }
      BOS_LOGE(TAG, "Failed to create the SPI interface.");
      return;
    }
  }
  spi_dev->spi_interface_ptr->init_device(device_config);
}

static void bos1901_spi_device_deinit(bos1901_dev_t *dev)
{
  bos1901_spi_dev_t *spi_dev = dev->spi_dev;

  /* free bos1901_spi_interface_t */
  if (spi_dev->spi_interface_ptr)
  {
    spi_dev->spi_interface_ptr->deinit_device(spi_dev->device_config);
    free(spi_dev->spi_interface_ptr);
    spi_dev->spi_interface_ptr = NULL;
  }

  /* free bos1901_spi_dev_config_t */
  if (spi_dev->device_config)
  {
    free(spi_dev->device_config);
    spi_dev->device_config = NULL;
  }
}

static bos1901_err_t bos1901_spi_read_write_reg(bos1901_dev_t *dev,
                                                uint16_t data_tx,
                                                uint16_t *data_rx)
{
  bos1901_spi_dev_t *spi_dev = dev->spi_dev;
  return spi_dev->spi_interface_ptr->read_write_word(spi_dev->device_config,
                                                     data_tx, data_rx);
}

static void bos1901_change_spi_interface(bos1901_dev_t *dev,
                                         bos1901_spi_interface_t new_interface)
{
  bos1901_spi_dev_t *spi_dev = dev->spi_dev;

  if (spi_dev->spi_interface_ptr)
  {
    // Deinitialize the current interface before updating
    spi_dev->spi_interface_ptr->deinit_device(spi_dev->device_config);

    const char *prev_interface_name = (spi_dev->spi_interface_ptr->name != NULL)
                                          ? spi_dev->spi_interface_ptr->name
                                          : "UNKNOWN";
    const char *dev_name = (dev->dev_name != NULL) ? dev->dev_name : "UNKNOWN";

    // Update only the non-NULL parts of the new interface
    bos1901_update_spi_interface(spi_dev->spi_interface_ptr, new_interface);

    const char *new_interface_name =
        (new_interface.name != NULL) ? new_interface.name : "PARTIALLY UPDATED";

    BOS_LOGI(TAG,
             "SPI interface updated from {%s} to {%s} for dev named: {%s}.",
             prev_interface_name, new_interface_name, dev_name);

    // Initialize the updated interface
    spi_dev->spi_interface_ptr->init_device(spi_dev->device_config);
  }
}

static bos1901_err_t bos1901_spi_device_read_write(bos1901_dev_t *dev,
                                                   uint16_t *data_tx,
                                                   uint16_t *data_rx,
                                                   uint32_t length)
{
  /* first byte in rx might be dummy byte */
  return dev->spi_dev->spi_interface_ptr->read_write(
      dev->spi_dev->device_config, data_tx, data_rx, length);
}
