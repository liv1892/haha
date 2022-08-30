#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

esp_err_t ble_write(uint8_t *value,uint16_t value_len);
void gattc_multi_connect(void);

#endif
