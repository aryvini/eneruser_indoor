#ifndef UTILS_H
#define UTILS_H

void sensors_start();
bool BME_start();
bool Specs_start();
bool SPS_start();
bool DS18_start();


void SD_start();
void OLED_start();
void ESP_start();
void wifi_connect();



bool SPS_request_data();
bool DS18_request_data();
void read_sensors();
void zero_variables();
void zero_averages();
float leaky_filter(float previous, float current);
void SPS_set_autoclean(uint32_t new_auto_clean_interval);
void SPS_clean_now();



void OLED_readings_print();
void Serial_readings_print();
void OLED_msg(const __FlashStringHelper * msg, int duration);

void SD_save_tabular_style();
void cloud_send_data();
void UDP_send_data(String variable_name, float value, int decimals=2);
void UDP_send_all();
bool get_NTP_time();
void adjust_RTC();
void ping_test();

void blink(uint8_t type=0);

void print_millis(uint8_t checkpoint=1);

void UDP_all_variables_string(int decimals=2);


#endif