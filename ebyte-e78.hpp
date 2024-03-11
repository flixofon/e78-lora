#pragma once
#include <string>
#include <cstring>
#include <functional>
#include <iostream>
#include <thread>
#include <sstream>
#include <iomanip>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define buffer_size 1024
#define ESP_LOGGING_TAG "E78LORA"
#define DEVICE_ALREADY_JOINED "Device already joined the network"
#define DEVICE_NOT_JOINED "Device not joined the network"
#define MAX_RESP_CHECK 250

class E78LORAWAN{
    private:
        int uart_num;
        bool joined;
        std::vector<std::string> responses;
        TaskHandle_t receive_data_task;
        std::function<void(const std::string&)> downstream_callback;
    public:
        enum data_rate{
            SF12 = 0,
            SF11 = 1,
            SF10 = 2,
            SF9 = 3,
            SF8 = 4,
            SF7 = 5
        };

        enum transmit_power{
            TX_17_DBM = 0,
            TX_15_DBM = 1,
            TX_13_DBM = 2,
            TX_11_DBM = 3,
            TX_09_DBM = 4,
            TX_07_DBM = 5,
            TX_05_DBM = 6,
            TX_03_DBM = 7
        };

        enum device_status{
            NO_DATA_OPERATION = 00,
            DATA_SENT = 01,
            DATA_DELIVERY_FAILED = 02,
            DATA_SENT_SUCCESSFULLY = 03,
            JOIN_SUCCESSFUL = 04,
            JOIN_FAILED = 05,
            NETWORK_ISSUE_DETECTED = 06,
            SEND_DATA_SUCCESSFULLY_NO_ACK = 07,
            SEND_DATA_SUCCESSFULLY_ACK = 8,
        };

        enum lorawan_classes{
            CLASS_A = 0,
            CLASS_C = 2
        };

        ~E78LORAWAN(){
            ESP_ERROR_CHECK(uart_driver_delete(this->uart_num));
        }

        int configure(int p_uart_num, int tx_pin, int rx_pin){
            if((tx_pin < 0 || rx_pin < 0) || (tx_pin > 34 || rx_pin > 34)){
                ESP_LOGE(ESP_LOGGING_TAG, "Invalid pin number");
                return -1;
            }
            this->joined = false;
            this->uart_num = p_uart_num;
            uart_config_t uart_config = {
                    .baud_rate = 9600,
                    .data_bits = UART_DATA_8_BITS,
                    .parity = UART_PARITY_DISABLE,
                    .stop_bits = UART_STOP_BITS_1,
                    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                    .source_clk = UART_SCLK_DEFAULT,
            };
            ESP_ERROR_CHECK(uart_driver_install(uart_num, buffer_size, buffer_size, 10, nullptr, 0));
            ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
            ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin,rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
            if(xTaskCreate(receive_data, "receive_data", 4096, this, 1, &this->receive_data_task) != pdPASS){
                ESP_LOGE(ESP_LOGGING_TAG, "Failed to create receive data task");
                return -1;
            }
            return 0;
        }

        int join_network(){
            if(this->joined){
                ESP_LOGE(ESP_LOGGING_TAG, "Device already joined to network");
                return -1;
            }
            this->write_to_uart("AT+CJOIN=1,0,8,8\r\n");
            while(1){
                if(this->answer_received("+CJOIN:OK", true) == 0){
                    ESP_LOGI(ESP_LOGGING_TAG, "Network join successful");
                    this->responses.clear();
                    this->joined = true;
                    return 0;
                } else if(this->answer_received("+CJOIN:FAIL", true) == 0){
                    ESP_LOGE(ESP_LOGGING_TAG, "Network join failed");
                    return -1;
                }
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
        }

        static void receive_data(void *parameters){
            auto* instance = static_cast<E78LORAWAN*>(parameters);
            while(1){
                std::size_t length = 0;
                ESP_ERROR_CHECK(uart_get_buffered_data_len(instance->uart_num, (size_t*)&length));
                if(length > 0){
                    char* data = (char*)malloc(length+1);
                    length = uart_read_bytes(instance->uart_num, data, length, 100);
                    data[length] = '\0';
                    std::string str(data);
                    str.erase(std::remove(str.begin(), str.end(), '\n'), str.end());
                    str.erase(std::remove(str.begin(), str.end(), '\r'), str.end());
                    instance->responses.push_back(str);
                    if(str.find("+DRX:") != std::string::npos) instance->downstream_callback(str.substr(5, str.length()-5));
                    free(data);
                }
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
        }

        std::int8_t send_data(const std::string& data){
            if(!this->joined){
                ESP_LOGE(ESP_LOGGING_TAG, DEVICE_NOT_JOINED);
                return -1;
            }
            std::string hex_data = E78LORAWAN::a_to_h(data);
            std::string to_send = "AT+DTRX=1,3," + std::to_string(hex_data.length()) + "," + hex_data + "\r\n";
            this->write_to_uart(to_send);
            if(this->answer_received("OK+RECV:02", true) == 0){
                ESP_LOGI(ESP_LOGGING_TAG, "Data sent successfully");
                return 0;
            } else {
                ESP_LOGE(ESP_LOGGING_TAG, "Data delivery failed");
                return -1;
            }
        }

        device_status get_device_status(){
            this->write_to_uart("AT+CSTATUS?\r\n");
            if(this->answer_received("+CSTATUS:", false) != 0){
                ESP_LOGE(ESP_LOGGING_TAG, "Device status not received");
                return NO_DATA_OPERATION;
            } else {
                std::string resp = this->get_latest_response();
                this->remove_response_from_vec(resp);
                return (device_status)std::stoi(resp.substr(9, 2));
            }
        }

        std::string get_serial_number(){
            this->write_to_uart("AT+CGSN?\r\n");
            if(this->answer_received("+CGSN=", false) != 0){
                ESP_LOGE(ESP_LOGGING_TAG, "Serial number not received");
                return "";
            }
            std::string resp = this->get_latest_response();
            this->remove_response_from_vec(resp);
            return resp.substr(6, resp.length() - 4);
        }

        std::uint8_t set_downstream_callback(const std::function<void(const std::string&)>& func){
            if(this->joined) {
                ESP_LOGE(ESP_LOGGING_TAG, DEVICE_ALREADY_JOINED);
                return -1;
            } else if(func == nullptr){
                ESP_LOGE(ESP_LOGGING_TAG, "Invalid callback function");
                return -1;
            } else this->downstream_callback = func;
            return 0;
        }

        void set_upstream_data_port(std::uint8_t port) const{
            if(this->joined)  ESP_LOGE(ESP_LOGGING_TAG, DEVICE_ALREADY_JOINED);
            else if(port > 223) ESP_LOGE(ESP_LOGGING_TAG, "Invalid port number");
            this->write_to_uart("AT+CAPPPORT=" + std::to_string(port) + "\r\n");
        }

        std::int8_t set_data_rate(data_rate data_rate) const{
            if(this->joined){
                ESP_LOGE(ESP_LOGGING_TAG, DEVICE_ALREADY_JOINED);
                return -1;
            }
            switch(data_rate) {
                case 0 ... 5:
                    this->write_to_uart("AT+CDATARATE=" + std::to_string(data_rate) + "\r\n");
                    break;
                default:
                    ESP_LOGE(ESP_LOGGING_TAG, "Invalid data rate");
                    return -1;
            }
            return 0;
        }

        std::int8_t save(){
            if(this->joined){
                ESP_LOGE(ESP_LOGGING_TAG, DEVICE_ALREADY_JOINED);
                return -1;
            }
            this->write_to_uart("AT+CSAVE\r\n");
            this->write_to_uart("AT+IREBOOT=0\r\n");
            vTaskSuspend(this->receive_data_task);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            vTaskResume(this->receive_data_task);
            return 0;
        }

        std::int8_t set_class(lorawan_classes class_type) const {
            if (this->joined) {
                ESP_LOGE(ESP_LOGGING_TAG, DEVICE_ALREADY_JOINED);
                return -1;
            }

            switch (class_type) {
                case CLASS_A:
                case CLASS_C:
                    return this->write_to_uart("AT+CCLASS=" + std::to_string(class_type) + "\r\n");
                default:
                    ESP_LOGE(ESP_LOGGING_TAG, "Invalid class type");
                    return -1;
            }
        }

        std::int8_t set_tx_power(transmit_power power) const{
            if(this->joined){
                ESP_LOGE(ESP_LOGGING_TAG, DEVICE_ALREADY_JOINED);
                return -1;
            }
            switch(power) {
                case 0 ... 7:
                    this->write_to_uart("AT+CTXP=" + std::to_string(power) + "\r\n");
                    break;
                default:
                    ESP_LOGE(ESP_LOGGING_TAG, "Invalid power level");
                    return -1;
            }
            return 0;
        }

        void set_number_of_trials(std::uint8_t trials) const{
            if(this->joined){
                ESP_LOGE(ESP_LOGGING_TAG, DEVICE_ALREADY_JOINED);
            } else if(trials > 15){
                ESP_LOGE(ESP_LOGGING_TAG, "Invalid number of trials");
            }
            this->write_to_uart("AT+CFREQTRIALS=1," + std::to_string(trials) + "\r\n");
        }

        int set_app_eui(const std::string& app_eui) const{
            if(app_eui.length() != 16){
                ESP_LOGE(ESP_LOGGING_TAG, "Invalid app eui");
                return -1;
            } else if(!this->joined)
                write_to_uart("AT+CAPPEUI=" + app_eui + "\r\n");
            else{
                ESP_LOGE(ESP_LOGGING_TAG, DEVICE_ALREADY_JOINED);
                return -1;
            }
            return 0;
        }

        std::int8_t set_app_key(const std::string& app_key) const{
            if(app_key.length() != 32){
                ESP_LOGE(ESP_LOGGING_TAG, "Invalid app key");
                return -1;
            } else if(!this->joined)
                write_to_uart("AT+CAPPKEY=" + app_key + "\r\n");
            else{
                ESP_LOGE(ESP_LOGGING_TAG, DEVICE_ALREADY_JOINED);
                return -1;
            }
            return 0;
        }

        std::int8_t set_confirmation_enable(bool enable) const{
            if(this->joined){
                ESP_LOGE(ESP_LOGGING_TAG, DEVICE_ALREADY_JOINED);
                return -1;
            }
            this->write_to_uart("AT+CCONFIRM=" + std::to_string(enable) + "\r\n");
            return 0;
        }

        std::int8_t set_dev_eui(const std::string& dev_eui) const{
            if(dev_eui.length() != 16){
                ESP_LOGE(ESP_LOGGING_TAG, "Invalid dev eui");
                return -1;
            } else if(!this->joined) write_to_uart("AT+CDEVEUI=" + dev_eui + "\r\n");
            else{
                ESP_LOGE(ESP_LOGGING_TAG, DEVICE_ALREADY_JOINED);
                return -1;
            }
            return 0;
        }

        std::int8_t write_to_uart(const std::string& data) const{
            uart_write_bytes(this->uart_num, data.c_str(), data.length());
            vTaskDelay(100 / portTICK_PERIOD_MS);
            return 0;
        }

        std::int8_t answer_received(const std::string& required, bool remove_from_vec = false) {
            constexpr int maxAttempts = 250;
            for (int i = 0; i < maxAttempts; ++i) {
                auto it = std::find_if(responses.begin(), responses.end(),
                                       [&](const std::string& response) { return response.find(required) != std::string::npos; });
                if (it != responses.end()) {
                    if (remove_from_vec) responses.erase(it);
                    return 0;
                }
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            return -1;
        }

        std::string get_latest_response(){
            return this->responses.back();
        }

        void remove_response_from_vec(const std::string& response){
            this->responses.erase(std::remove(this->responses.begin(),this->responses.end(),response),this->responses.end());
        }

        static std::string a_to_h(const std::string& asciiString) {
            std::stringstream hex;
            for (char c : asciiString) {
                hex << std::hex << std::setw(2) << std::setfill('0') << (int) c;
            }
            return hex.str();
        }
};