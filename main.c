#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/binary_info.h"
#include "hardware/timer.h"
#include "pico/bootrom.h"

#include "main.h"


/*
***************
** CONSTANTS **
***************
*/
const uint WATCHDOG_TIMEOUT_MS = 5 * 1000; // Time in milliseconds the watchdog waits for a response before rebooting the device.

const uint AWAIT_USB_CONNECTION_INTERVAL_MS = 1000;     // Interval inbetween checks to wait for a USB connection (using PuTTY, for example).

const uint USB_READ_TIMEOUT_US = 1 * 1000; // Time in microseconds to wait when reading from USB, with each request.

const uint LED_PIN = 25;
const uint ADC0_PIN = 26;

const uint DOORBELL_ADC_PIN = ADC0_PIN; // The pin of the ADC connected to the doorbell circuit.
const uint MIN_DOORBELL_VOLTAGE = 2.0f; // Minimum voltage required to register a ringing doorbell.

#define ESP8266_UART uart0                  // The UART to use for the ESP8266 module
const int ESP8266_UART_TX_PIN = 0;          // GPIO TX PIN for the ESP8266 module
const int ESP8266_UART_RX_PIN = 1;          // GPIO RX PIN for the ESP8266 module
const int ESP8266_UART_SPEED = 115200;      // The transmission speed (baudrate) for the ESP8266 module
const int ESP8266_UART_READ_TIMEOUT_US = 1 * 1000;    // Time in microseconds to wait when reading from ESP8266 UART

const char* NEW_COMMAND_PROMPT_TEXT = "\n>> ";  /* Text that shows when prompting the user for the next command */
const uint32_t COMMAND_RESPONSE_DELAY_US = 1 * 1000 * 1000; /* Time to wait after the last ESP8266 response to execute the next command in the command queue */

const char *TCP_SERVER_IP = "0.0.0.0";
const char *TCP_SERVER_PORT = "1234";


/*
**************
** COMMANDS **
**************
*/
const char *CMD_PRINT_HELP = "help";    // Prints help text.
const char *CMD_PRINT_ADC0 = "adc0";    // Prints the current readout value from the ADC0 pin
const char *CMD_REBOOT_USB = "stop";    // Command to reboot the device in USB Mass Storage mode
const char *CMD_SEND_TCP_DATA = "tcp";  // Sends TCP data to server (format: tcp <data>)

const char const *CMD_PRINT_HELP_TEXT[] = {
    "List of available commands:\n"
    "   help\t\t Prints this help.\n"
    "   AT  \t\t Begins a new AT command and sends it to the ESP8266. Refer to the ESP8266 documentation for available AT commands.\n"
    "   adc0\t\t Prints the voltage currently readable on the ADC0 pin.\n"
    "   tcp \t\t Test command: Sends the current ADC value to the TCP server.\n"
    "   stop\t\t Stops the program and reboots Pi Pico in USB mass storage mode.\n"
};


/*
**********************
** GLOBAL VARIABLES **
**********************
*/
static char *input_str_buffer = NULL;
static uint input_str_buffer_len = 0;

static command_queue cmd_queue;

static uint64_t response_timestamp_us = 0; /* Timestamp of the last response from the ESP8266 in microseconds */


/*
**********************
** HARDWARE CONTROL **
**********************
*/

/* Component: On-board LED */
void init_led() {    // Initialises the on-board LED.
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
}


void set_led(int state) {
    gpio_put(LED_PIN, state);
}


void led_on() {
    set_led(1);
}


void led_off() {
    set_led(0);
}


/* Component: ADC */
void init_adc(int pin) { // Initialises an analogue-to-digital-converter (ADC) input to measure voltage.
    adc_init();
    adc_gpio_init(pin);
    adc_select_input(0);
}


/*! \brief Returns voltage on ADC.
 * 
 * Gets the voltage measured by the ADC, up to a maximum of 3.3 V.
 * 
 * \return The measurement in Volt.
 */
float get_voltage_on_adc() {  // 
    const float conversion_factor = 3.3f / (1 << 12);
    return adc_read() * conversion_factor;
}


/* Component: UART */
void init_uart(uart_inst_t *uart, int tx_pin, int rx_pin, int speed) {
    uart_init(uart, speed);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
}


/*
*******************************
** EXTERNAL HARDWARE CONTROL **
*******************************
*/

/* External Component: ESP8266-01S */


/*! \brief  Write raw bytes to the ESP8266's UART.
 *
 * This function will block until all the data has been sent to the UART.
 *
 * \param data The bytes to send
 * \param len The number of bytes to send
 */
void send_bytes_to_esp8266(const u_int8_t* data, size_t len) {
    uart_write_blocking(ESP8266_UART, data, len);
}


/*! \brief Sends null terminated string to ESP8266.
 * 
 * Sends null terminated string to ESP8266 over UART. Primarily used to send AT messages.
 * 
 * \param msg The null terminated string to send.
 */
int send_string_to_esp8266(const char* msg) {
    if (uart_is_writable(ESP8266_UART)) {
        uart_write_blocking(ESP8266_UART, msg, strlen(msg));
        uart_write_blocking(ESP8266_UART, "\r\n", 2);
        return PICO_OK;
    } else {
        return PICO_ERROR_IO;
    }
}


/*! \brief Reads bytes from the ESP8266. 
 *
 * Reads bytes from the ESP8266 over UART. 
 * 
 * \return Any bytes read or NULL if no bytes were read.
 */
u_int8_t* read_esp8266_uart() {
    const size_t initial_buffer_size = 2;

    bool bytes_readable = uart_is_readable_within_us(ESP8266_UART, ESP8266_UART_READ_TIMEOUT_US);    // bytes readable from UART
    uint total_bytes = 0;  // final number of bytes

    if (!bytes_readable) return NULL;

    u_int8_t *response = calloc(1, initial_buffer_size);
    size_t current_buffer_size = initial_buffer_size;

    do {
        while (total_bytes >= current_buffer_size)  // Dynamically adjust buffer size
        {
            current_buffer_size = 2 * current_buffer_size;
            response = realloc(response, current_buffer_size);
        }

        uart_read_blocking(ESP8266_UART, response + total_bytes, bytes_readable);   // Read from UART to buffer
        total_bytes += bytes_readable;

        bytes_readable = uart_is_readable_within_us(ESP8266_UART, ESP8266_UART_READ_TIMEOUT_US);
    } while (bytes_readable);

    if (total_bytes) {        
        response = realloc(response, total_bytes + 1);  // Trim buffer size to minimum
        response[total_bytes] = '\0';
    } else {    // No bytes read.
        free(response);
        response = NULL;
    }

    return response;
}


/*
*************
** HELPERS **
*************
*/
int is_doorbell_ringing(float min_volt) {    // Returns whether the doorbell is ringing at this moment (if the voltage across the ADC is equal to or larger than min_volt)
    return get_voltage_on_adc() >= min_volt;
}


/*! \brief Returns true if char is normally printable.
 * 
 * Checks if ascii character is between 32 (space) and 126 (tilde), e.g. whether it should be printed.
 * 
 * \param c The char to check.
 * 
 * \return True if character is valid for printing, else false.
 */
bool is_valid_char(char c) {
    return c >= ' ' && c <= '~';
}


/*! \brief Returns true if char is the backspace key character.
 * 
 * Checks if ascii character is 127 (DEL), the character produced by pressing the "backspace" key on the keyboard.
 * Not to be confused with the char '\b', which just moves the cursor back one character without deleting anything.
 * 
 * \param c The char to check.
 * 
 * \return True if character is 127 (DEL), else false.
 */
bool is_backspace_key(char c) {
    return c == 127;
}


/*! \brief Reads string from USB.
 * 
 * Attempts to read string from input, blocking until '\r' is read.
 * 
 * \return The string read.
 */
char* read_string_from_input() {
    uint32_t input_c;
    size_t buffer_size = 2;

    do {  
        input_c = getchar_timeout_us(USB_READ_TIMEOUT_US);  /* Read input until timeout */

        /* Input Timeout */
        if (input_c == PICO_ERROR_TIMEOUT) break;
        
        /* Input is valid character */
        if (is_valid_char((char) input_c)) { 
            /* Set up memory for new message */
            if (input_str_buffer == NULL) {
                input_str_buffer = calloc(buffer_size, sizeof(char));
                input_str_buffer_len = 0;
            }
            /* Dynamically adjust buffer size */
            if (input_str_buffer_len % 4 == 0) { 
                buffer_size = input_str_buffer_len + 4;
                input_str_buffer = realloc(input_str_buffer, buffer_size);                
            }

            input_str_buffer[input_str_buffer_len++] = (char) input_c;
            printf("%c", (char) input_c);
            continue;
        }

        /* The following operations only work on non-empty strings */
        if (input_str_buffer == NULL) break;

        /* Backspace */
        if (is_backspace_key((char) input_c) && input_str_buffer_len > 0) { 
            input_str_buffer[--input_str_buffer_len] = '\0';
            printf("%c", (char) input_c);
        } 

    } while(input_c != '\r'); /* Message ready to be sent */

    if (input_c == '\r' && input_str_buffer != NULL)
    {
        /* Prepare return string */
        char *return_str = calloc(input_str_buffer_len + 1, sizeof(char)); 
        for (size_t i = 0; i < input_str_buffer_len; i++)
        {
            return_str[i] = input_str_buffer[i];
        }
        
        return_str[input_str_buffer_len] = '\0';
        
        /* Reset input buffer */
        free(input_str_buffer);
        input_str_buffer = NULL;
        input_str_buffer_len = 0;

        printf("\n");

        return return_str;
    } else {
        return NULL;
    } 
}


bool is_cmd_print_help(const char *cmd) { 
    return strcmp(cmd, CMD_PRINT_HELP) == 0;
}


int print_cmd_help() {
        printf("%s\n", *CMD_PRINT_HELP_TEXT);
        return PICO_OK;
}


bool is_cmd_AT(const char *cmd) {
    return cmd[0] == 'A' && cmd[1] == 'T';
}


int send_AT_cmd(const char *cmd) {
    int esp8266_comm_error = send_string_to_esp8266(cmd);
    return esp8266_comm_error;
}


bool is_cmd_print_adc0(const char *cmd) {
    return strcmp(cmd, CMD_PRINT_ADC0) == 0;
}


int print_adc0_voltage() {
    printf("ADC0 Voltage: %f V\n", get_voltage_on_adc());
    return PICO_OK;
}


bool is_cmd_reboot_usb(const char *cmd) {
    return strcmp(cmd, CMD_REBOOT_USB) == 0;
}


int reboot_usb() {
    printf("Rebooting device in USB mass storage mode...\n");
    reset_usb_boot(0, 0);
}


bool is_cmd_send_tcp_data(const char *cmd) {
    return strcmp(cmd, CMD_SEND_TCP_DATA) == 0;
}


int invalid_cmd() {
    printf("Invalid command. Type 'help' for help.\n");
    return PICO_ERROR_GENERIC;
}


void clear_new_message_prompt() {
    uint prompt_len = strlen(NEW_COMMAND_PROMPT_TEXT);
    uint msg_len = input_str_buffer_len;
    for (size_t i = 0; i < (prompt_len + msg_len); i++)
    {
        printf("%c", 127);  /* Backspace */
    }
}


void reprint_new_msg_prompt() {
    printf("%s", NEW_COMMAND_PROMPT_TEXT);
    for (size_t i = 0; i < input_str_buffer_len; i++)
    {
        printf("%c", input_str_buffer[i]);
    }    
}


int send_tcp_data() {
    char *buffer = calloc(64, sizeof(char));
    char *data = calloc(64, sizeof(char));
    snprintf(data, 64, "adc=%f", get_voltage_on_adc());
    
    snprintf(buffer, 64, "AT+CIPSTART=\"TCP\",\"%s\",%s", TCP_SERVER_IP, TCP_SERVER_PORT);
    send_string_to_esp8266(buffer);

    sleep_ms(100);

    snprintf(buffer, 64, "AT+CIPSEND=%d", strlen(data));
    send_string_to_esp8266(buffer);

    sleep_ms(100);

    send_string_to_esp8266(data);

    free(data);
    free(buffer);
    return 0;
}


int can_process_command() {
    return time_us_64() > (response_timestamp_us + COMMAND_RESPONSE_DELAY_US);
}


char* process_next_command() {
    if (!can_process_command()) return NULL;

    char *cmd = get_next_command_from_queue(&cmd_queue);
    if (cmd == NULL) return NULL;
    printf("processing cmd: %s\n", cmd);

    /* ESP8266 AT Command */
    if (is_cmd_AT(cmd)) 
        send_AT_cmd(cmd);

    /* Command: TCP test */
    else if (is_cmd_send_tcp_data(cmd))
        send_tcp_data();

    /* Command: Reboot in mass storage mode */
    else if (is_cmd_reboot_usb(cmd))
        reboot_usb();

    else if (is_cmd_print_adc0(cmd)) 
        print_adc0_voltage();
    
    /* Command: Print help */
    else if (is_cmd_print_help(cmd))
        print_cmd_help();

    /* Invalid command. */
    else 
        invalid_cmd();

    return cmd;
}


/*
**********
** MAIN **
**********
*/
int main() {
    bi_decl(bi_program_description("This program sends a message to another device when voltage is registered on the ADC 0 Pin."))
    bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"))
    bi_decl(bi_1pin_with_name(ADC0_PIN, "ADC 0 Pin"))

    stdio_init_all();

    /* Initialise hardware */
    init_led();
    init_adc(DOORBELL_ADC_PIN); /* Init ADC to read doorbell voltage */
    init_uart(ESP8266_UART, ESP8266_UART_TX_PIN, ESP8266_UART_RX_PIN, ESP8266_UART_SPEED);  /* Init UART communication with ESP8266 */

    led_on();   // debug indicator just showing that the pi is on for now.

    int prompt_new_msg = 1;

    /* Wait for USB connection, so we don't waste any output */
    while (!stdio_usb_connected()) {
        sleep_ms(AWAIT_USB_CONNECTION_INTERVAL_MS);
    }

    /* Enable the Watchdog */
    if (watchdog_enable_caused_reboot()) {
        printf("Watchdog caused reboot, did something crash?\n");
        reboot_usb();
    }
    watchdog_enable(WATCHDOG_TIMEOUT_MS, 1);
    

    while (1) {
        watchdog_update();        

        /* Prompt for new message if we haven't already. */
        if (prompt_new_msg)
        {
            printf("%s", NEW_COMMAND_PROMPT_TEXT);
            prompt_new_msg = 0;
        }

        /* Read input from USB */
        char *msg = read_string_from_input();

        /* Process input command */
        if (msg != NULL)
        {
            add_command_to_queue(msg, &cmd_queue);
        }

        char *processed_cmd = process_next_command();

        /* Read ESP8266 */
        char *esp8266_response = (char*) read_esp8266_uart();
        if (esp8266_response != NULL) {
            if (!prompt_new_msg && msg == NULL)
                clear_new_message_prompt();
            
            printf("%s", esp8266_response); 
            response_timestamp_us = time_us_64();

            if (!prompt_new_msg && msg == NULL)
                reprint_new_msg_prompt();

        }
        
        /* Cleanup */
        if (msg != NULL) {
            prompt_new_msg = 1;
        }
        if (processed_cmd != NULL) free(processed_cmd);
        if (esp8266_response != NULL) free(esp8266_response);
    }
}