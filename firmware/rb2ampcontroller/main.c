/*
 *
 */

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "lwip/dhcp.h"
#include "lwip/init.h"
#include "lwip/apps/httpd.h"
#include "rmii_ethernet/netif.h"
#include "lwip/ip4_addr.h"
#include "lwip/tcp.h"
#include "lwip/api.h"
#include "string.h"

#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

#define VERSION  0x01

const uint LED_PIN = PICO_DEFAULT_LED_PIN;
const uint BIAS_PIN = 16U;
const uint RELAIS_PIN = 17U;
const uint PWM_PIN = 1;

const uint PWM_FREQ = 30000; //3KHz

#define TCP_PORT 4242
#define DEBUG_printf printf
#define BUF_SIZE 20
#define POLL_TIME_S 5

uint16_t forward_adc_value = 0;
uint16_t reverse_adc_value = 0;

uint16_t pa_temparature = 0;   // temperature in celcius.
bool pa_temp_measure_error = false;

enum PowerState {
    OFF = 0x00,
    STANDBY = 0x0f,
    ACTIVE = 0xff
};

enum PowerState deviceState = OFF;

enum PowerState convertToPowerState(unsigned char stateValue) {
    switch (stateValue) {
        case OFF:
            return OFF;
        case STANDBY:
            return STANDBY;
        case ACTIVE:
            return ACTIVE;
        default:
            return OFF; // Return a default value or handle error
    }
}

enum LPFFilter {
    NO_LPF      = 0x00,
    LPF_160M    = 0x80,
    LPF_80M     = 0x40,
    LPF_60M     = 0x20,
    LPF_40M     = 0x10,
    LPF_20M     = 0x08,
    LPF_10M     = 0x04
};

enum LPFFilter lpfFilter = NO_LPF;

enum LPFFilter convertToLPFFilter(unsigned char filterValue) {
    switch (filterValue) {
        case NO_LPF:
            return NO_LPF;
        case LPF_160M:
            return LPF_160M;
        case LPF_80M:
            return LPF_80M;
        case LPF_60M:
            return LPF_60M;
        case LPF_40M:
            return LPF_40M;
        case LPF_20M:
            return LPF_20M;
        case LPF_10M:
            return LPF_10M;
        default:
            return NO_LPF; // Return a default value or handle error
    }
}

enum PTTState {
    PA_OFF = 0x00,
    PA_ON = 0xFF
};

enum PTTState pttState = PA_OFF;

enum PTTState convertToPTTState(unsigned char pttValue) {
    switch (pttValue) {
        case PA_OFF:
            return PA_OFF;
        case PA_ON:
            if (deviceState == ACTIVE) return PA_ON; else return PA_OFF;
        default:
            return PA_OFF; // Return a default value or handle error
    }
}

enum ActualTxState {
    TX_OFF          = 0x01,
    TX_ON           = 0x10
};

enum ActualTxState actualAmpState = TX_OFF;


void read_adc_values() {
    //read forward power
    adc_select_input(0);
    forward_adc_value = adc_read(); // Read the ADC value
    //DEBUG_printf("Forward ADC value is %d \n", forward_adc_value);

    //read reverse power
    adc_select_input(1);
    reverse_adc_value = adc_read(); // Read the ADC value
    //DEBUG_printf("Reverse ADC value is %d \n", reverse_adc_value);
}

/*
    Define constants for MCP9808

    http://adafru.it/1782

    i2c address 0x18
*/

#define MCP9808_ADDR 0x18
const uint8_t REG_TEMP_CRIT = 0x04;
const uint8_t REG_TEMP_AMB = 0x05;
const uint8_t REG_RESOLUTION = 0x08;

int mcp9808_init(void) {

    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN,  GPIO_FUNC_I2C));
}


float mcp9808_convert_temp(uint8_t upper_byte, uint8_t lower_byte) {

    float temperature;

    //Check if TA <= 0°C and convert to denary accordingly
    if ((upper_byte & 0x10) == 0x10) {
        upper_byte = upper_byte & 0x0F;
        temperature = 256 - (((float) upper_byte * 16) + ((float) lower_byte / 16));
    } else {
        temperature = (((float) upper_byte * 16) + ((float) lower_byte / 16));
    }
    return temperature;
}


bool mcp9808_readtemperature(void) {

    uint8_t buf[2];
    uint16_t upper_byte;
    uint16_t lower_byte;

    float temperature;

    int result = i2c_write_timeout_us(i2c_default, MCP9808_ADDR, &REG_TEMP_AMB, 1, true, 1000);
    if (result == PICO_ERROR_GENERIC) {
        printf("failure in connecting / writing to temperature sensor\n"); 
        return false;
    } 
    if (result == PICO_ERROR_TIMEOUT) {
        printf("timeout in writing to temperature sensor\n"); 
        return false;
    } 

    result = i2c_read_timeout_us(i2c_default, MCP9808_ADDR, buf, 2, false, 1000);
    if (result == PICO_ERROR_GENERIC ) {
        printf("failure in connecting / reading temperature sensor\n"); 
        return false; 
    };
    if (result == PICO_ERROR_TIMEOUT) {
        printf("timeout in reading from temperature sensor\n"); 
        return false;
    } 

    upper_byte = buf[0];
    lower_byte = buf[1];

    //clears flag bits in upper byte
    temperature = mcp9808_convert_temp(upper_byte & 0x1F, lower_byte);
    //printf("PA temperature: %.4f°C\n", temperature);
    pa_temparature = (temperature > 0.0) ? (uint16_t)(temperature + 0.5) : (uint16_t)(temperature - 0.5);
    //printf("PA temperature: %d°C\n", pa_temparature);

    return true;
}


/*  Define constants for MCP23008 I2C bus addresses
    This address is set by pulling the pins A0, A1, A2 up/high (to +3.3V) or down/low (to GND).
    addr  A2,A1,A0 =>   0x20  0, 0, 0
*/

#define MCP0_ADDR 0x20


// Define constants for MCP23008 register numbers.
#define IODIR   0x00
#define GPIO    0x09

// Define a constant for output
#define OUTPUT   0x00
#define ALL_HIGH 0xFF
#define ALL_LOW  0x00
                  
void initOutputs(void) {
  uint8_t buffer[2]; 

  buffer[0] = GPIO;
  buffer[1] = ALL_LOW;

  int result = i2c_write_blocking(i2c1, MCP0_ADDR, buffer, 2, false); 
  DEBUG_printf("Result init I2C IO extender (Ok > 0) = %x\n", result);
}

int mcp_init(void) {

    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(PICO_I2C0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_I2C0_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_I2C0_SDA_PIN);
    gpio_pull_up(PICO_I2C0_SCL_PIN);
    
    // printf("\nI2C Bus Scan for finding connected i2c devices\n");
    // printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    // for (int addr = 0; addr < (1 << 7); ++addr) {
    //   if (addr % 16 == 0) {
    //     printf("%02x ", addr);
    //   }
    //   int ret;
    //   uint8_t rxdata;
    //   ret = i2c_read_blocking(i2c1, addr, &rxdata, 1, false);

    //   printf(ret < 0 ? "." : "@");
    //   printf(addr % 16 == 15 ? "\n" : "  ");
    // }


    // First, we must set the I/O direction to output. Write 0x00 to IODIR register.
    uint8_t buffer[2];
    buffer[0] = IODIR;
    buffer[1] = OUTPUT;
    if (i2c_write_blocking(i2c1, MCP0_ADDR, buffer, 2, false) ==  PICO_ERROR_GENERIC) DEBUG_printf("Error writing to IO extender\n");

    initOutputs();
}

void setIOExtenderLPF(int byte) {
    uint8_t buffer[2];
    buffer[0] = GPIO;
    buffer[1] = byte;
    if (i2c_write_blocking(i2c1, MCP0_ADDR, buffer, 2, false) ==  PICO_ERROR_GENERIC) DEBUG_printf("Error writing to IO extender\n");
}

 void execute_sequencer_switch_pa_on(void) {
    //sequencer
    if (actualAmpState == TX_OFF) {
        sleep_ms(15);
        gpio_put(RELAIS_PIN, 1);
        sleep_ms(15);
        gpio_put(BIAS_PIN, 1); 
        actualAmpState = TX_ON;
    }      
 }

 void execute_sequencer_switch_pa_off(void) {
    //sequencer
    if (actualAmpState == TX_ON) {
        sleep_ms(15);
        gpio_put(BIAS_PIN, 0);
        sleep_ms(15);
        gpio_put(RELAIS_PIN, 0);
        actualAmpState = TX_OFF;
    } 
 }

typedef struct TCP_SERVER_T_ {
    struct tcp_pcb *server_pcb;
    struct tcp_pcb *client_pcb;
    bool complete;
    uint8_t buffer_sent[BUF_SIZE];
    uint8_t buffer_recv[BUF_SIZE];
    int sent_len;
    int recv_len;
    int run_count;
} TCP_SERVER_T;

static TCP_SERVER_T* tcp_server_init(void) {
    TCP_SERVER_T *state = calloc(1, sizeof(TCP_SERVER_T));
    if (!state) {
        DEBUG_printf("failed to allocate state\n");
        return NULL;
    }
    return state;
}

static err_t tcp_server_close(void *arg) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    err_t err = ERR_OK;
    DEBUG_printf("Close TCP connection\n");
    if (state->client_pcb != NULL) {
        tcp_arg(state->client_pcb, NULL);
        //tcp_poll(state->client_pcb, NULL, 0);
        tcp_sent(state->client_pcb, NULL);
        tcp_recv(state->client_pcb, NULL);
        tcp_err(state->client_pcb, NULL);
        err = tcp_close(state->client_pcb);
        if (err != ERR_OK) {
            DEBUG_printf("close failed %d, calling abort\n", err);
            tcp_abort(state->client_pcb);
            err = ERR_ABRT;
        }
        state->client_pcb = NULL;
    }
    if (state->server_pcb) {
        tcp_arg(state->server_pcb, NULL);
        tcp_close(state->server_pcb);
        state->server_pcb = NULL;
    }
    return err;

}

static err_t tcp_server_result(void *arg, int status) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (status == 0) {
        DEBUG_printf("test success\n");
    } else {
        DEBUG_printf("test failed %d\n", status);
    }
    state->complete = true;
    return tcp_server_close(arg);
}

static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    //DEBUG_printf("tcp_server_sent %u\n", len);
    state->sent_len += len;

    if (state->sent_len >= BUF_SIZE) {
        // We should get the data back from the client
        state->recv_len = 0;
        //DEBUG_printf("Waiting for buffer from client\n");
    }
    return ERR_OK;
}

unsigned char reply[5] = {0x00, 0x00, 0xef, 0xfe, VERSION};  //calc at firmware => 0x01 = version 1/10 = 0.1

err_t tcp_server_send_data(void *arg, struct tcp_pcb *tpcb)
{
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    memcpy(state->buffer_sent, reply, 5);
    for(int i=5; i< BUF_SIZE; i++) {
        state->buffer_sent[i] = 0x0; 
    }
    // temperature
    if (pa_temp_measure_error) {
        state->buffer_sent[5] =  0xFF; //ERROR reading temperature; switch off tx mode.
        state->buffer_sent[6] =  0;
    } else {
        state->buffer_sent[5] =  0x00;
        state->buffer_sent[6] =  (pa_temparature & 0xFF); // temperature is N digit.
    }
    read_adc_values(); // not in loop for now.
    uint16_t forward = forward_adc_value;
    state->buffer_sent[7] = (forward >> 8) & 0xFF;  //msb
    state->buffer_sent[8] = forward & 0xFF; 
    uint16_t reverse = reverse_adc_value;
    state->buffer_sent[9] = (reverse >> 8) & 0xFF;  //msb
    state->buffer_sent[10] = reverse & 0xFF; 

    //DEBUG_printf("Forward voltage  %d\n", forward);
    //DEBUG_printf("Reverse voltage  %d\n", reverse);

    state->sent_len = 0;
    //DEBUG_printf("Writing %ld bytes to client\n", BUF_SIZE);

    err_t err = tcp_write(tpcb, state->buffer_sent, BUF_SIZE, TCP_WRITE_FLAG_COPY);
    if (err != ERR_OK) {
        DEBUG_printf("Failed to write data %d\n", err);
        //return tcp_server_result(arg, -12);
        return ERR_OK;
    }
    return ERR_OK;
}

err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (!p) {
       // return tcp_server_result(arg, -13);
        return ERR_OK;
    }
    
    if (p->tot_len > 0) {
        //DEBUG_printf("tcp_server_recv %d/%d err %d\n", p->tot_len, state->recv_len, err);

        // Receive the buffer
        const uint16_t buffer_left = BUF_SIZE - state->recv_len;
        state->recv_len += pbuf_copy_partial(p, state->buffer_recv + state->recv_len,
                                             p->tot_len > buffer_left ? buffer_left : p->tot_len, 0);
        tcp_recved(tpcb, p->tot_len);
    }
    pbuf_free(p);

    if (state->recv_len == BUF_SIZE) {
        //DEBUG_printf("tcp_server_recv buffer ok\n");

        uint32_t code;
        memcpy(&code, state->buffer_recv, 4);
        //DEBUG_printf("code = %x\n", code);
        switch (code)
        {
            default:
                {
                    DEBUG_printf("message not for PA controller\n");
                    for(int i=0; i< BUF_SIZE; i++) {
                            DEBUG_printf("%x-", state->buffer_recv[i]);
                    }
                    DEBUG_printf("\n");
                }
                break;
            case 0xfeef0000:
                {
                    //DEBUG_printf("message for PA controller\n");
                    //for(int i=0; i< BUF_SIZE; i++) {
                            //DEBUG_printf("%x-", state->buffer_recv[i]);
                    //}
                    //DEBUG_printf("\n");
                    deviceState = convertToPowerState(state->buffer_recv[4]);
                    if (deviceState == ACTIVE) DEBUG_printf("PA Controller active \n"); else DEBUG_printf("PA Controller not active / standby\n");
                    pttState = convertToPTTState(state->buffer_recv[5]);
                    if (pttState == PA_ON) DEBUG_printf("PTT on \n"); else DEBUG_printf("PTT off\n");
                    lpfFilter = convertToLPFFilter(state->buffer_recv[6]);
                    //DEBUG_printf("LPF is set to %x \n", lpfFilter);
                }
                break;
        }
        // sending in answer.
        return tcp_server_send_data(arg, state->client_pcb);
    }
    return ERR_OK;
}

static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb) {
    DEBUG_printf("tcp_server_poll_fn\n");
    return tcp_server_result(arg, -10); // no response is an error?
}

static void tcp_server_err(void *arg, err_t err) {
    if (err != ERR_ABRT) {
        DEBUG_printf("tcp_client_err_fn %d\n", err);
        tcp_server_result(arg, err);
    }
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (err != ERR_OK || client_pcb == NULL) {
        DEBUG_printf("Failure in accept\n");
        tcp_server_result(arg, err);
        return ERR_VAL;
    }
    DEBUG_printf("Client connected\n");

    state->client_pcb = client_pcb;
    tcp_arg(client_pcb, state);
    tcp_sent(client_pcb, tcp_server_sent);
    tcp_recv(client_pcb, tcp_server_recv);
    //tcp_poll(client_pcb, tcp_server_poll, POLL_TIME_S * 2);
    
    
    tcp_err(client_pcb, tcp_server_err);

    //return tcp_server_send_data(arg, state->client_pcb);
    
    return ERR_OK;
}

static bool tcp_server_open(void *arg) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    DEBUG_printf("Starting server at %s on port %u\n", ip4addr_ntoa(netif_ip4_addr(netif_list)), TCP_PORT);

    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        DEBUG_printf("failed to create pcb\n");
        return false;
    }

    err_t err = tcp_bind(pcb, NULL, TCP_PORT);
    if (err) {
        DEBUG_printf("failed to bind to port %u\n", TCP_PORT);
        return false;
    }

    state->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!state->server_pcb) {
        DEBUG_printf("failed to listen\n");
        if (pcb) {
            tcp_close(pcb);
        }
        return false;
    }

    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_server_accept);

    return true;
}

void run_tcp_server(void) {
    int blink_toggle = 0;

    TCP_SERVER_T *state = tcp_server_init();
    if (!state) {
        return;
    }
    if (!tcp_server_open(state)) {
        tcp_server_result(state, -11);
        return;
    }

    // Initialize PWM slice 0 on GPIO pin 1
    gpio_set_function(1, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(1); // get slice for pin 1.

    // Set PWM frequency and duty cycle
    pwm_config config = pwm_get_default_config();
    // Set the PWM frequency to 30 kHz
    pwm_config_set_clkdiv(&config, 4); // Pico runs at 125 MHz (5 is for 25KHz // 6 for 20KHz)
    pwm_init(slice_num, &config, true);

    uint16_t pwm_level = pwm_hw->slice[slice_num].top;
    //DEBUG_printf("PWM max level  %d \n", pwm_level);
   
    //while(!state->complete) {
    while(true) {
        sleep_ms(1);

        //set LED pin indication device state.
        blink_toggle++;
        if (blink_toggle > 1000) blink_toggle = 0;
        (deviceState == OFF) ? gpio_put(LED_PIN, 0) :  (deviceState == STANDBY) ? (blink_toggle > 500) ? gpio_put(LED_PIN, 0) : gpio_put(LED_PIN, 1) : (deviceState == ACTIVE) ? gpio_put(LED_PIN, 1): blink_toggle++;

        if (actualAmpState == TX_OFF)  setIOExtenderLPF(lpfFilter); // only setting filters via IO extender;  if not in TX mode.

        // if the temperature could not be deterimined do switch PTT off.
        if (!mcp9808_readtemperature()) {
            pa_temp_measure_error = true;   
            execute_sequencer_switch_pa_off();
        } else {
            pa_temp_measure_error = false;
            if (pttState == PA_ON ) execute_sequencer_switch_pa_on();
            if (pttState == PA_OFF ) execute_sequencer_switch_pa_off();
        }
        // Calculate PWM level based on temperature to set the FAN; in case of temp reading error => FAN max.
        uint16_t pwm_level = ( (pa_temparature > 25) || pa_temp_measure_error ) ? 65535 : 32767;
        pwm_set_gpio_level(1, pwm_level); 
    }
    free(state);
}

void netif_link_callback(struct netif *netif)
{
    printf("netif link status changed %s\n", netif_is_link_up(netif) ? "up" : "down");
}

void netif_status_callback(struct netif *netif)
{
    printf("netif status changed %s\n", ip4addr_ntoa(netif_ip4_addr(netif)));
}

int main() {
	// initialiseer IO
    gpio_init(BIAS_PIN);
    gpio_set_dir(BIAS_PIN, GPIO_OUT);
    gpio_put(BIAS_PIN, 0);
    gpio_init(RELAIS_PIN);
    gpio_set_dir(RELAIS_PIN, GPIO_OUT);
    gpio_put(RELAIS_PIN, 0);
	
    // LWIP network interface
    struct netif netif;

    struct netif_rmii_ethernet_config netif_config = {
        pio0, // PIO:            0
        0,    // pio SM:         0 and 1
        6,    // rx pin start:   6, 7, 8    => RX0, RX1, CRS
        10,   // tx pin start:   10, 11, 12 => TX0, TX1, TX-EN
        14,   // mdio pin start: 14, 15   => ?MDIO, MDC
        NULL, // MAC address (optional - NULL generates one based on flash id) 
    };

    // change the system clock to use the RMII reference clock from pin 20
    clock_configure_gpin(clk_sys, 20, 50 * MHZ, 50 * MHZ);
    sleep_ms(100);

    // initialize stdio after the clock change
    stdio_init_all();
    sleep_ms(5000);

    // initilize LWIP in NO SYS mode
    lwip_init();

    // initialize the PIO base RMII Ethernet network interface
    netif_rmii_ethernet_init(&netif, &netif_config);

    // Iam not usinf DHCP; i like a fixed IP address
    IP4_ADDR(&netif.ip_addr, 169, 254, 19, 101);
    IP4_ADDR(&netif.netmask, 255, 255, 0, 0);
    
    // assign callbacks for link and status
    netif_set_link_callback(&netif, netif_link_callback);
    netif_set_status_callback(&netif, netif_status_callback);

    // set the default interface and bring it up
    netif_set_default(&netif);
    netif_set_up(&netif);
    
    // setup core 1 to monitor the RMII ethernet interface
    // this let's core 0 do other things :)
    multicore_launch_core1(netif_rmii_ethernet_loop);

    httpd_init();

    // RP2040 LED ; Used as PTT indicator.
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    // initialiseer IO
    gpio_init(BIAS_PIN);
    gpio_set_dir(BIAS_PIN, GPIO_OUT);
    gpio_put(BIAS_PIN, 0);
    gpio_init(RELAIS_PIN);
    gpio_set_dir(RELAIS_PIN, GPIO_OUT);
    gpio_put(RELAIS_PIN, 0);

    // initialiseer temperature sensor and IO extender.
    mcp9808_init(); 
    mcp_init();

    // initialiseer RP2040 ADC's
    adc_init();
    adc_gpio_init(26);  //ADC0
    adc_gpio_init(27);  //ADC1

    run_tcp_server();

    return 0;
}

//end of file.