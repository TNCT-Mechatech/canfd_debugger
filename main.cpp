#include "CANFDMessage.h"
#include "mbed.h"
#include "mbed_wait_api.h"
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "ACAN2517FD.h"
#include "MbedHardwareSPI.h"

using namespace acan2517fd;

#define MOSI D11
#define MISO D12
#define SCLK D13
#define CS D10
#define ACKNOWLEDGE D9

#define UPDATE_DURATION 50
#define LOG_DURATION 2000

Timer timer;

/**
 * Return elapsed milliseconds time.
 */
uint32_t getMillisecond() {
    return (uint32_t) duration_cast<std::chrono::milliseconds>(timer.elapsed_time()).count();
}

//  spi and can device
SPI spi(MOSI, MISO, SCLK);
MbedHardwareSPI hardware_dev(spi, CS);
ACAN2517FD dev_can(hardware_dev, getMillisecond);

//  ACKNOWLEDGE LED
DigitalOut acknowledge(ACKNOWLEDGE);

//  loop counter
uint32_t last_update_time = 0;
uint32_t last_log_time = 0;
uint32_t current_time = 0;

//  observing data
typedef struct {
    bool is_used;
    uint32_t id;
    uint8_t len;
    uint8_t data[64];
    uint32_t receive_at;
    int received_count;
    int last_received_count;
} observed_message_t;

#define SIZE 20
observed_message_t messages[SIZE];

//  proto type define
void report();

int find_msg(CANFDMessage & msg);
int get_new_id();

// main() runs in its own thread in the OS
int main()
{
    //  init messages
    for (int i = 0; i < SIZE; i++) {
        messages[i].is_used = false;
        messages[i].id = 0;
        messages[i].len = 0;
        memset(messages[i].data, 0, 64);
        messages[i].receive_at = 0;
        messages[i].received_count = 0;
        messages[i].last_received_count = 0;
    }

    ACAN2517FDSettings settings (ACAN2517FDSettings::OSC_4MHz, 125UL * 1000UL, DataBitRateFactor::x1);
    
    //  listen only 
    settings.mRequestedMode = ACAN2517FDSettings::ListenOnly;

    //  disable tx buffer
    settings.mDriverTransmitFIFOSize = 0;
    settings.mDriverReceiveFIFOSize = 6;


    printf("initializing device...\n\r");
    const uint32_t errorCode = dev_can.begin (settings) ;
    if (errorCode == 0) {
        printf("initialized device!\n\r");
    }else{
        printf("Configuration error 0x%x\n\r", errorCode);
    }

    while (true) {
        current_time = getMillisecond();

        //  can poll
        dev_can.poll();

        if(current_time - last_update_time > UPDATE_DURATION) {
            while (dev_can.available()) {
                CANFDMessage msg;

                //  toggle acknowledge
                acknowledge = !acknowledge;

                if (dev_can.receive(msg)) {
                    int id = find_msg(msg);
                    if (id >= 0) {
                        messages[id].len = msg.len;
                        memcpy(messages[id].data, msg.data, msg.len);
                        messages[id].receive_at = current_time;
                        messages[id].received_count++;
                    } else {
                        id = get_new_id();
                        //  cannot find greate id
                        if (id < 0) {
                            continue;
                        }

                        messages[id].is_used = true;
                        messages[id].id = msg.id;
                        messages[id].len = msg.len;
                        memcpy(messages[id].data, msg.data, msg.len);
                        messages[id].receive_at = current_time;
                        messages[id].received_count++;
                    }
                }
            }

            //  update time
            last_update_time = current_time;
        }

        if (current_time - last_log_time > LOG_DURATION) {
            report();

            //  update time
            last_log_time = current_time;
        }
    }
}

int find_msg(CANFDMessage & msg) {
    for (int i = 0; i < SIZE; i++) {
        if(messages[i].is_used && messages[i].id == msg.id) {
            return i;
        }
    }
    return -1;
}

int get_new_id() {
    for (int i = 0; i < SIZE; i++) {
        if(!messages[i].is_used) {
            return i;
        }
    }
    return -1;
}

void report() {
    uint32_t elapsed_time = getMillisecond();
    double elapsed_second = elapsed_time / 1000.0;

    printf("==============================\n\r");
    printf("elapsed time[ms]: %.2lf\n\r", elapsed_second);
    printf("< Received message list >\n\n\r");
    
    //  print received messages with rate
    for (int i = 0; i < SIZE; i++) {
        if(!messages[i].is_used) {
            continue;
        }

        double last_received_at = (elapsed_time - messages[i].receive_at) / 1000.0;

        double rate = (messages[i].received_count - messages[i].last_received_count) / (LOG_DURATION / 1000.0);
        messages[i].last_received_count = messages[i].received_count;

        printf("id: %4u len: %2u rate: %.1lf received at: %.1lf\n\r", messages[i].id, messages[i].len, rate, last_received_at);
        
        //  show data
        for (int j = 0; i < messages[i].len; j++) {
            printf(" %x", messages[i].data[j]);
            
            if ((j+1) % 5 == 0) {
                printf("   ");
            }
        }

        printf("\n\n\r");
    }

    printf("END\n\n\r");
}