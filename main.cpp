#include "CANFDMessage.h"
#include "mbed.h"
#include "mbed_wait_api.h"
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "ACAN2517FD.h"
#include "MbedHardwareSPI.h"

using namespace acan2517fd;

#define SPI_MOSI PA_7
#define SPI_MISO PA_6
#define SPI_SCLK PB_3
#define SPI_CS PA_4
#define SPI_INT PA_3
#define ACKNOWLEDGE PA_1


#define UPDATE_DURATION 50
#define LOG_DURATION 1000

Timer timer;

/**
 * Return elapsed milliseconds time.
 */
uint32_t getMillisecond() {
    return (uint32_t) duration_cast<std::chrono::milliseconds>(timer.elapsed_time()).count();
}

//  spi and can device
SPI spi(SPI_MOSI, SPI_MISO, SPI_SCLK);
DigitalOut acknowledge(ACKNOWLEDGE);
MbedHardwareSPI dev_spi(spi, SPI_CS);
ACAN2517FD dev_can(dev_spi, getMillisecond);
DigitalIn canfd_int(SPI_INT);


//  loop counter
uint32_t last_update_time = 0;
uint32_t last_log_time = 0;
uint32_t current_time = 0;

//  error count
int error_count = 0;

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
    printf("CAN FD Software Debugger\n\r");

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

    ACAN2517FDSettings settings (ACAN2517FDSettings::OSC_4MHz, 125UL * 1000UL, DataBitRateFactor::x8);
    
    //  listen only 
    settings.mRequestedMode = ACAN2517FDSettings::ListenOnly;

    //  disable tx buffer
    settings.mDriverTransmitFIFOSize = 0;
    settings.mDriverReceiveFIFOSize = 6;

    settings.mBitRatePrescaler = 1;
    //  Arbitation Bit Rate
    settings.mArbitrationPhaseSegment1 = 255;
    settings.mArbitrationPhaseSegment2 = 64;
    settings.mArbitrationSJW = 64;
    //  Data Bit Rate
    settings.mDataPhaseSegment1 = 31;
    settings.mDataPhaseSegment2 = 8;
    settings.mDataSJW = 8;


    printf("initializing device...\n\r");
    const uint32_t errorCode = dev_can.begin (settings) ;
    if (errorCode == 0) {
        printf("initialized device!\n\r");
    }else{
        printf("Configuration error 0x%x\n\r", errorCode);
    }

    timer.start();

    while (true) {
        current_time = getMillisecond();

        //  can poll
        if (!canfd_int) {
            dev_can.isr_poll_core();
        }

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
                } else {
                    error_count++;
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

        wait_us(1000*10);
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
    printf("elapsed time[s]: %.2lf\n\r", elapsed_second);
    printf("erorr count: %d\n\r", error_count);
    printf("< Received message list >\n\n\r");
    
    //  print received messages with rate
    for (int i = 0; i < SIZE; i++) {
        if(!messages[i].is_used) {
            continue;
        }

        double last_received_at = (elapsed_time - messages[i].receive_at) / 1000.0;

        double rate = (messages[i].received_count - messages[i].last_received_count) / ((elapsed_time - last_log_time)/ 1000.0);
        messages[i].last_received_count = messages[i].received_count;

        printf(
            "id: %2u len: %2u rate: %.1lf received count: %3d received at: %.3lf\n\r", 
            messages[i].id, 
            messages[i].len, 
            rate, 
            messages[i].received_count, 
            last_received_at
        );
        
        //  show data
        for (int j = 0; j < messages[i].len; j++) {
            printf(" %x", messages[i].data[j]);
            
            if ((j+1) % 5 == 0) {
                printf("   ");
            }
            if ((j+1) % 15 == 0) {
                printf("\n\r");
            }
        }

        printf("\n\n\r");
    }
}