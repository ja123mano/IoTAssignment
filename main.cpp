#include "mbed.h"
#include "string.h"
#include "ADXL362.h"
#include "ESP8266Interface.h"
#include "Adafruit_SSD1331.h"
#include "Adafruit_GFX.h"
#include <MQTTClientMbedOs.h>
#include <cmath>

#define Black 0x0000
#define Blue 0x001F
#define Red 0xF800
#define Green 0x07E0
#define Cyan 0x07FF
#define Magenta 0xF81F
#define Yellow 0xFFE0
#define White 0xFFFF

ADXL362 ADXL362(D5,D11,D12,D13);
SPI spi(D11, D12, D13);
DigitalOut alsCS(D4);
Adafruit_SSD1331 OLED(A7, D6, D10, D11, NC, D13);

//Threads
Thread detect_thread;
Thread recv_thread;
Thread send_thread;

DigitalOut lightLed(D2);
DigitalOut stationaryLed(D3);

void movement_necessary(int detectValue);
int ADXL362_movement_detect();
int getALS();
void udpReceive();
void udpSend();

char buffer[128];
int alsScaledI = 0;
int8_t x,y,z;
int movementNecessary = 0;  //Data stored in milliseconds
int i = 0;

int main() {
    //MOD WIFI settings configuration ---------------------------------------------------------------------------
    ESP8266Interface esp(MBED_CONF_APP_ESP_TX_PIN, MBED_CONF_APP_ESP_RX_PIN);
    SocketAddress deviceIP;
    SocketAddress MQTTBroker;
    TCPSocket socket;
    MQTTClient client(&socket);
    ThisThread::sleep_for(3s);    // Waiting for the user to open serial terminal

    int ret = esp.connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) { printf("\nConnection error\n"); }
    else { printf("\nConnection success\n"); }

    esp.gethostbyname(MBED_CONF_APP_MQTT_BROKER_HOSTNAME, &MQTTBroker, NSAPI_IPv4, "esp");
    MQTTBroker.set_port(MBED_CONF_APP_MQTT_BROKER_PORT);

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;       
    data.MQTTVersion = 3;
    char *id = MBED_CONF_APP_MQTT_ID;
    data.clientID.cstring = id;

    sprintf(buffer, "Hello from Mbed OS %d.%d", MBED_MAJOR_VERSION, MBED_MINOR_VERSION);
    MQTT::Message msg;
    msg.qos = MQTT::QOS0;
    msg.retained = false;
    msg.dup = false;
    msg.payload = (void*)buffer;
    msg.payloadlen = strlen(buffer);
    ThisThread::sleep_for(5s);

    socket.open(&esp);
    socket.connect(MQTTBroker);
    client.connect(data);
    client.publish(MBED_CONF_APP_MQTT_TOPIC, msg);

    //Pmod ALS settings configuration -----------------------------------------------------------------------------
    spi.format(8,0);           
    spi.frequency(2000000);
    alsCS.write(1);

    //Pmod ACL2 settings configuration ----------------------------------------------------------------------------
    ADXL362.reset();
    ThisThread::sleep_for(600ms);
    ADXL362.set_mode(ADXL362::MEASUREMENT);
    //If the sensor is not moved within 1 second (for real purposes, it should be 1 hour)
    //a LED will turn on indicating the lack of physical activity
    detect_thread.start(ADXL362_movement_detect);

    //Pmod OLED rgb settings configuration ----------------------------------------------------------------------------
    OLED.begin();
    OLED.clearScreen();

    while(true) {
    alsScaledI = getALS(); 
    //alsScaledI = ambient light scaled to LUX, if less than 100, it's too dark
    if (alsScaledI > 100) { lightLed.write(0); }
    else { lightLed.write(1); }

    sprintf(buffer, "{\"light\":%0d, \"mov\":%d}", alsScaledI, movementNecessary);
    msg.payload = (void*)buffer;
    msg.payloadlen = strlen(buffer);
    client.publish(MBED_CONF_APP_MQTT_TOPIC, msg);

    //printf("Sent payload with light intensity and ms without movement\n");
    //printf("Light intensity (LUX) = %d\n", alsScaledI);
    //printf("Seconds without moving = %d\n\n", movementNecessary/1000);

    ThisThread::sleep_for(50ms);
    OLED.clearScreen();
    OLED.fillScreen(Blue);
    OLED.setTextColor(Cyan);
    OLED.setCursor(0,0);
    OLED.printf("Hello from program\n");
    ThisThread::sleep_for(1s);
    }
}

void movement_necessary(int detectValue) {
    if (!detectValue) {
        movementNecessary += 100;
    } else {
        movementNecessary = 0;
        stationaryLed.write(0);
    }

    if (movementNecessary >= 1000) {
        stationaryLed.write(1);
    }
}

int ADXL362_movement_detect() {
    int8_t x1,y1,z1,x2,y2,z2,dx,dy,dz;
    int detect;
    while(1){
        x1=ADXL362.scanx_u8();
        y1=ADXL362.scany_u8();
        z1=ADXL362.scanz_u8();
        ThisThread::sleep_for(10ms);
        x2=ADXL362.scanx_u8();
        y2=ADXL362.scany_u8();
        z2=ADXL362.scanz_u8();
            
        x=(x1 + x2)/2;
        y=(y1 + y2)/2;
        z=(z1 + z2)/2;
         
        dx=abs(x1 - x2);
        dy=abs(y1 - y2);
        dz=abs(z1 - z2);
        
        if (dx>10 || dy>10 || dz>10) {
            detect = 1;
        } else {
            detect = 0;
        }

        movement_necessary(detect);
        ThisThread::sleep_for(100ms);
        }    
}

int getALS() {
    char alsByte0 = 0;
    char alsByte1 = 0;
    char alsByteSh0 = 0;
    char alsByteSh1 = 0;
    char als8bit = 0;
    unsigned short alsRaw = 0;
    float alsScaledF = 0;

    alsCS.write(0); 
    ThisThread::sleep_for(1ms);
    alsByte0 = spi.write(0x00);
    alsByte1 = spi.write(0x00);
    alsCS.write(1);
    ThisThread::sleep_for(1ms);

    alsByteSh0 = alsByte0 << 4;
    alsByteSh1 = alsByte1 >> 4;
    als8bit =( alsByteSh0 | alsByteSh1 );    
    alsRaw = als8bit; 
    alsScaledF = (float(alsRaw))*(float(6.68)); 
    
    //printf("Ambient light raw (8 bit) = '%d' \r\n",alsRaw);
    return (int)alsScaledF; 
}
