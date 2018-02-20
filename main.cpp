#include "mbed.h"
#include "mbed_events.h"

#include "SkipperS2v2.h"
#include "BMP280.h"
#include "hcsr04.h"
#include "RCChannel.h"
#include "PPMIn.h"
#include "PPMOut.h"
#include "FATFileSystem.h"
#include "SDBlockDevice.h"
#include "TinyGPSplus.h"
#include "DigitalFilter.h"
#include "ConfigFile.h"
#include "D3Gadgets.h"
#include "MPU6050_DMP6.h"
#include <math.h>

#define LOGNAME "log"
#define CHNUM 8


RawSerial pc(s2v2::PC_TX, s2v2::PC_RX);
// RawSerial pc(s2v2::SBUS_TX, s2v2::SBUS_RX);
RawSerial gpsSerial(s2v2::CH5, s2v2::CH6);
BMP280 bmp(s2v2::BMP_SDA, s2v2::BMP_SCL);
HCSR04 hcsr04(s2v2::CH3, s2v2::CH4);
PpmOut ppmOut(s2v2::CH2,CHNUM);
PpmIn ppmIn(s2v2::CH1,CHNUM);
TinyGPSPlus gps;
SDBlockDevice bd(PB_15, PB_14, PB_13, PB_12);
FATFileSystem fs("sd");
MPU6050DMP6 mpu6050(s2v2::MPU_INT,&pc); //割り込みピン，シリアルポインタ i2cのピン指定は MPU6050>>I2Cdev.h 内のdefine
ConfigFile config;

D3Gadgets D3G;

Timer t;
Ticker chTicker;

RCChannel rcch(CHNUM);
RCChannel chControl(CHNUM);

float g_firstPress;
float g_goalLat, g_goalLong;
float g_thlRatio;
float g_thlRatio_old = 1000;


bool g_isControlled = false;

struct SensorVal{
    float press;
    float pressLP[3];
    bool BmpisUpdated;
    float rpy[3];
    int dist_cm;
} sensor;

struct ConfigVal{
    float pressPgain;
    float pressDgain;
    float targetHeight;
    float thresholdHeightRange;
    float goalLat;
    float goalLng;
    uint16_t hoveringTHL;
    uint16_t trimAIL;
    uint16_t trimELE;
    uint16_t trimRUD;
} confVal;

int searchNewLogFileNumber(char fname[]);
void getRcch(RCChannel& ch);
void setPpm(RCChannel& ch);
void printChannels(int chNum, bool linefeed = true);
bool loadConfigFile(const char *fname);


void bmp_thread(){
    CMyFilter filter[3];
    // bmp.initialize(BMP280::INDOOR_NAVIGATION);

    while(1){
        sensor.press = bmp.getPressure();
        sensor.BmpisUpdated = true;
        Thread::wait(bmp.getCycle_ms());
    }
}

void mpu_thread(){
    // if(mpu6050.setup() == -1){
    //     pc.printf("mpu is failed initilize\r\n");
    //     return;
    // }
    while(1){
        mpu6050.getRollPitchYaw_Skipper(sensor.rpy);
        // pc.printf("%f,%f,%f\r\n",sensor.rpy[0],sensor.rpy[1],sensor.rpy[2]);
        Thread::wait(20);
    }
}

void hcsr04_thread(){
    while(1){
        hcsr04.start();
        Thread::wait(25);
        sensor.dist_cm = hcsr04.get_dist_cm();
    }
}

void log_thread(){
    char filename[30];
    searchNewLogFileNumber(filename);
    FILE* fp = fopen(filename, "w");
    fprintf(fp,"create new log file\r\n");
    //fprintf(fp,"press[hPa]\tdist[cm]\ttime[ms]\tAIL\tELE\tTHL\tRUD\r\n");
    while(1){
        fprintf(fp, "%d\t%d\t%f\t", t.read(), sensor.dist_cm, sensor.press);
        // for(int i=0; i<3; i++){
        //     fprintf(fp, "%f\t", sensor.pressLP[i]); 
        // }
        for(int i=0; i<4; i++){
            fprintf(fp,"%d\t", rcch.value(i));
        }
        fprintf(fp, "%f\t", g_thlRatio); 
        fprintf(fp, "%f\t", g_thlRatio_old);
        fprintf(fp, "%f\t%f\t", D3G.hover.dH, D3G.hover.dV);
        fprintf(fp,"\r\n");
        Thread::wait(50);
    }
}

void print_thread(){
    pc.printf("press[hPa]\tdist[cm]\r\n");
    while(1){
        // pc.printf("%f\t%d\t%f",
        //           D3G.hover.pressToHeight_m(sensor.press),
        //           sensor.dist_cm,
        //           g_thlRatio);
        // pc.printf("%f\t",sensor.press);
        // printChannels(4,false);
        // pc.printf("%d\t", chControl.throttole());
        //pc.printf("%f\t%f\t", D3G.hover.dH * 100, D3G.hover.dV * 100);

        // if(gps.location.isUpdated()){
        //     pc.printf("lat:%lf\tlng:%lf\t",gps.location.lat(), gps.location.lng());
        //     pc.printf("dist:%lf\tcourse%lf\t",TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),confVal.goalLat,confVal.goalLng),
        //                                 TinyGPSPlus::courseTo(gps.location.lat(),gps.location.lng(),confVal.goalLat,confVal.goalLng));
        //     pc.printf("alt:%f\tspeed:%f\t",gps.altitude.meters(),gps.speed.mps());

        //     // pc.printf("date:%d/%d/%d/%d\t",gps.date.year(), gps.date.month(), gps.date.month(), gps.date.day());
        //     // pc.printf("time:%d h %d m %d s",gps.time.hour(), gps.time.minute(), gps.time.second());
        //     pc.printf("\r\n");
        // }
        pc.printf("%f\t%f\t%f",sensor.rpy[0]*180.0/M_PI, sensor.rpy[1]*180.0/M_PI, sensor.rpy[2]*180.0/M_PI);
        pc.printf("\r\n");
        Thread::wait(40);
    }
}

void ch_callback(){
    if(g_isControlled){
        rcch.setThrottole(chControl.throttole());
    }    
    setPpm(rcch);
    getRcch(rcch);
}

void gps_callback(){
    // pc.putc(gpsSerial.getc());
    gps.encode(gpsSerial.getc());
}


// main() runs in its own thread in the OS
int main()
{
    Thread thread[4];
    Thread qThread;
    
    EventQueue queue(32 * EVENTS_EVENT_SIZE);

    pc.baud(115200);
    pc.printf("Hello SkipperS2v2\r\n");
    pc.printf("Now initilizing...\r\n");

    fs.mount(&bd);
    loadConfigFile("/sd/D3config.txt");


    D3G.hover.setPgain(confVal.pressPgain);
    D3G.hover.setDgain(confVal.pressDgain);   
    D3G.hover.setHoveringTHLRatio(0.55);
    D3G.hover.setMaxMinTHL(0.08, -0.08);
    D3G.hover.setTargetPressure(bmp.getPressure());
    D3G.hover.setThresholdHeightRange(confVal.thresholdHeightRange);


    if(config.load("/sd/D3config.txt")){
        float test1 = config.get("TEST1_FLOAT");
        float test2 = config.get("TEST2_INT");
        pc.printf("sdinputtest\r\ntest1:%f  test2:%f\r\n",test1,test2);
    }else{
        pc.printf("false\r\n");        
    }

    // I2C i2c(s2v2::MPU_SDA, s2v2::MPU_SCL);
    // char c = 0x75;
    // i2c.write(0x68,&c,1);
    // i2c.read(0x68,&c,1);
    
    // pc.printf("id:%x\r\n",c);
    // while(1){}

    bmp.initialize(BMP280::INDOOR_NAVIGATION);
    if(mpu6050.setup() == -1){
        pc.printf("mpu is failed initilize\r\n");
    }


    thread[0].start(callback(bmp_thread));
    thread[1].start(callback(mpu_thread));
    // thread[2].start(callback(log_thread));
    thread[3].start(callback(print_thread));
    // qThread.start(callback(&queue, &EventQueue::dispatch_forever));
    // chTicker.attach_us(queue.event(ch_callback),18000);
    queue.call_every(17,ch_callback);
    queue.dispatch();
    gpsSerial.attach(callback(gps_callback));
    Thread::yield;
    wait(1);

    pc.printf("Complete initilizing\r\n");

    int t_current, t_old = 0;
    t.start();
    bool swRise = false;
    D3G.hover.setTargetPressure(sensor.press);
    
    uint16_t g_thl_old;

    while(1){
        if(rcch.value(7) > 1500){
        // if(true){
            if(!swRise){
                D3G.hover.setTargetPressure(sensor.press);
                g_thl_old = rcch.throttole();
                g_thlRatio_old = (static_cast<float>(rcch.throttole() - RCChannel::CH_MIN) )/ (static_cast<float>((RCChannel::CH_MAX - RCChannel::CH_MIN))); //
                swRise = true;
            }
            if(sensor.BmpisUpdated){
                g_thlRatio = g_thlRatio_old + D3G.hover.calcTHLRatio(sensor.press);
                chControl.setThrottoleWithRatio(g_thlRatio);
                sensor.BmpisUpdated = false;
            }
            g_isControlled = true;
        }else{
            swRise = false;
            g_isControlled = false;
        }
        Thread::yield;
    }
}

int searchNewLogFileNumber(char fname[]){
    uint16_t i_FileCheck;
    FILE *fileP;

    for (i_FileCheck = 1; i_FileCheck < 99; i_FileCheck++) {
        sprintf(fname, "/sd/Log/%s%02d.txt", LOGNAME, i_FileCheck);
        //pc.printf("%s\r\n",fname);
        
        fileP = fopen(fname, "r");
        if(fileP == NULL){              //参照した番号のファイルがなければ生成
            fileP = fopen(fname,"w");
       
            if(fileP != NULL) {
                //pc.printf("%s was created\r\n",fname);
                fclose(fileP);
                break;
            }
            //else pc.printf("Failed to create a new file.\r\n");
        }
        else fclose(fileP);
    }
    return i_FileCheck;
}

void getRcch(RCChannel& ch){
    for(int i=0; i<CHNUM; i++){
        ch.setValue(i,ppmIn.getChannel(i));
    }
}

void setPpm(RCChannel& ch){
    for(int i=0; i<CHNUM; i++){
        ppmOut.setChannel(i,ch.value(i));
    }
}

void printChannels(int chNum, bool linefeed){
    for(int i = 0; i<chNum; i++)
        pc.printf("%d\t", rcch.value(i));
    if(linefeed)
        pc.printf("\r\n");
    
}

bool loadConfigFile(const char *fname){
    namespace DF = DEFAULT;
    if(!config.load(fname)){
        confVal.pressPgain   = DF::PRESSURE_PGAIN;
        confVal.pressDgain   = DF::PRESSURE_DGAIN;
        confVal.targetHeight = DF::TARGET_HEIGHT;
        confVal.goalLat      = DF::GOAL_LATITUDE;
        confVal.goalLng      = DF::GOAL_LONGTITUDE;
        confVal.hoveringTHL  = DF::THLOTTOLE_HOVERING;
        confVal.thresholdHeightRange = DF::THRESHOLD_HEIGHT_RANGE;        
        confVal.trimAIL      = DF::TLIM_AIL;
        confVal.trimELE      = DF::TLIM_ELE;
        confVal.trimRUD      = DF::TLIM_RUD;
        return false;
    }else{
        confVal.pressPgain   = config.get("PRESSURE_PGAIN");
        confVal.pressDgain   = config.get("PRESSURE_DGAIN");
        confVal.targetHeight = config.get("TARGET_HEIGHT");
        confVal.goalLat      = config.get("GOAL_LATITUDE");
        confVal.goalLng      = config.get("GOAL_LONGTITUDE");
        confVal.hoveringTHL  = config.get("THLOTTOLE_HOVERING");
        confVal.thresholdHeightRange = config.get("THRESHOLD_HEIGHT_RANGE");
        confVal.trimAIL      = static_cast<float>(config.get("TLIM_AIL"));
        confVal.trimELE      = static_cast<float>(config.get("TLIM_ELE"));
        confVal.trimRUD      = static_cast<float>(config.get("TLIM_RUD"));
        return true;
    }
}

