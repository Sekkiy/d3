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
#include <math.h>
#include "MPU9250.h"
#include "HMC5983.h"

#define LOGNAME "log"
#define CHNUM 8

//CALIBRATE_COMPASS:コメントアウトを外すとプログラム実行せずコンパスの値をシリアルとファイルに流すだけになる
// #define CALIBRATE_COMPASS    

RawSerial pc(s2v2::PC_TX, s2v2::PC_RX);
// RawSerial pc(s2v2::SBUS_TX, s2v2::SBUS_RX);
RawSerial gpsSerial(s2v2::CH5, s2v2::CH6);
BMP280 bmp(s2v2::BMP_SDA, s2v2::BMP_SCL);
RawSerial pi(s2v2::CH1, s2v2::CH2);
PpmOut ppmOut(s2v2::CH2, CHNUM); //CH2
PpmIn ppmIn(s2v2::CH1, CHNUM); //CH1
TinyGPSPlus gps;
SDBlockDevice bd(PB_15, PB_14, PB_13, PB_12);
FATFileSystem fs("sd");
MPU9250 mpu(s2v2::MPU_SDA,s2v2::MPU_SCL,&pc);
HMC5983 hmc(s2v2::CH8, s2v2::CH7);
DigitalOut led1(s2v2::CH5);

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
    bool bmpIsUpdated;
    float rpy[3];
    int16_t magxyz[3];
    bool hmcIsUpdated;
    double lat;
    double lng;
    double speed;
    float timer[3];
    float timer_old[3];
} sensor;

struct ConfigVal{
    float pressPgain;
    float pressIgain;
    float pressDgain;
    float targetHeight;
    float thresholdHeightRange;
    float goalLat;
    float goalLng;
    int16_t compassCalib[3];
    uint16_t hoveringTHL;
    uint16_t trimAIL;
    uint16_t trimELE;
    uint16_t trimRUD;
    
    //平面誘導
    float startTurnDeg;
    uint16_t AILIncremental;
    uint16_t ELEIncremental;
    uint16_t RUDIncremental;
    float gainVtoS;
    float maxStopSecond;
    uint8_t compassAxisForward;
    uint8_t compassAxisRight;
    float changeSequenceDist;
    float startDecelerationDist;
    float decelerationRate;

    uint16_t AILNeutral;
    uint16_t ELENeutral;
    uint16_t RUDNeutral;
} confVal;

void setupD3G();
int searchNewLogFileNumber(char fname[]);
void getRcch(RCChannel& ch);
void setPpm(RCChannel& ch);
void printChannels(int chNum, bool linefeed = true);
bool loadConfigFile(const char *fname);
void initializeMPU9250();
void initializeHMC5983();
void calibrateCompass();


void bmp_thread(){
    sensor.press = bmp.getPressure();
    D3G.updateBaroData(sensor.press);
    sensor.bmpIsUpdated = true;
    sensor.timer_old[0] = sensor.timer[0];
    sensor.timer[0] = t.read_ms();
}

void mpu_thread(){
    if(!mpu.sensingAcGyMg()){
        return;
    }
    
    // if (imu.fifoAvailable()){ //fifo is not being available
    //     wait_ms(5);
    //     // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    //     if (imu.dmpUpdateFifo() == INV_SUCCESS){
    //         // imu.computeEulerAngles();
    //         imu.computeEulerAngles_D3();
    //         pc.printf("%.4lf, %.4lf, %.4lf\r\n", imu.yaw,imu.pitch,imu.roll);
            
    //     }
    // }
    sensor.timer_old[1] = sensor.timer[1];
    sensor.timer[1] = t.read_ms();
}

//x -250
//y -270
void hmc_thread(){
    int16_t magxyz[3];
    hmc.getXYZ(magxyz);
    //ノイズ対策
    if((abs(magxyz[0])<3000) && (abs(magxyz[1])<3000) && (abs(magxyz[2])<3000)){
        for(int i=0; i<3; i++)
            sensor.magxyz[i] = magxyz[i];
        D3G.updateCompassData(sensor.magxyz);
    } 
    // D3G.guide.updateCompass(sensor.magxyz);
    // pc.printf("%d\t%d\t%d\t%lf\t%lf\t%lf\r\n",sensor.magxyz[0],sensor.magxyz[1],sensor.magxyz[2],
    //                                 hmc.getHeadingDeg(sensor.magxyz[0],sensor.magxyz[1],-13, -169),
    //                                 hmc.getHeadingDeg(sensor.magxyz[0],sensor.magxyz[1],-250, -270),                                    
    //                                 hmc.getHeadingDeg(-(sensor.magxyz[2]+208),sensor.magxyz[1]+169,0,0));
}

void log_thread(){
    char filename[30];
    searchNewLogFileNumber(filename);
    FILE* fp = fopen(filename, "w");
    fprintf(fp,"create new log file\r\n");
    //fprintf(fp,"press[hPa]\tdist[cm]\ttime[ms]\tAIL\tELE\tTHL\tRUD\r\n");
    while(1){
        fprintf(fp, "%f\t%f\t", t.read(), sensor.press);
        for(int i=0; i<3; i++)
            fprintf(fp,"%d\t",sensor.magxyz[i]);
        fprintf(fp,"%f\t",hmc.getHeadingDeg(sensor.magxyz[0],sensor.magxyz[1],-250, -270));        
        fprintf(fp, "%f\t%f\tf\t", sensor.lat,sensor.lng,sensor.speed);
        fprintf(fp, "%f\t%f\t", D3G.getTurnDeg(), D3G.getGoalDist());
        for(int i=0; i<4; i++)
            fprintf(fp,"%d\t", rcch.value(i));
        // fprintf(fp, "%f\t", g_thlRatio); 
        // fprintf(fp, "%f\t", g_thlRatio_old);
        // fprintf(fp, "%f\t%f\t", D3G.hover.dH, D3G.hover.dV);
        fprintf(fp,"\r\n");
        Thread::wait(100);
    }
}

void print_thread(){
    pc.printf("press[hPa]\tdist[cm]\r\n");
    while(1){
        // pc.printf("%f\t%f",
        //           D3G.hover.pressToHeight_m(sensor.press),
        //           g_thlRatio);
        pc.printf("%f\t",sensor.press);
        for(int i=0; i<3; i++)
            pc.printf("%d\t",sensor.magxyz[i]);
        // pc.printf("%f\t",hmc.getHeadingDeg(sensor.magxyz[0],sensor.magxyz[1],-250, -270, false));
        pc.printf("%f\t%f\t%f\t",sensor.lat,sensor.lng,sensor.speed);        
        pc.printf("%f\t",D3G.guide.getCompassRad()* 180.0/M_PI);
        pc.printf("%f\t%f\t", D3G.getTurnDeg(), D3G.getGoalDist());
        // for(int i = 0; i<4; i++)
        //     pc.printf("%d\t", D3G.mRcch.value(i));

        printChannels(4,false);
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
        
        //pc.printf("%f %f ",sensor.timer[0]-sensor.timer_old[0], sensor.timer[1]-sensor.timer_old[1]);
        pc.printf("\r\n");
        Thread::wait(100);
    }
}

void ch_thread(){
    D3G.process(rcch);   
    if(g_isControlled){
        // rcch.setThrottole(chControl.throttole());
    }
    setPpm(rcch);
    getRcch(rcch);
}

void gps_callback(){
    gps.encode(gpsSerial.getc());
}

void raspi_callback(){
    D3G.guide.encode(pc.getc());
    // if(D3G.guide.RaspiIsUpdated())
    //     pc.printf("%d\t%d\r\n",D3G.guide.getRaspiX(), D3G.guide.getRaspiY());
}

// main() runs in its own thread in the OS
int main()
{
    Thread thread[4];
    
    EventQueue queue(100 * EVENTS_EVENT_SIZE);

    pc.baud(115200);
    pc.printf("Hello SkipperS2v2\r\n");
    pc.printf("Now initilizing...\r\n");

    // fs.mount(&bd);
    // loadConfigFile("/sd/D3config.txt");

    setupD3G();
    t.start();

    // initializeMPU9250();
    initializeHMC5983();
#ifdef CALIBRATE_COMPASS
    calibrateCompass();
#endif    
    bmp.initialize(BMP280::INDOOR_NAVIGATION);

    gpsSerial.attach(callback(gps_callback));
    pc.attach(callback((raspi_callback)));

    // thread[0].start(callback(log_thread));
    thread[1].start(callback(print_thread));
    thread[2].start(callback(&queue, &EventQueue::dispatch_forever));
    queue.call_every(17,ch_thread);
    queue.call_every(bmp.getCycle_ms(),bmp_thread);
    // queue.call_every(30,mpu_thread);
    queue.call_every(70,hmc_thread);
    
    Thread::yield;
    led1 = 0;
    wait(1);
    led1 = 1;

    pc.printf("Complete initilizing\r\n");

    int t_current, t_old = 0;
    bool swRise = false;
    D3G.hover.setTargetPressure(sensor.press);

    uint16_t g_thl_old;
     
    while(1){
        if(gps.location.isUpdated()){
            sensor.lat = gps.location.lat();
            sensor.lng = gps.location.lng();
            sensor.speed = gps.speed.mps();
            D3G.updateGPSData(sensor.lat, sensor.lng, sensor.speed);
            // D3G.guide.updateCurrentLocation(sensor.lat,sensor.lng);
        }
        if(D3G.getGoalDist() < confVal.changeSequenceDist)
            led1 = 0;
        else
            led1 = 1;
        if(rcch.value(7) > 1500){
        // if(true){
            D3G.nowControlling(true);
            if(!swRise){
                D3G.hover.setTargetPressure(sensor.press);
                g_thl_old = rcch.throttole();
                g_thlRatio_old = (static_cast<float>(rcch.throttole() - RCChannel::CH_MIN) )/ (static_cast<float>((RCChannel::CH_MAX - RCChannel::CH_MIN))); //
                swRise = true;
            }
            if(sensor.bmpIsUpdated){
                g_thlRatio = g_thlRatio_old + D3G.hover.calcTHLRatio(sensor.press);
                chControl.setThrottoleWithRatio(g_thlRatio);
                sensor.bmpIsUpdated = false;
            }
            g_isControlled = true;
        }else{
            D3G.nowControlling(false);        
            swRise = false;
            g_isControlled = false;
        }
        Thread::yield;
    }
}

void setupD3G(){
    D3G.setStartTurnDeg(confVal.startTurnDeg);
    D3G.setMaxELEIncremental(confVal.ELEIncremental);
    D3G.setMaxRUDIncremental(confVal.RUDIncremental);
    D3G.setGainVtoS(confVal.gainVtoS);
    D3G.setMaxStopSec(confVal.maxStopSecond);
    D3G.setChangeSequenceDist(confVal.changeSequenceDist);
    D3G.setStartDecelerationDist(confVal.startDecelerationDist);
    D3G.setDecelerationRate(confVal.decelerationRate);    
    // D3G.setHoveringTHLRatio(confVal.hoveringTHLRatio);
    
    D3G.move.setNeutralELE(confVal.ELENeutral);
    D3G.move.setForwardELE(confVal.ELENeutral + confVal.ELEIncremental);
    D3G.move.setBackELE(confVal.ELENeutral - confVal.ELEIncremental);
    D3G.move.setNeutralAIL(confVal.AILNeutral);
    D3G.move.setRightAIL(confVal.AILNeutral + confVal.AILIncremental);
    D3G.move.setLeftAIL(confVal.AILNeutral - confVal.AILIncremental);
    D3G.move.setNeutralRUD(confVal.RUDNeutral);
    D3G.move.setRightTurnRUD(confVal.RUDNeutral + confVal.RUDIncremental);
    D3G.move.setLeftTurnRUD(confVal.RUDNeutral - confVal.RUDIncremental);

    D3G.guide.setGoalLocation(confVal.goalLat, confVal.goalLng);
    D3G.guide.setCompassAxis(static_cast<D3Guide::compassAxis>(confVal.compassAxisForward),
                             static_cast<D3Guide::compassAxis>(confVal.compassAxisRight));
    D3G.guide.setMagBias(confVal.compassCalib);

    D3G.hover.setPgain(confVal.pressPgain);
    D3G.hover.setIgain(confVal.pressIgain);
    D3G.hover.setDgain(confVal.pressDgain);
    D3G.hover.setHoveringTHLRatio(0.55);
    D3G.hover.setMaxMinTHL(0.08, -0.08);
    D3G.hover.setTargetPressure(bmp.getPressure());
    D3G.hover.setThresholdHeightRange(confVal.thresholdHeightRange);
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
    ConfigFile config;
    if(!config.load(fname)){
        confVal.pressPgain   = DF::PRESSURE_PGAIN;
        confVal.pressIgain   = DF::PRESSURE_IGAIN;
        confVal.pressDgain   = DF::PRESSURE_DGAIN;
        confVal.targetHeight = DF::TARGET_HEIGHT;
        confVal.goalLat      = DF::GOAL_LATITUDE;
        confVal.goalLng      = DF::GOAL_LONGTITUDE;
        confVal.hoveringTHL  = DF::THLOTTOLE_HOVERING;
        confVal.thresholdHeightRange = DF::THRESHOLD_HEIGHT_RANGE;        
        confVal.compassCalib[0] = DF::COMPASS_CALIBRATION_X;
        confVal.compassCalib[1] = DF::COMPASS_CALIBRATION_Y;
        confVal.compassCalib[2] = DF::COMPASS_CALIBRATION_Z;
        confVal.trimAIL      = DF::TLIM_AIL;
        confVal.trimELE      = DF::TLIM_ELE;
        confVal.trimRUD      = DF::TLIM_RUD;

        confVal.startTurnDeg    = DF::START_TURN_DEGREE;
        confVal.AILIncremental  = DF::AIL_INCLIMENTAL;
        confVal.ELEIncremental  = DF::ELE_INCLIMENTAL;
        confVal.RUDIncremental  = DF::RUD_INCLIMENTAL;
        confVal.gainVtoS        = DF::GAIN_V_TO_S;
        confVal.maxStopSecond   = DF::MAX_STOP_SECOND;
        confVal.compassAxisForward = DF::COMPASS_AXIS_FORWARD;
        confVal.compassAxisRight   = DF::COMPASS_AXIS_RIGHT;
        confVal.changeSequenceDist = DF::CHANGE_SEQUENCE_DISTANCE;
        confVal.startDecelerationDist = DF::START_DECELERATION_DISTANCE;
        confVal.decelerationRate = DF::DECELERATION_RATE;

        confVal.AILNeutral   = DF::NTL_AIL;
        confVal.ELENeutral   = DF::NTL_ELE;
        confVal.RUDNeutral   = DF::NTL_RUD;
        return false;
    }else{
        confVal.pressPgain   = config.get("PRESSURE_PGAIN");
        confVal.pressIgain   = config.get("PRESSURE_IGAIN");
        confVal.pressDgain   = config.get("PRESSURE_DGAIN");
        confVal.targetHeight = config.get("TARGET_HEIGHT");
        confVal.goalLat      = config.get("GOAL_LATITUDE");
        confVal.goalLng      = config.get("GOAL_LONGTITUDE");
        confVal.hoveringTHL  = config.get("THLOTTOLE_HOVERING");
        confVal.thresholdHeightRange = config.get("THRESHOLD_HEIGHT_RANGE");
        confVal.compassCalib[0] = config.get("COMPASS_CALIBRATION_X");
        confVal.compassCalib[1] = config.get("COMPASS_CALIBRATION_Y");
        confVal.compassCalib[2] = config.get("COMPASS_CALIBRATION_Z");
        confVal.trimAIL      = static_cast<uint16_t>(config.get("TLIM_AIL"));
        confVal.trimELE      = static_cast<uint16_t>(config.get("TLIM_ELE"));
        confVal.trimRUD      = static_cast<uint16_t>(config.get("TLIM_RUD"));
        
        confVal.startTurnDeg = config.get("START_TURN_DEGREE");
        confVal.AILIncremental= static_cast<uint16_t>(config.get("AIL_INCLIMENTAL"));
        confVal.ELEIncremental= static_cast<uint16_t>(config.get("ELE_INCLIMENTAL"));
        confVal.RUDIncremental= static_cast<uint16_t>(config.get("RUD_INCLIMENTAL"));
        confVal.gainVtoS     = config.get("GAIN_V_TO_S");
        confVal.maxStopSecond= config.get("MAX_STOP_SECOND");
        confVal.compassAxisForward = static_cast<uint8_t>(config.get("COMPASS_AXIS_FORWARD"));
        confVal.compassAxisRight   = static_cast<uint8_t>(config.get("COMPASS_AXIS_RIGHT"));
        confVal.changeSequenceDist = config.get("CHANGE_SEQUENCE_DISTANCE");
        confVal.startDecelerationDist = config.get("START_DECELERATION_DISTANCE");
        confVal.decelerationRate = config.get("DECELERATION_RATE");

        confVal.AILNeutral   = static_cast<uint16_t>(config.get("NTL_AIL"));
        confVal.ELENeutral   = static_cast<uint16_t>(config.get("NTL_ELE"));
        confVal.RUDNeutral   = static_cast<uint16_t>(config.get("NTL_RUD"));
        return true;
    }
}

void initializeMPU9250(){

    if(!mpu.Initialize()){
        pc.printf("failed initialize\r\n");
        while(1);
    }
    mpu.setMagBias(0,0,0);

    mpu.sensingAcGyMg();

    // imu_init();
    // stamper_init();
    // if (imu.begin() != INV_SUCCESS)
    // {
    //     while(1){
    //         pc.printf("Unable to communicate with MPU-9250");
    //         pc.printf("Check connections, and try again.\n");
    //         wait_ms(5000);
    //     }
    // }
    // pc.printf("imu.begin() suceeded\n");

    // if (imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT |  // Enable 6-axis quat
    //                      DMP_FEATURE_GYRO_CAL, // Use gyro calibration
    //                 150) == INV_ERROR)
    // {                                            // Set DMP FIFO rate to 150 Hz
    //     pc.printf("imu.dmpBegin have failed\n"); //dmpLoad function under it fails which is caused by memcmp(firmware+ii, cur, this_write) (it is located at row 2871 of inv_mpu.c)
    // }
    // else
    // {
    //     pc.printf("imu.dmpBegin() suceeded\n");
    // }
}

void initializeHMC5983(){
    hmc.init();
}

void calibrateCompass(){
    int16_t magxyz[3];
    bool enableWrite = true;
    FILE* fp = fopen("/sd/CompassCalibration.txt", "w");
    if(fp == NULL)
        enableWrite = false;
    pc.printf("compass calibration...\r\n");
    while(1){
        hmc.getXYZ(magxyz);
        if ((abs(magxyz[0]) < 3000) && (abs(magxyz[1]) < 3000) && (abs(magxyz[2]) < 3000)){
            pc.printf("%d\t%d\t%d\r\n", magxyz[0], magxyz[1], magxyz[2]);
            if (enableWrite)
                fprintf(fp, "%d\t%d\t%d\r\n", magxyz[0], magxyz[1], magxyz[2]);
            wait_ms(30);
        }
    }
}
