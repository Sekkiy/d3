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


RawSerial pc(s2v2::PC_TX, s2v2::PC_RX);
// RawSerial pc(s2v2::SBUS_TX, s2v2::SBUS_RX);
RawSerial gpsSerial(s2v2::CH5, s2v2::CH6);
BMP280 bmp(s2v2::BMP_SDA, s2v2::BMP_SCL);
HCSR04 hcsr04(s2v2::CH3, s2v2::CH4);
PpmOut ppmOut(s2v2::CH2,CHNUM); //Ch2
PpmIn ppmIn(s2v2::CH1,CHNUM); //CH1
TinyGPSPlus gps;
SDBlockDevice bd(PB_15, PB_14, PB_13, PB_12);
FATFileSystem fs("sd");
ConfigFile config;
MPU9250 mpu(s2v2::MPU_SDA,s2v2::MPU_SCL,&pc);
HMC5983 hmc(s2v2::CH8, s2v2::CH7);

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
} confVal;

void setupD3G();
int searchNewLogFileNumber(char fname[]);
void getRcch(RCChannel& ch);
void setPpm(RCChannel& ch);
void printChannels(int chNum, bool linefeed = true);
bool loadConfigFile(const char *fname);
void initializeMPU9250();
void initializeHMC5983();


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
    hmc.getXYZ(sensor.magxyz);
    D3G.updateCompassData(sensor.magxyz);
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
        pc.printf("%f\t",hmc.getHeadingDeg(sensor.magxyz[0],sensor.magxyz[1],-250, -270, false));
        pc.printf("%f\t%f\t%f\t",sensor.lat,sensor.lng,sensor.speed);        
        pc.printf("%f\t",D3G.guide.getCompassRad()* 180.0/M_PI);
        pc.printf("%f\t%f\t", D3G.getTurnDeg(), D3G.getGoalDist());
        // for(int i = 0; i<4; i++)
        //     pc.printf("%d\t", D3G.mRcch.value(i));

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
        
        //pc.printf("%f %f ",sensor.timer[0]-sensor.timer_old[0], sensor.timer[1]-sensor.timer_old[1]);
        pc.printf("\r\n");
        Thread::wait(100);
    }
}

void ch_callback(){
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


// main() runs in its own thread in the OS
int main()
{
    Thread thread[4];
    
    EventQueue queue(100 * EVENTS_EVENT_SIZE);

    pc.baud(115200);
    pc.printf("Hello SkipperS2v2\r\n");
    pc.printf("Now initilizing...\r\n");

    fs.mount(&bd);
    loadConfigFile("/sd/D3config.txt");

    setupD3G();
    t.start();

    // initializeMPU9250();
    initializeHMC5983();
    bmp.initialize(BMP280::INDOOR_NAVIGATION);

    gpsSerial.attach(callback(gps_callback));

    // thread[0].start(callback(log_thread));
    thread[1].start(callback(print_thread));
    thread[2].start(callback(&queue, &EventQueue::dispatch_forever));
    queue.call_every(17,ch_callback);
    queue.call_every(bmp.getCycle_ms(),bmp_thread);
    // queue.call_every(30,mpu_thread);
    queue.call_every(70,hmc_thread);
    
    Thread::yield;
    wait(1);

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
    // D3G.setStartTurnDeg(DEFAULT::START_TURN_DEGREE);
    // D3G.setMaxELEIncremental(DEFAULT::ELE_INCLIMENTAL);
    // D3G.setMaxRUDIncremental(DEFAULT::RUD_INCLIMENTAL);
    // D3G.setGainVtoS(DEFAULT::GAIN_V_TO_S);
    // D3G.setMaxStopSec(DEFAULT::MAX_STOP_SECOND);
    // D3G.setHoveringTHLRatio(DEFAULT::THLOTTOLE_HOVERING);
    
    // D3G.move.setNeutralELE(DEFAULT::NTL_ELE);
    // D3G.move.setForwardELE(DEFAULT::FWD_ELE);
    // D3G.move.setBackELE(DEFAULT::BCK_ELE);   
    // D3G.move.setNeutralAIL(DEFAULT::NTL_AIL);
    // D3G.move.setRightAIL(DEFAULT::RGT_AIL);    
    // D3G.move.setLeftAIL(DEFAULT::LFT_AIL);
    // D3G.move.setNeutralRUD(DEFAULT::NTL_RUD);
    // D3G.move.setRightTurnRUD(DEFAULT::RGT_TURN_RUD);
    // D3G.move.setLeftTurnRUD(DEFAULT::LFT_TURN_RUD);

    D3G.guide.setGoalLocation(confVal.goalLat, confVal.goalLng);
    D3G.guide.setCompassAxis(D3Guide::AXIS_Y, D3Guide::AXIS_X);
    D3G.guide.setMagBias(confVal.compassCalib);

    D3G.hover.setPgain(confVal.pressPgain);
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
    if(!config.load(fname)){
        confVal.pressPgain   = DF::PRESSURE_PGAIN;
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
        return false;
    }else{
        confVal.pressPgain   = config.get("PRESSURE_PGAIN");
        confVal.pressDgain   = config.get("PRESSURE_DGAIN");
        confVal.targetHeight = config.get("TARGET_HEIGHT");
        confVal.goalLat      = config.get("GOAL_LATITUDE");
        confVal.goalLng      = config.get("GOAL_LONGTITUDE");
        confVal.hoveringTHL  = config.get("THLOTTOLE_HOVERING");
        confVal.thresholdHeightRange = config.get("THRESHOLD_HEIGHT_RANGE");
        confVal.compassCalib[0] = config.get("COMPASS_CALIBRATION_X");
        confVal.compassCalib[1] = config.get("COMPASS_CALIBRATION_Y");
        confVal.compassCalib[2] = config.get("COMPASS_CALIBRATION_Z");
        confVal.trimAIL      = static_cast<float>(config.get("TLIM_AIL"));
        confVal.trimELE      = static_cast<float>(config.get("TLIM_ELE"));
        confVal.trimRUD      = static_cast<float>(config.get("TLIM_RUD"));
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
