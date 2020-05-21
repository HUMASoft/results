#include <iostream>
#include <fstream>
#include "Cia402device.h"
#include "SocketCanPort.h"

#include "math.h"

#include "SerialArduino.h"

#include "fcontrol.h"
#include "IPlot.h"
#include "OnlineSystemIdentification.h"

#include "Kinematics.h"


/// IEEE Access paper experimental code
/// Performs all experiments used in the paper and
/// stores all data in .csv files

// It requires: -Platform inclination=0
//              -Reset IMU sensor

long AdaptiveStep(double target, double time);

int main (){

    //--sensor--
    SerialArduino imu;
    double imuIncli=0,imuOrien,imuIncliOld=0;
//    SystemBlock imuFilter(0.09516,0,- 0.9048,1); //w=5

    //    sleep(4); //wait for sensor

    ofstream sysdatanum("/home/humasoft/Escritorio/adasysnum000.csv",std::ofstream::out);
    ofstream sysdataden("/home/humasoft/Escritorio/adasysden000.csv",std::ofstream::out);
    ofstream sysdatamp("/home/humasoft/Escritorio/sensor-response.csv",std::ofstream::out);

    ofstream condata("/home/humasoft/Escritorio/adacon000.c.jsv",std::ofstream::out);
    ofstream data("/home/humasoft/Escritorio/sensor-response.csv",std::ofstream::out);

    //Samplinfg time
    double dts=0.02; //
    SamplingTime Ts(dts);

    /// System identification
    //tau = 0.1
    //    0.09516
    //   ----------
    //   z - 0.9048
//    SystemBlock filter(0.09516,0,- 0.9048,1); //w=5
//    SystemBlock filter(-19,21,1,1); //w=5   21 z - 19 /  z + 1
    double wf=1;

    SystemBlock filterSensor(wf*dts,wf*dts,wf*dts-2,2+wf*dts); //w*dts*(z+1)/(z*(2+w*dts)+(w*dts-2));
    SystemBlock filterSignal(wf*dts,wf*dts,wf*dts-2,2+wf*dts); //w*dts*(z+1)/(z*(2+w*dts)+(w*dts-2));
    SystemBlock filterMag(wf*dts,wf*dts,wf*dts-2,2+wf*dts); //w*dts*(z+1)/(z*(2+w*dts)+(w*dts-2));
    SystemBlock filterPhi(wf*dts,wf*dts,wf*dts-2,2+wf*dts); //w*dts*(z+1)/(z*(2+w*dts)+(w*dts-2));


    ulong numOrder=0,denOrder=1;
    SystemBlock filter(wf*dts,wf*dts,wf*dts-2,2+wf*dts); //w*dts*(z+1)/(z*(2+w*dts)+(w*dts-2));
    OnlineSystemIdentification model(numOrder, denOrder, filter, 0.95, 0.99, 400 );
    double convergence=0;
    vector<double> num(numOrder+1),den(denOrder+1); //(order 0 also counts)
    SystemBlock integral(0,1,-1,1);
    vector<SystemBlock> sys = {SystemBlock(num,den),integral}; //the resulting identification
    double sysk=0, syskAverage=0.1;



    ///Controller and tuning
//    FPDBlock con(0,0,0,dts);
    FPDBlock con(0.15,0.03,0.75,dts);
    FPDBlock scon(0.23,0.36,-0.6,dts);
    PIDBlock intcon(0.1,0,0.1,dts);
    //double phi,mag,w=1;


    ///Motor command
    //  data << "Controller PID" << " , " << " 0.1,0.05,0,dts "<< endl;
    //m1 setup
    SocketCanPort pm31("can1");
    CiA402SetupData sd31(2048,24,0.001, 0.144, 20);
    CiA402Device m1 (31, &pm31, &sd31);
    m1.Reset();
    m1.SwitchOn();
    //    m1.SetupPositionMode(10,10);
    m1.Setup_Velocity_Mode(5);
    //  m1.Setup_Torque_Mode();


    IPlot id;



    //tilt sensor initialization
    for (double t=0; t<3; t+=10*dts)
    {
        if (imu.readSensor(imuIncli,imuOrien)>=0) /*break;*/
        cout << "Initializing sensor! " ;

    }



    double psr; //pseudorandom
    double interval=3; //in seconds

    //populate system matrices
    for (double t=0;t<interval; t+=dts)
    {

        psr=+0.1*((rand() % 10 + 1)-5); //pseudorandom
        imuIncliOld=imuIncli;

        if (imu.readSensor(imuIncli,imuOrien) <0)
        {
            cout << "Initializing sensor! " ;
            //Sensor error, do nothing.
            cout << "Inc: " << imuIncli << " ; Ori: "  << imuOrien << endl;

        }
        else
        {
//            cout << "t: " << t << endl;
//            cout << "Inc: " << imuIncli << " ; Ori: "  << imuOrien << endl;

            m1.SetVelocity(psr);
            model.UpdateSystem(psr, imuIncliOld);
//            model.GetAvgSystemBlock(sys[0]);
//            tuner.TuneIsom(sys,con);
        }

        Ts.WaitSamplingTime();

    }


    //Main control loop


    double incli=20, error=0, cs=0; double vincli=0;
    double kp = 0.0,kd = 0.0,fex = 0.0;
    double smag = 0.0,sphi = 0.0, wgc=1.0;
    interval=300; //in seconds


    for (double t=0;t<interval; t+=dts)
    {


        psr=+0.1*((rand() % 10 + 0)-5); //new pseudorandom data

        //    incli=incli+psr;
        //    orien=0;

        ///read sensor
        imuIncliOld=imuIncli;
        if (imu.readSensor(imuIncli,imuOrien) <0)
        {
            cout << "Sensor error! ";
            //Sensor error, do nothing.
//            cout << "Inc: " << imuIncli << " ; Ori: "  << imuOrien << endl;
        }
        else
        {
//                        imuIncli = imuIncli > imuFilter;
            //Compute error
            error=(psr+incli)-imuIncli;
            //        cout << "incli: " << incli << " ; imuIncli: "  << imuIncli << endl;

            //Controller command
            cs = error > scon;
            m1.SetVelocity(cs);
            //            cout << "cs: " << cs << " ; error: "  << error << endl;
            //Update model

            //velocity / velocity id
            convergence = model.UpdateSystem(cs, (imuIncli-imuIncliOld)/dts);

            //velocity / pos id
//                        convergence = model.UpdateSystem(cs, imuIncliOld);

            model.GetSystemBlock(sys[0]);
            sysk=sys[0].GetZTransferFunction(num,den);
            sys[0].GetMagnitudeAndPhase(dts,wgc,smag,sphi);

//            model.PrintZTransferFunction(dts);

        }
/*
        cs=2*(sin(1*t));
        m1.SetVelocity(cs);

        cs=m1.GetVelocity();
        imuIncli=imuIncliOld;
        if (imu.readSensor(imuIncli,imuOrien)<0)
        {
            cout << "Sensor read error !" << endl;
        }
        else
        {
            vincli= ((imuIncli-imuIncliOld)/dts);
            cout << "incli_sen: " <<  (vincli) << " , orient_sen: " << imuOrien << endl;
        }


        //velocity / velocity id
        convergence = model.UpdateSystem(cs, vincli);
        model.GetSystemBlock(sys[0]);
        sysk=sys[0].GetZTransferFunction(num,den);

        data << t << ", " << cs << ", " << vincli<< ", " << (vincli>filterSensor)  << ", " <<  (cs>filterSignal)  <<  endl;
//        data << t << ", " << pos << ", " << inc   << ", " <<  (pos>filterSignal)  <<  endl;

*/
        condata << t << ", " << kp << ", " << kd << ", " << fex   << endl;
        sysdatamp << t << ", " << (smag>filterMag) << ", " << (sphi>filterPhi) <<  endl;


        sysdatanum << t;
        sysdatanum << ", " << num.back();
        for (int i=num.size()-1; i>=0; i--)
        {
            sysdatanum << ", " << sysk*num[i];
        }
        sysdatanum << endl;

        sysdataden << t;
        sysdataden << ", " << den.back();
        for (int i=den.size()-1; i>=0; i--)
        {
            sysdataden << ", " << den[i];

        }
        sysdataden << endl;
        //        sysdatanum << ", " << smag << ", " << sphi;

        Ts.WaitSamplingTime();


    }


    //  m1.SetPosition(0);
    m1.SetVelocity(0);
    //  sleep (3);
    m1.SwitchOff();
    //  m3.SetPosition(0);

    sysdatanum.close();
    sysdataden.close();

    condata.close();


    return 0;

}


long AdaptiveStep(double target, double time)
{
    return 0;
}


