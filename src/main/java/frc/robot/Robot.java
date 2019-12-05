/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.lib.Calibration.CalWrangler;
import frc.lib.DataServer.CasseroleDataServer;
import frc.lib.DataServer.Signal;
import frc.lib.LoadMon.CasseroleRIOLoadMonitor;
import frc.lib.WebServer.CasseroleWebServer;
import frc.robot.LoopTiming;

public class Robot extends TimedRobot {
  // Website Utilities
  CasseroleWebServer webserver;
  CalWrangler wrangler;
  CasseroleDataServer dataServer;

  // Processor metric utilities
  CasseroleRIOLoadMonitor loadMon;
  LoopTiming loopTiming;

  PowerDistributionPanel pdp;

  Signal rioCANBusUsagePctSig;
  Signal rioBattVoltLoadSig;
  Signal rioCurrDrawLoadSig;
  Signal testStepNum;

  InstrumentedCANVenom motor1;
  InstrumentedCANVenom motor2;
  InstrumentedCANVenom motor3;
  InstrumentedCANVenom motor4;


  int testSeqStepNum = 0;

  @Override
  public void robotInit() {

    loopTiming = LoopTiming.getInstance();
    loadMon = new CasseroleRIOLoadMonitor();
    pdp = new PowerDistributionPanel(0);


    //Init DUT's
    motor1 = new InstrumentedCANVenom(1, "motor1");
    motor2 = new InstrumentedCANVenom(2, "motor2");
    motor3 = new InstrumentedCANVenom(3, "motor3");
    motor4 = new InstrumentedCANVenom(4, "motor4");

    /* Init website utilties */
    webserver = new CasseroleWebServer();
    wrangler = new CalWrangler();
    dataServer = CasseroleDataServer.getInstance();

    rioCurrDrawLoadSig = new Signal("Battery Current Draw", "A");
    rioBattVoltLoadSig = new Signal("Battery Voltage", "V");
    rioCANBusUsagePctSig = new Signal("Robot CAN Bus Utilization", "pct");
    /* Fire up webserver & telemetry dataserver */
    webserver.startServer();
    dataServer.startServer();

  }

  @Override
  public void teleopInit() {
    testSeqStepNum = 0;
  }

  @Override
  public void teleopPeriodic() {
    loopTiming.markLoopStart();

    double sampleTimeMs = loopTiming.getLoopStartTimeSec()*1000.0;

    if(testSeqStepNum == 0){

    } else {
      //Test completed, do nothing.
    }



    rioCurrDrawLoadSig.addSample(sampleTimeMs, pdp.getTotalCurrent());
    rioBattVoltLoadSig.addSample(sampleTimeMs, pdp.getVoltage());  
    rioCANBusUsagePctSig.addSample(sampleTimeMs, RobotController.getCANStatus().percentBusUtilization);

    loopTiming.markLoopEnd();
  }

  @Override
  public void testPeriodic() {
  }
}
