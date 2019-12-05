/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.lib.Calibration.CalWrangler;
import frc.lib.DataServer.CasseroleDataServer;
import frc.lib.DataServer.Signal;
import frc.lib.LoadMon.CasseroleRIOLoadMonitor;
import frc.lib.WebServer.CasseroleDriverView;
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

  Signal rioCANBusUsagePctSig;
  Signal rioDSLogQueueLenSig;
  Signal rioBattVoltLoadSig;
  Signal rioCurrDrawLoadSig;

  @Override
  public void robotInit() {

    loopTiming = LoopTiming.getInstance();
    loadMon = new CasseroleRIOLoadMonitor();

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
  public void robotPeriodic() {

  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopPeriodic() {
    loopTiming.markLoopStart();

    loopTiming.markLoopEnd();
  }

  @Override
  public void testPeriodic() {
  }
}
