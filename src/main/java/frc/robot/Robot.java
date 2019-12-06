/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.playingwithfusion.CANVenom;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
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
  int testSeqStepNum_prev = 0;
  double testStepStartTime = 0;

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
  public void disabledInit() {
    dataServer.logger.stopLogging();
  }

  @Override
  public void teleopInit() {
    dataServer.logger.startLoggingTeleop();
    testSeqStepNum = 0;
    testSeqStepNum_prev = 0;
    testStepStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void teleopPeriodic() {
    loopTiming.markLoopStart();

    double timeSinceStepStart = Timer.getFPGATimestamp() - testStepStartTime;

    //Do action for current test step, and calculate next step.
    if(testSeqStepNum == 0){
      //Command open-loop duty cycle commands
      double curTime = Timer.getFPGATimestamp();
      double cmd1 = Math.sin(2*Math.PI*0.25*curTime + 0*Math.PI/6);
      double cmd2 = Math.sin(2*Math.PI*0.25*curTime + 1*Math.PI/6);
      double cmd3 = Math.sin(2*Math.PI*0.25*curTime + 2*Math.PI/6);
      double cmd4 = Math.sin(2*Math.PI*0.25*curTime + 3*Math.PI/6);

      motor1.set(cmd1);
      //motor2.set(cmd2);
      //motor3.set(cmd3);
      //motor4.set(cmd4);

      //After ten seconds of this, move on.
      if(timeSinceStepStart > 10.0){
        testSeqStepNum = 1;
      }

    } else if (testSeqStepNum == 1){
      //Voltage open-loop commands
      double curTime = Timer.getFPGATimestamp();
      double voltCmd1 = 11.0*Math.sin(2*Math.PI*0.3*curTime + 2*Math.PI/6);
      double voltCmd2 = 11.0*Math.sin(2*Math.PI*0.3*curTime + 4*Math.PI/6);
      double voltCmd3 = 11.0*Math.sin(2*Math.PI*0.3*curTime + 6*Math.PI/6);
      double voltCmd4 = 11.0*Math.sin(2*Math.PI*0.3*curTime + 8*Math.PI/6);
    
      motor1.setVoltage(voltCmd1);
      //motor2.setVoltage(voltCmd2);
      //motor3.setVoltage(voltCmd3);
      //motor4.setVoltage(voltCmd4);
    
      //After ten seconds of this, move on.
      if(timeSinceStepStart > 10.0){
        testSeqStepNum = 2;

        //One time, set new PID constants for step 2.
        motor1.setPID(1.5, 0, 0, 0.184, 0);
        //motor2.setPID(1.5, 0, 0, 0.184, 0);
        //motor3.setPID(1.5, 0, 0, 0.184, 0);
        //motor4.setPID(1.5, 0, 0, 0.184, 0);
      }


    } else if (testSeqStepNum == 2){
      //Speed Closed Loop Commands
      double curTime = Timer.getFPGATimestamp();
      double speedCmd1 = 1000.0*Math.sin(2*Math.PI*0.3*curTime + 1*Math.PI/6);
      double speedCmd2 = 2000.0*Math.sin(2*Math.PI*0.3*curTime + 3*Math.PI/6);
      double speedCmd3 = 3000.0*Math.sin(2*Math.PI*0.3*curTime + 4*Math.PI/6);
      double speedCmd4 = 4000.0*Math.sin(2*Math.PI*0.3*curTime + 6*Math.PI/6);
    
      motor1.setCommand(CANVenom.ControlMode.SpeedControl, speedCmd1);
      //motor2.setCommand(CANVenom.ControlMode.SpeedControl, speedCmd2);
      //motor3.setCommand(CANVenom.ControlMode.SpeedControl, speedCmd3);
      //motor4.setCommand(CANVenom.ControlMode.SpeedControl, speedCmd4);
    
      //After ten seconds of this, move on.
      if(timeSinceStepStart > 10.0){
        testSeqStepNum = 3;

        //One time, set new PID constants for step 3.
        motor1.setPID(1.5, 0, 0, 0.184, 0);
        //motor2.setPID(1.5, 0, 0, 0.184, 0);
        //motor3.setPID(1.5, 0, 0, 0.184, 0);
        //motor4.setPID(1.5, 0, 0, 0.184, 0);
      }
      
      
    } else if (testSeqStepNum == 3){
      //position Closed Loop Commands
      double curTime = Timer.getFPGATimestamp();
      double posCmd1 = 10.0*Math.sin(2*Math.PI*0.1*curTime + 0*Math.PI/6);
      double posCmd2 = 12.0*Math.sin(2*Math.PI*0.1*curTime + 1*Math.PI/6);
      double posCmd3 = 14.0*Math.sin(2*Math.PI*0.1*curTime + 2*Math.PI/6);
      double posCmd4 = 16.0*Math.sin(2*Math.PI*0.1*curTime + 3*Math.PI/6);
    
      motor1.setCommand(CANVenom.ControlMode.PositionControl, posCmd1);
      //motor2.setCommand(CANVenom.ControlMode.PositionControl, posCmd2);
      //motor3.setCommand(CANVenom.ControlMode.PositionControl, posCmd3);
      //motor4.setCommand(CANVenom.ControlMode.PositionControl, posCmd4);
    
      //After ten seconds of this, move on.
      if(timeSinceStepStart > 10.0){
        testSeqStepNum = 4;
      }
    } else {
      //Test completed, do nothing.
    }

    //move to next test step
    if(testSeqStepNum_prev != testSeqStepNum){
      testStepStartTime = Timer.getFPGATimestamp();
      testSeqStepNum_prev = testSeqStepNum;
    }

    motor1.logData();
    
    //Log RIO & testboard stats
    double sampleTimeMs = loopTiming.getLoopStartTimeSec()*1000.0;
    rioCurrDrawLoadSig.addSample(sampleTimeMs, pdp.getTotalCurrent());
    rioBattVoltLoadSig.addSample(sampleTimeMs, pdp.getVoltage());  
    rioCANBusUsagePctSig.addSample(sampleTimeMs, RobotController.getCANStatus().percentBusUtilization);

    loopTiming.markLoopEnd();
  }

}
