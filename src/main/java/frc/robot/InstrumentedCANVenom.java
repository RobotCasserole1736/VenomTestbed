package frc.robot;


import com.playingwithfusion.CANVenom;

import frc.lib.DataServer.Signal;

public class InstrumentedCANVenom extends CANVenom {

    Signal controllerBusVoltage;
    Signal controllerOutputCurrent;
    Signal controllerOutputVoltage;
    Signal motorPosition;
    Signal motorSpeed;
    Signal motorTemperature;
    Signal motorCommandSpeed;
    Signal motorCommandPos;

    LoopTiming lt;


    public InstrumentedCANVenom(int motorID, String motorName){
        super(motorID);

        controllerBusVoltage = new Signal(motorName+"_BusVoltage", "V");
        controllerOutputCurrent = new Signal(motorName+"_OutputCurrent", "A");
        controllerOutputVoltage = new Signal(motorName+"_OutputVoltage", "V");
        motorPosition = new Signal(motorName+"_Position", "rev");
        motorSpeed = new Signal(motorName+"_Speed", "RPM");
        motorTemperature = new Signal(motorName+"_Temperature", "C");
        motorCommandSpeed = new Signal(motorName+"_SpdCommand", "RPM");
        motorCommandPos = new Signal(motorName+"_PosCommand", "rev");
        lt = LoopTiming.getInstance();
    }

    public void logData(){
        double sampleTimeMs = lt.getLoopStartTimeSec()*1000.0;
        controllerBusVoltage.addSample(sampleTimeMs, super.getBusVoltage());
        controllerOutputCurrent.addSample(sampleTimeMs, super.getOutputCurrent());
        controllerOutputVoltage.addSample(sampleTimeMs, super.getOutputVoltage());
        motorPosition.addSample(sampleTimeMs, super.getPosition());
        motorSpeed.addSample(sampleTimeMs, super.getSpeed());
        motorTemperature.addSample(sampleTimeMs, super.getTemperature());
    }

    @Override
    public void setCommand(CANVenom.ControlMode mode, double command){
        double sampleTimeMs = lt.getLoopStartTimeSec()*1000.0;

        super.setCommand(mode, command);

        if(mode == CANVenom.ControlMode.PositionControl){
            motorCommandPos.addSample(sampleTimeMs, command);
        } else if (mode == ControlMode.SpeedControl){
            motorCommandSpeed.addSample(sampleTimeMs, command);
        }
    }

    @Override
    public void setCommand(CANVenom.ControlMode mode, double command, double kF, double b){
        double sampleTimeMs = lt.getLoopStartTimeSec()*1000.0;

        super.setCommand(mode, command, kF, b);
        
        if(mode == CANVenom.ControlMode.PositionControl){
            motorCommandPos.addSample(sampleTimeMs, command);
        } else if (mode == ControlMode.SpeedControl){
            motorCommandSpeed.addSample(sampleTimeMs, command);
        }
    }

}
    
