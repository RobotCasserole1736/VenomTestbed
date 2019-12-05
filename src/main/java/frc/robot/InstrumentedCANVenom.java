package frc.robot;


import com.playingwithfusion.CANVenom;

import frc.lib.DataServer.Signal;

public class InstrumentedCANVenom extends CANVenom {

    Signal motorBusVoltage;
    Signal motorOutputCurrent;
    Signal motorOutputVoltage;
    Signal motorPosition;
    Signal motorSpeed;
    Signal motorTemperature;
    Signal motorCommandSpeed;
    Signal motorCommandPos;

    LoopTiming lt;


    public InstrumentedCANVenom(int motorID, String motorName){
        super(motorID);

        motorBusVoltage = new Signal(motorName+"_BusVoltage", "V");
        motorOutputCurrent = new Signal(motorName+"_OutputCurrent", "A");
        motorOutputVoltage = new Signal(motorName+"_OutputVoltage", "V");
        motorPosition = new Signal(motorName+"_Position", "rev");
        motorSpeed = new Signal(motorName+"_BusVoltage", "RPM");
        motorTemperature = new Signal(motorName+"_Temperature", "C");
        motorCommandSpeed = new Signal(motorName+"_Comand", "RPM");
        motorCommandPos = new Signal(motorName+"_Comand", "rev");
        lt = LoopTiming.getInstance();
    }

    public void logData(){
        double sample_time = lt.getLoopStartTimeSec();
        motorBusVoltage.addSample(sample_time, super.getBusVoltage());
        motorOutputCurrent.addSample(sample_time, super.getOutputCurrent());
        motorOutputVoltage.addSample(sample_time, super.getOutputVoltage());
        motorPosition.addSample(sample_time, super.getPosition());
        motorSpeed.addSample(sample_time, super.getSpeed());
        motorTemperature.addSample(sample_time, super.getTemperature());
    }

    @Override
    public void setCommand(CANVenom.ControlMode mode, double command){
        double sample_time = lt.getLoopStartTimeSec();

        super.setCommand(mode, command);

        if(mode == CANVenom.ControlMode.PositionControl){
            motorCommandPos.addSample(sample_time, command);
        } else if (mode == ControlMode.SpeedControl){
            motorCommandSpeed.addSample(sample_time, command);
        }
    }

    @Override
    public void setCommand(CANVenom.ControlMode mode, double command, double kF, double b){
        double sample_time = lt.getLoopStartTimeSec();

        super.setCommand(mode, command, kF, b);
        
        if(mode == CANVenom.ControlMode.PositionControl){
            motorCommandPos.addSample(sample_time, command);
        } else if (mode == ControlMode.SpeedControl){
            motorCommandSpeed.addSample(sample_time, command);
        }
    }

}
    
