/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7280.mecanum_drive_test.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import org.usfirst.frc7280.mecanum_drive_test.Constants;
import org.usfirst.frc7280.mecanum_drive_test.Robot;
import org.usfirst.frc7280.mecanum_drive_test.RobotMap;
import org.usfirst.frc7280.mecanum_drive_test.commands.ManualElevator;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public TalonSRX elevatorMaster = new TalonSRX(RobotMap.elevatorMasterMotor);
  private VictorSPX elevatorSlave = new VictorSPX(RobotMap.elevatorSlaveMotor);

  RobotMap robotMap = new RobotMap();
  public int elevatorPosition;
  public int targetPosition;

  

  public Elevator(){
    
    robotMap.TalonSRXInit(elevatorMaster, Constants.kElevatorPeakOutput); 
    
    elevatorMaster.setNeutralMode(NeutralMode.Brake);
    elevatorSlave.setNeutralMode(NeutralMode.Brake);
    
    elevatorSlave.follow(elevatorMaster); 

    // set whether you need to invert the motor to get right value
    elevatorMaster.setInverted(Constants.kMotorInverted);
    elevatorSlave.setInverted(false); // modified

    robotMap.setMotorPID(
    elevatorMaster, 
    Constants.kElevatorF, 
    Constants.kElevatorP, 
    Constants.kElevatorI, 
    Constants.kElevatorD);

    // elevatorMaster.setSelectedSensorPosition(0, Constants.kSlotIdx, Constants.kTimeoutMs);

    // current limit 
    elevatorMaster.enableCurrentLimit(true);
    elevatorMaster.configContinuousCurrentLimit(Constants.kContinueCurrentLimit, Constants.kTimeoutMs);
    elevatorMaster.configPeakCurrentLimit(Constants.kPeakCurrentLimit, Constants.kTimeoutMs);
    elevatorMaster.configPeakCurrentDuration(Constants.kpeakCurrentDuration, Constants.kTimeoutMs);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ManualElevator());
  }

  public void liftToPosition(int _position){

    /* 
    evaluate PID and peak output, separate into three part
    1. elevator going up within the first frame
    2. elevator going up to using the second lift 
    3. elevator going down
    */
    targetPosition = _position;
    elevatorPosition = elevatorMaster.getSelectedSensorPosition(Constants.kSlotIdx);
    if (_position < elevatorPosition && _position < -50000) {
      robotMap.setMotorPID(
        elevatorMaster, 
        Constants.kElevatorHigherF, 
        Constants.kElevatorHigherP, 
        Constants.kElevatorHigherI, 
        Constants.kElevatorHigherD);
      elevatorMaster.configClosedLoopPeakOutput(Constants.kSlotIdx, Constants.kElevatorHigherPeakOutput, Constants.kTimeoutMs);
    } else if ((_position < elevatorPosition && _position > -50000)) {
      robotMap.setMotorPID(
        elevatorMaster, 
        Constants.kElevatorF, 
        Constants.kElevatorP, 
        Constants.kElevatorI, 
        Constants.kElevatorD);
      elevatorMaster.configClosedLoopPeakOutput(Constants.kSlotIdx, Constants.kElevatorPeakOutput, Constants.kTimeoutMs);
    } else {
      robotMap.setMotorPID(
        elevatorMaster, 
        Constants.kElevatorDownF, 
        Constants.kElevatorDownP, 
        Constants.kElevatorDownI, 
        Constants.kElevatorDownD);
        elevatorMaster.configClosedLoopPeakOutput(Constants.kSlotIdx, Constants.kElevatorDownPeakOutput, Constants.kTimeoutMs);

    }

    elevatorMaster.set(ControlMode.Position, _position);

    SmartDashboard.putNumber("current position", elevatorMaster.getSelectedSensorPosition(Constants.kSlotIdx));
    SmartDashboard.putNumber("Target position", _position);
    SmartDashboard.putNumber("elevator output", elevatorMaster.getMotorOutputPercent());
    SmartDashboard.putNumber("elevator current", elevatorMaster.getOutputCurrent());

  }

  public void elevatorDown(){
    //这边改成速度pid会不会好些
    targetPosition = elevatorPosition + 5000;
    robotMap.setMotorPID(
        elevatorMaster, 
        Constants.kElevatorDownF, 
        Constants.kElevatorDownP, 
        Constants.kElevatorDownI, 
        Constants.kElevatorDownD);
        elevatorMaster.configClosedLoopPeakOutput(Constants.kSlotIdx, Constants.kElevatorDownPeakOutput, Constants.kTimeoutMs);
   
    elevatorMaster.set(ControlMode.Position, targetPosition);
    

  }

  public void manualRun(double _outPut){
    elevatorMaster.set(ControlMode.PercentOutput, _outPut);

  }

  public void stop(){
    elevatorMaster.set(ControlMode.PercentOutput, 0);
  }


  // use for testing
  public void testRun(double _speed){
    elevatorMaster.set(ControlMode.PercentOutput, _speed/3);
    elevatorSlave.follow(elevatorMaster);
  }

}
