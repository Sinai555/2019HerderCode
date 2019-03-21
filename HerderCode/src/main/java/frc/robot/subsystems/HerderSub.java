/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.HerdersStopCom;

public class HerderSub extends Subsystem {

  // Decalre the variables for the herder motors
  private WPI_VictorSPX topHerderMotor;
  private WPI_VictorSPX bottomHerderMotor;

  public HerderSub() {

    // Instatiate the motor and encoder objects and assign values for the subsystem

    topHerderMotor = new WPI_VictorSPX(RobotMap.topHerderMotor);
    bottomHerderMotor = new WPI_VictorSPX(RobotMap.bottomHerderMotor);
  }

  // Stop the herder motors

  public void stopHerders() {
    topHerderMotor.set(0);
    bottomHerderMotor.set(0);
  }

  // Run herder motors to take in ball
  public void herdersIn() {
    bottomHerderMotor.set(1);
    topHerderMotor.set(1);
  }

  // Run the herder motors to spit out the ball
  public void herdersOut() {
    bottomHerderMotor.set(-1);
    topHerderMotor.set(-1);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new HerdersStopCom());
  }
}
