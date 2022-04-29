/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Comment for Git demo

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.ArmInCom;
import frc.robot.commands.ArmOutCom;
import frc.robot.commands.HerderInCom;
import frc.robot.commands.HerderOutCom;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public Joystick elevatorStick;

  private JoystickButton herdersInButton;
  private JoystickButton herdersOutButton;

  private JoystickButton herderArmInButton;
  private JoystickButton herderArmOutButton;

  public OI() {

    elevatorStick = new Joystick(RobotMap.elevatorStickCh);

    herderArmInButton = new JoystickButton(elevatorStick, RobotMap.herderArmInButtonCh);
    herderArmInButton.whenPressed(new ArmInCom());

    herderArmOutButton = new JoystickButton(elevatorStick, RobotMap.herderArmOutButtonCh);
    herderArmOutButton.whenPressed(new ArmOutCom());

    herdersInButton = new JoystickButton(elevatorStick, RobotMap.herdersInButtonCh);
    herdersInButton.whileHeld(new HerderInCom());

    herdersOutButton = new JoystickButton(elevatorStick, RobotMap.herdersOutButtonCh);
    herdersOutButton.whileHeld(new HerderOutCom());
  }

}
