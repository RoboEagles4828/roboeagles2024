#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import wpimath
import phoenix5

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """


        self.controller = wpilib.XboxController(0)
        self.left = phoenix5.TalonFX(14)
        self.right = phoenix5.TalonFX(13)
        self.timer = wpilib.Timer()


        self.left.setNeutralMode(phoenix5.NeutralMode.Brake)
        self.right.setNeutralMode(phoenix5.NeutralMode.Brake)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.restart()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""

        # Drive for two seconds
      

    def teleopInit(self):
        """This function is called once each time the robot enters teleoperated mode."""

    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        self.left.set(
            phoenix5.TalonFXControlMode.PercentOutput,
            wpimath.applyDeadband(-self.controller.getLeftY(), 0.1)
        )
        self.right.set(
            phoenix5.TalonFXControlMode.PercentOutput,
            wpimath.applyDeadband(-self.controller.getRightY(), 0.1)
        )
        
        # these are supposed to make the amp power be shown on the SmartDashboard, but for some reason it makes the whole thing not work
        # wpilib.SmartDashboard.putNumber("left amps", self.left.getStatorCurrent()) 
        # wpilib.SmartDashboard.putNumber("right amps", self.right.getStatorCurrent())
        

    def testInit(self):
        """This function is called once each time the robot enters test mode."""

    def testPeriodic(self):
        """This function is called periodically during test mode."""


if __name__ == "__main__":
    wpilib.run(MyRobot)
