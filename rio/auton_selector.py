import wpilib
from hardware_interface.armcontroller import ArmController
from hardware_interface.drivetrain import DriveTrain
from hardware_interface.subsystems.drive_subsystem import DriveSubsystem
from hardware_interface.subsystems.arm_subsystem import ArmSubsystem
from hardware_interface.commands.drive_commands import *
from hardware_interface.commands.arm_commands import *
import time

from pathplannerlib.auto import AutoBuilder, PathPlannerAuto

from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from typing import List

class AutonSelector():
    def __init__(self, arm_controller: ArmController, drive_train: DriveTrain):
        self.arm_controller = arm_controller
        self.drive_train = drive_train
        self.TAXI = "Taxi Auton"
        # self.HIGH_PLACE = "High Place Auton"
        # self.HIGH_TAXI = "High Taxi Auton"
        # self.CHARGE = "Charge Auton"
        # self.HIGH_CHARGE = "High Charge Auton"
        # self.MID_TAXI = "Mid Taxi Auton"
        # self.CUBE_HIGH_TAXI = "Cube High Taxi Auton"
        # self.CUBE_HIGH_PLACE = "Cube High Place Auton"
        # self.MID_PLACE = "Mid Place Auton"
        # self.MID_CHARGE = "Mid Charge Auton"
        # self.MID_TAXI_B = "Mid Taxi BUMP Auton"
        # self.HIGH_TAXI_B = "High Taxi BUMP Auton"
        # self.CUBE_HIGH_TAXI_B = "Cube High Taxi BUMP Auton"
        # self.TAXI_AUTON_B = "Taxi Auton BUMP"
        self.TRAJ = "Trajectory Auton"
        self.PATHPLANNER = "Pathplanner Auton"

        self.autonChooser = wpilib.SendableChooser()
        self.autonChooser.setDefaultOption("Taxi CLEAN Auton", self.TAXI)
        # self.autonChooser.addOption("Taxi BUMP Auton", self.TAXI_AUTON_B)
        # self.autonChooser.addOption("High Place Auton", self.HIGH_PLACE)
        # self.autonChooser.setDefaultOption("High Taxi CLEAN Auton", self.HIGH_TAXI)
        # self.autonChooser.addOption("High Taxi BUMP Auton", self.HIGH_TAXI_B)
        # self.autonChooser.addOption("Charge Auton", self.CHARGE)
        # self.autonChooser.addOption("High Charge Auton", self.HIGH_CHARGE)
        self.autonChooser.addOption("Trajectory Auton", self.TRAJ)
        self.autonChooser.addOption("Pathplanner Auton", self.PATHPLANNER)

        self.selected = self.autonChooser.getSelected()

        self.start = 0
        self.turn_done = False
        self.first_pitch = False
        
        self.command = DoNothingCommand()
        
        self.drive_subsystem = DriveSubsystem(self.drive_train)
        self.arm_subsystem = ArmSubsystem(self.arm_controller)

    def run(self):
        self.selected = self.autonChooser.getSelected()
        
        autons = {
            self.TAXI: self.taxi_auton("clean"),
            # self.TAXI_AUTON_B: self.taxi_auton("bump"),
            # self.HIGH_PLACE: self.high_place_auton(),
            # self.HIGH_TAXI: self.high_taxi_auton("clean"),
            # self.HIGH_TAXI_B: self.high_taxi_auton("bump"),
            # self.CHARGE: self.charge_auton(),
            # self.HIGH_CHARGE: self.high_charge_auton(),
            self.TRAJ: self.trajectory_auton(),
            self.PATHPLANNER: self.pathplannerAuton("1+1")
        }
        
        self.command = autons[self.selected]
        
        auton: Command = self.command
        auton.schedule()
        
    def trajectory_auton(self):
        waypoints = [
            Pose2d(1.24, 2.39, Rotation2d.fromDegrees(0.0)),
            Pose2d(2.84, 2.39, Rotation2d.fromDegrees(90.0))
        ]
        
        trajectory_command = SwerveTrajectoryCommand(
            self.drive_subsystem,
            waypoints
        )
        
        return trajectory_command
    
    def pathplannerAuton(self, auto):
        return PathPlannerAuto(auto)

    def high_place_auton(self):
        high_place_auton = ScoreCommand(self.arm_subsystem, ElevatorState.HIGH, "cone")
        return high_place_auton

    def high_taxi_auton(self, side):
        high_taxi_auton = SequentialCommandGroup(
            ScoreCommand(self.arm_subsystem, ElevatorState.HIGH, "cone"),
            TaxiAutoCommand(self.drive_subsystem, side)
        )
        return high_taxi_auton

    def taxi_auton(self, side):
        taxiAuton = TaxiAutoCommand(self.drive_subsystem, side)
        return taxiAuton    
    
    def charge_auton(self):
        chargeAuton = SequentialCommandGroup(
            DriveToChargeStationCommand(self.drive_subsystem, 10),
            BalanceOnChargeStationCommand(self.drive_subsystem, 7),
            DriveTimeAutoCommand(self.drive_subsystem, 1.0, (0.04, 0.0, 0.0)),
        )
        return chargeAuton
        
    def high_charge_auton(self):
        high_charge_auton = SequentialCommandGroup(
            ScoreCommand(self.arm_subsystem, ElevatorState.HIGH, "cone"),
            DriveToChargeStationCommand(self.drive_subsystem, 10),
            BalanceOnChargeStationCommand(self.drive_subsystem, 7),
            DriveTimeAutoCommand(self.drive_subsystem, 1.0, (0.04, 0.0, 0.0)),
        )
        return high_charge_auton


        
    



