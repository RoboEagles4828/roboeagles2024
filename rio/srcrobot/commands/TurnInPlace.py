from commands2 import Command
from constants import Constants
from wpimath.controller import ProfiledPIDControllerRadians
from subsystems.Swerve import Swerve
from wpimath.geometry import Translation2d, Rotation2d
import math

class TurnInPlace(Command):
    def __init__(self, s_Swerve, desiredRotationSup):
        self.addRequirements(s_Swerve)

        self.controller = ProfiledPIDControllerRadians(0.1, 0.0, 0.0, Constants.AutoConstants.kThetaControllerConstraints)
        self.s_Swerve: Swerve = s_Swerve
        
        self.desiredRotationSup = desiredRotationSup

    def initialize(self):
        self.controller.setGoal(self.desiredRotationSup().radians())
        self.controller.setTolerance(math.radians(1))

    def execute(self):
        value = self.controller.calculate(-self.s_Swerve.gyro.getYaw())
        self.s_Swerve.drive(Translation2d(0, 0), math.radians(value), False, False)

    def end(self, interrupted: bool):
        self.s_Swerve.drive(Translation2d(0, 0), 0.0, False, False)

    def isFinished(self) -> bool:
        return self.controller.atGoal()