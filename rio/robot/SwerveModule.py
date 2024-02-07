from phoenix6.controls import DutyCycleOut, PositionVoltage, VelocityVoltage
from phoenix6.hardware import CANcoder, TalonFX

from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from lib.mathlib.conversions import Conversions
from lib.util.SwerveModuleConstants import SwerveModuleConstants
from constants import Constants
from robot import Robot

from wpimath.units import radiansToRotations, rotationsToRadians

class SwerveModule:
    moduleNumber: int
    angleOffset: Rotation2d

    mAngleMotor: TalonFX
    mDriveMotor: TalonFX
    angleEncoder: CANcoder

    driveFeedForward = SimpleMotorFeedforwardMeters(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA)

    driveDutyCycle: DutyCycleOut = DutyCycleOut(0)
    driveVelocity: VelocityVoltage = VelocityVoltage(0)

    anglePosition: PositionVoltage = PositionVoltage(0)

    def __init__(self, moduleNumber: int, moduleConstants: SwerveModuleConstants):
        self.moduleNumber = moduleNumber
        self.angleOffset = moduleConstants.angleOffset

        self.angleEncoder = CANcoder(moduleConstants.cancoderID)
        self.angleEncoder.configurator.apply(Robot.ctreConfigs.swerveCANcoderConfig)

        self.mAngleMotor = TalonFX(moduleConstants.angleMotorID)
        self.mAngleMotor.configurator.apply(Robot.ctreConfigs.swerveAngleFXConfig)
        self.resetToAbsolute()

        self.mDriveMotor = TalonFX(moduleConstants.driveMotorID)
        self.mDriveMotor.configurator.apply(Robot.ctreConfigs.swerveDriveFXConfig)
        self.mDriveMotor.configurator.set_position(0.0)

    def setDesiredState(self, desiredState: SwerveModuleState, isOpenLoop: bool):
        desiredState = SwerveModuleState.optimize(desiredState, self.getState().angle)
        self.mAngleMotor.set_control(self.anglePosition.with_position(radiansToRotations(desiredState.angle.radians())))
        self.setSpeed(desiredState, isOpenLoop)

    def setSpeed(self, desiredState: SwerveModuleState, isOpenLoop: bool):
        if isOpenLoop:
            self.driveDutyCycle.output = desiredState.speed / Constants.Swerve.maxSpeed
            self.mDriveMotor.set_control(self.driveDutyCycle)
        else:
            self.driveVelocity.velocity = Conversions.MPSToRPS(desiredState.speed, Constants.Swerve.wheelCircumference)
            self.driveVelocity.feed_forward = self.driveFeedForward.calculate(desiredState.speed)
            self.mDriveMotor.set_control(self.driveVelocity)

    def getCANcoder(self):
        return Rotation2d(rotationsToRadians(self.angleEncoder.get_absolute_position().value_as_double))

    def resetToAbsolute(self):
        absolutePosition = radiansToRotations(self.getCANcoder().radians()) - rotationsToRadians(self.angleOffset.radians())
        self.mAngleMotor.set_position(absolutePosition)

    def getState(self):
        return SwerveModuleState(
            Conversions.RPSToMPS(self.mDriveMotor.get_velocity().value_as_double, Constants.Swerve.wheelCircumference),
            Rotation2d(rotationsToRadians(self.mAngleMotor.get_position().value_as_double))
        )

    def getPosition(self):
        return SwerveModulePosition(
            Conversions.rotationsToMeters(self.mDriveMotor.get_position().value_as_double, Constants.Swerve.wheelCircumference),
            Rotation2d(rotationsToRadians(self.mAngleMotor.get_position().value_as_double))
        )
    


