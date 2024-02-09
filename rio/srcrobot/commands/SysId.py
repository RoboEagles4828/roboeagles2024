from commands2.sysid import SysIdRoutine
from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand
from subsystems.Swerve import Swerve

class DriveSysId(SequentialCommandGroup):

    routine: SysIdRoutine

    def __init__(self, s_Swerve: Swerve):
        self.addCommands(s_Swerve)

        self.routine = SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(
                s_Swerve.driveMotorsVoltage,
                s_Swerve.logDriveMotors,
                s_Swerve,
                name="SwerveDrive"
            )
        )

        self.quasiStaticForward = self.routine.quasistatic(SysIdRoutine.Direction.kForward)
        self.quasiStaticReverse = self.routine.quasistatic(SysIdRoutine.Direction.kReverse)
        self.dynamicForward = self.routine.dynamic(SysIdRoutine.Direction.kForward)
        self.dynamicReverse = self.routine.dynamic(SysIdRoutine.Direction.kReverse)

        self.stop = InstantCommand(s_Swerve.stop, s_Swerve)
        self.waitTwoSeconds = WaitCommand(2)

        self.resetAngleMotors = InstantCommand(s_Swerve.resetModulesToAbsolute)

        self.addCommands(
            self.resetAngleMotors,
            self.stop,

            self.quasiStaticForward,
            self.stop,
            self.waitTwoSeconds,

            self.quasiStaticReverse,
            self.stop,
            self.waitTwoSeconds,

            self.dynamicForward,
            self.stop,
            self.waitTwoSeconds,

            self.dynamicReverse,
            self.stop,
            self.waitTwoSeconds
        )