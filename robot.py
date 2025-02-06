import wpilib
import wpilib.drive
from wpilib import SmartDashboard, RobotBase
from magicbot import will_reset_to

from wpimath import applyDeadband
from rev import SparkMax, SparkMaxConfig
from wpilib import DriverStation, SendableChooser
from lemonlib.control import LemonInput
from lemonlib.preference import SmartPreference


class GloruisCreation(wpilib.TimedRobot):

    maxspeed = SmartPreference(0.8)
    defspeed = SmartPreference(0.5)
    swagadd = SmartPreference(1)
    maxswag = SmartPreference(9000)
    minswag = SmartPreference(0.1)
    swagmulti = SmartPreference(10)

    def robotInit(self):
        if RobotBase.isSimulation():
            self.frontleftmotor = wpilib.Jaguar(15)
            self.frontrightmotor = wpilib.Jaguar(18)
            self.backleftmotor = wpilib.Jaguar(5)  # 55
            self.backrightmotor = wpilib.Jaguar(12)
        else:
            BRUSHLESS = SparkMax.MotorType.kBrushless
            self.frontleftmotor = SparkMax(5, BRUSHLESS)
            self.frontrightmotor = SparkMax(50, BRUSHLESS)
            self.backleftmotor = SparkMax(51, BRUSHLESS)
            self.backrightmotor = SparkMax(52, BRUSHLESS)
        self.con1 = LemonInput(0)
        self.con2 = LemonInput(1)

        self.rightmotor = wpilib.MotorControllerGroup(
            self.frontrightmotor, self.backrightmotor
        )
        self.leftmotor = wpilib.MotorControllerGroup(
            self.frontleftmotor, self.backleftmotor
        )
        self.rightmotor.setInverted(True)

        self.robotDrive = wpilib.drive.DifferentialDrive(
            self.leftmotor, self.rightmotor
        )
        self.oldMove = 0
        self.oldRotate = 0
        self.swagLevel = 0
        self.swagPeriod = 0
        self.multi = 0.0

        self.swagDrivesel = SendableChooser()
        self.swagDrivesel.setDefaultOption("Off", False)
        self.swagDrivesel.addOption("On", True)
        SmartDashboard.putData("Swag Drive", self.swagDrivesel)

        self.multicon = SendableChooser()
        self.multicon.setDefaultOption("Off", False)
        self.multicon.addOption("On", True)
        SmartDashboard.putData("Multi Con", self.multicon)

    def swagDrive(self, moveValue, rotateValue):
        """Custom drive function that incorporates 'swag' logic
                https://www.chiefdelphi.com/t/introducing-swagdrive-the-drive-code-of-the-future/129519
        ```
        So I attended the World Championships this year. It was great seeing all these high level robots.
        However, I had a problem with them, they drove too straight. They drove with too much precision. There was no sense of “YOLO”, the robots did not drive with enough swag.
        So I decided to remedy this issue at the programming level, by creating SwagDrive. SwagDrive increases the robot’s level of swag by at least ten-fold.
        By using new and innovative algorithms (or rather “swagorithms”), SwagDrive decreases the robot’s consistency and accuracy so that when it drives on the playing field it
        looks a lot cooler.

        It is similar to ArcadeDrive with some important modifications. If the change on an axis is not larger than the “swag barrier”, it will multiplied by the
        “swag multiplier” in order to “swag up” the driver’s inputs. If the input is larger then the “swag barrier” for that particular cycle, then the robot’s “swag level”
        increases by one. If the “swag level” becomes over 9000 the robot enters a moment of ultimate swag and rotates for one “swag period” (truly a YOLO move).
        Many of these values need to be tuned and modified to achieve optimal swag.

        ```
        """

        SWAG_BARRIER = self.minswag
        SWAG_MULTIPLIER = self.swagmulti
        MAX_SWAG_LEVEL = self.maxswag
        SWAG_PERIOD = 500
        SWAG_ADD = self.swagadd

        if self.swagDrivesel.getSelected():
            moveToSend = moveValue
            rotateToSend = rotateValue

            if self.swagPeriod == 0:
                moveDiff = abs(moveValue) + abs(self.oldMove)
                rotateDiff = abs(rotateValue) + abs(self.oldRotate)

                if moveDiff < SWAG_BARRIER:
                    moveToSend = (moveDiff * SWAG_MULTIPLIER) + moveValue
                else:
                    self.swagLevel += SWAG_ADD

                if rotateDiff < SWAG_BARRIER:
                    rotateToSend = (rotateDiff * SWAG_MULTIPLIER) + rotateValue
                else:
                    self.swagLevel += SWAG_ADD

                if self.swagLevel > MAX_SWAG_LEVEL:
                    self.swagPeriod = SWAG_PERIOD
                    self.swagLevel = 0

                SmartDashboard.putNumber("Swag Level", self.swagLevel)
                SmartDashboard.putNumber("Move Diff", moveDiff)
                SmartDashboard.putNumber("Rotate Diff", rotateDiff)
                SmartDashboard.putNumber("Period", SWAG_PERIOD)
                
            else:
                moveToSend = 0
                rotateToSend = 1.0
                self.swagPeriod -= 1

            # Call arcadeDrive with modified move and rotate values

            self.robotDrive.arcadeDrive(moveToSend, rotateToSend)

            self.oldMove = moveValue
            self.oldRotate = rotateValue

        else:
            self.robotDrive.arcadeDrive(moveValue, rotateValue)

    def teleopPeriodic(self):
        """Called periodically during operator control"""
        SmartDashboard.putBoolean("Swag Mode", self.swagDrivesel.getSelected())

        maxspeed = self.maxspeed
        defspeed = self.defspeed

        if self.multicon.getSelected():
            if (
                self.con1.leftbumper()
                and self.con1.rightbumper()
                or self.con2.leftbumper()
                and self.con2.rightbumper()
            ):
                self.multi = maxspeed
            else:
                self.multi = defspeed
            if (
                applyDeadband(self.con1.lefty(), 0.15) == 0
                and applyDeadband(self.con1.rightx(), 0.15) == 0
            ):
                self.leftStick = self.con2.lefty()
                self.rightStick = self.con2.rightx()
            elif (
                applyDeadband(self.con2.lefty(), 0.15) == 0
                and applyDeadband(self.con2.rightx(), 0.15) == 0
            ):
                self.leftStick = self.con1.lefty()
                self.rightStick = self.con1.rightx()
            else:
                self.leftStick = (self.con1.lefty() + self.con2.lefty()) / 2.0
                self.rightStick = (self.con1.rightx() + self.con2.rightx()) / 2.0
        else:
            if self.con1.leftbumper() and self.con1.rightbumper():
                self.multi = maxspeed
            else:
                self.multi = defspeed
            self.leftStick = self.con1.lefty()
            self.rightStick = self.con1.rightx()

        moveValue = applyDeadband(-self.leftStick, 0.15)
        rotateValue = applyDeadband(-self.rightStick, 0.15)
        SmartDashboard.putNumber("left", moveValue)
        SmartDashboard.putNumber("right", rotateValue)
        self.swagDrive(moveValue * self.multi, rotateValue * self.multi)
