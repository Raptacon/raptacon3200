import math
from pydantic import BaseModel, Field, computed_field


class SwerveDriveConsts(BaseModel):
    massKG: float = Field(
        default=63.9565,
        description="Total mass, in kilograms, of the robot. Used in PathPlanner"
    )
    MOI: float = Field(
        default=5.94175290870316,
        description="Robot's moment of inertia, in kg*m^2. Used in PathPlanner"
    )
    swerveSteerPID: tuple[float] = Field(
        default=(0.007, 0, 0),
        description="PID constants for controlling the steer motor of a swerve module"
    )
    swerveDrivePID: tuple[float] = Field(
        default=(0.1, 0, 0.1, 0),
        description="PID constants for controlling the drive motor of a swerve module"
    )

    moduleFrontLeftX: float = Field(
        default=0.31115,
        description="""
            X (+ front, - back) position of the center of the front-left swerve module
            relative to robot center, in meters
        """
    )
    moduleFrontLeftY: float = Field(
        default=0.26035,
        description="""
            Y (+ left, - right) position of the center of the front-left swerve module
            relative to robot center, in meters
        """
    )
    moduleFrontRightX: float = Field(
        default=0.31115,
        description="""
            X (+ front, - back) position of the center of the front-right swerve module
            relative to robot center, in meters
        """
    )
    moduleFrontRightY: float = Field(
        default=-0.26035,
        description="""
            Y (+ left, - right) position of the center of the front-right swerve module
            relative to robot center, in meters
        """
    )
    moduleBackLeftX: float = Field(
        default=-0.31115,
        description="""
            X (+ front, - back) position of the center of the back-left swerve module
            relative to robot center, in meters
        """
    )
    moduleBackLeftY: float = Field(
        default=0.26035,
        description="""
            Y (+ left, - right) position of the center of the back-left swerve module
            relative to robot center, in meters
        """
    )
    moduleBackRightX: float = Field(
        default=-0.31115,
        description="""
            X (+ front, - back) position of the center of the back-right swerve module
            relative to robot center, in meters
        """
    )
    moduleBackRightY: float = Field(
        default=-0.26035,
        description="""
            Y (+ left, - right) position of the center of the back-right swerve module
            relative to robot center, in meters
        """
    )

    invertGyro: bool = Field(
        default=False,
        description="""
            If True, flip the polarity of each direction of rotation for the gyro. Desired end state
            is for counter-clockwise rotations to produce positive values
        """
    )
    moduleFrontLeftInvertDrive: bool = Field(
        default=True,
        description="""
            If True, flip the polarity of each direction of rotation for the front-left module's drive motor.
            Desired end state is for counter-clockwise rotations to produce positive values
        """
    )
    moduleFrontRightInvertDrive: bool = Field(
        default=True,
        description="""
            If True, flip the polarity of each direction of rotation for the front-right module's drive motor.
            Desired end state is for counter-clockwise rotations to produce positive values
        """
    )
    moduleBackLeftInvertDrive: bool = Field(
        default=True,
        description="""
            If True, flip the polarity of each direction of rotation for the back-left module's drive motor.
            Desired end state is for counter-clockwise rotations to produce positive values
        """
    )
    moduleBackRightInvertDrive: bool = Field(
        default=True,
        description="""
            If True, flip the polarity of each direction of rotation for the back-right module's drive motor.
            Desired end state is for counter-clockwise rotations to produce positive values
        """
    )

    moduleFrontLeftInvertSteer: bool = Field(
        default=True,
        description="""
            If True, flip the polarity of each direction of rotation for the front-left module's steer motor.
            Desired end state is for counter-clockwise rotations to produce positive values
        """
    )
    moduleFrontRightInvertSteer: bool = Field(
        default=True,
        description="""
            If True, flip the polarity of each direction of rotation for the front-right module's steer motor.
            Desired end state is for counter-clockwise rotations to produce positive values
        """
    )
    moduleBackLeftInvertSteer: bool = Field(
        default=True,
        description="""
            If True, flip the polarity of each direction of rotation for the back-left module's steer motor.
            Desired end state is for counter-clockwise rotations to produce positive values
        """
    )
    moduleBackRightInvertSteer: bool = Field(
        deafult=True,
        description="""
            If True, flip the polarity of each direction of rotation for the back-right module's steer motor.
            Desired end state is for counter-clockwise rotations to produce positive values
        """
    )

    maxTranslationMPS: float = Field(
        default=4.14528, description="Maximum translation speed of the swerve drive, in meters per second"
    )

    @computed_field
    @property
    def maxAngularDPS(self) -> float:
        """
        Maximum rotational speed of the swerve drive, in degrees per second
        """
        return math.degrees(self.maxTranslationMPS / math.hypot(self.moduleFrontLeftY, self.moduleFrontLeftX))

class SwerveModuleMk4iConsts(BaseModel):
    kNominalVoltage: float = Field(default=12.0, description="expected voltage at standard system functioning")
    kDriveCurrentLimit: int = Field(default=40, description="current limit of the drive motors, in amps")
    kSteerCurrentLimit: int = Field(default=20, description="current limit of the steer motors, in amps")
    kRampRate: float = Field(default=0.25, description="how quickly the robot is allowed to go from 0% to 100% velocity")
    kTicksPerRotation: int = Field(default=1, description="how many encoder ticks correspond to one full rotation of a motor")
    kCanStatusFrameHz: int = Field(default=10, description="the frame rate, in hertz, for updating the CAN network")
    quadratureMeasurementRateMs: int = Field(default=10, description="the frame rate, in milliseconds, in which quadrature encoder readings are taken")
    quadratureAverageDepth: int = Field(default=2, description="hell if I know")
    numDriveMotors: int = Field(default=1, description="how many drive motors exist within a single swerve module")
    motorType: str = Field(
        default="NEO",
        description="classification for the motors used in the swerve module. Should be an option in wpimath.system.plant.DCMotor"
    )
    encoderCalibration: float = Field(default=None, description="value of the absolute encoder when wheel is at 0 position")

    wheelDiameter: float = Field(default=0.10033, description="diameter of the drive wheel, in meters")
    driveGearRatio: float = Field(default=8.14, description="ratio between the input to the drive motor gear system and the output")
    steerGearRatio: float = Field(default=150 / 7, description="ratio between the input to the steer motor gear system and the output")

    @computed_field
    @property
    def drivePositionConversionFactor(self) -> float:
        """
        Multiplier to convert from drive motor rotations to change in position.
        Resulting units are meters per rotation
        """
        return (math.pi * self.wheelDiameter) / (self.driveGearRatio * self.kTicksPerRotation)

    @computed_field
    @property
    def driveVelocityConversionFactor(self) -> float:
        """
        Multiplier to convert from drive motor rotations to current velocity.
        Resulting units are meters per second
        """
        return self.drivePositionConversionFactor / 60.0

    @computed_field
    @property
    def steerPositionConversionFactor(self) -> float:
        """
        Multiplier to convert from steer motor rotations to change in position.
        Resulting units are meters per rotation
        """
        return 360 / (self.steerGearRatio * self.kTicksPerRotation)

    @computed_field
    @property
    def steerVelocityConversionFactor(self) -> float:
        """
        Multiplier to convert from steer motor rotations to current velocity.
        Resulting units are meters per second
        """
        return self.steerPositionConversionFactor / 60.0


    moduleType: str = Field(default="Mk4i_L2", description="name of the module")
