import pytest
import rev
from raptacon3200.utils import sparkMaxUtils


# Need to change names etc per https://docs.revrobotics.com/revlib/24-to-25


def test_sparkmax_naming():
    """Test that SparkMax is the >=2025 name, not CANSparkMax"""

    # Should work - new naming convention
    assert hasattr(rev, 'SparkMax'), "rev.SparkMax should exist"
    assert hasattr(rev, 'SparkLowLevel'), "rev.SparkLowLevel should exist"

    # Should not work - old naming convention
    assert not hasattr(rev, 'CANSparkMax'), "rev.CANSparkMax should not exist"
    assert not hasattr(rev, 'CANSparkMaxLowLevel'), "rev.CANSparkMaxLowLevel should not exist"


def test_sparkmax_creation():
    """Test that we can create a SparkMax instance with proper parameters"""

    try:
        motor = rev.SparkMax(1, rev.SparkLowLevel.MotorType.kBrushless)
        assert motor is not None, "Should be able to create SparkMax instance"
    except Exception as e:
        pytest.fail(f"Failed to create SparkMax: {str(e)}")


def test_motor_type_enum():
    """Test that MotorType enum exists and has expected values"""

    assert hasattr(rev.SparkLowLevel.MotorType, 'kBrushless'), "kBrushless should exist"
    assert hasattr(rev.SparkLowLevel.MotorType, 'kBrushed'), "kBrushed should exist"


def test_sparkmax_utils():
    """Test that we can use the sparkMaxUtils to configure the CAN rates (where still applicable)"""

    try:
        motor = rev.SparkMax(1, rev.SparkLowLevel.MotorType.kBrushless)
        sparkMaxUtils.configureSparkMaxCanRates(motor)
        assert motor is not None, "Should be able to create SparkMax instance"
    except Exception as e:
        pytest.fail(f"Failed to create SparkMax: {str(e)}")
