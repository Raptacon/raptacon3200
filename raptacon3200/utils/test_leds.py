import leds
import pytest
import wpilib



# Create Mock DriverStation
class Mock_Alliance():
    kBlue = 1
    kRed = 0
class Mock_DS():
    def __init__(self):
        self.Alliance = Mock_Alliance()
        self.alliance_color = self.Alliance.kBlue
    def getAlliance(self):
        return self.alliance_color
@pytest.fixture()
def mock_ds(*args,**kwargs):
    return Mock_DS()


# Create test object
led_strip = leds.Strip([], 'test')

# First test: valid values
def test_default_colors(monkey_patch):
    monkey_patch.setattr(wpilib, "DriverStation",mock_ds)
    assert led_strip.getDefaultHue() in [243, 360] # Blue or Red
    assert led_strip.getDefaultRgb() in [
        [0, 0, 255], 
        [255, 0, 0]
    ] # Full Blue or Full Red