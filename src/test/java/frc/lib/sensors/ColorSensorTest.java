package frc.lib.sensors;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;
import org.powermock.api.mockito.PowerMockito;
import org.powermock.core.classloader.annotations.PrepareForTest;
import org.powermock.modules.junit4.PowerMockRunner;

@RunWith(PowerMockRunner.class)
@PrepareForTest({ColorSensor.class})
public class ColorSensorTest {

  // Mock instances needed to test a ColorSensor
  @Mock private ColorSensorV3 mockHardwareSensor;

  // The actual ColorSensor to test (will be constructed for each test)
  private ColorSensor colorSensor;

  // Allowable error in floating point comparisons
  // Needed because the precision is not perfect
  private static final double colorEpsilon = 0.001;

  @Before
  public void setup() throws Exception {
    MockitoAnnotations.initMocks(this);
    PowerMockito.whenNew(ColorSensorV3.class).withAnyArguments().thenReturn(mockHardwareSensor);
    colorSensor = new ColorSensor(Port.kMXP);
  }

  /** Test that the ColorSensor class can successfully initialize */
  @Test
  public void testConstructor() {
    assertNotNull(colorSensor);
  }

  /** Test that the ColorSensor class doesn't swizzle the R, G, or B values */
  @Test
  public void testNoRGBSwizzle() {
    Color exampleColor = new Color(0.2, 0.5, 0.8);
    when(mockHardwareSensor.getColor()).thenReturn(exampleColor);
    assertEquals(0.2, colorSensor.getRed(), colorEpsilon);
    assertEquals(0.5, colorSensor.getGreen(), colorEpsilon);
    assertEquals(0.8, colorSensor.getBlue(), colorEpsilon);
    verify(mockHardwareSensor, times(3)).getColor();
  }

  /**
   * Test a positive match when the sensor returns the measured value "exactly" (within double
   * precision tolerance)
   */
  @Test
  public void testSolidBlue() {
    when(mockHardwareSensor.getColor()).thenReturn(ColorSensor.kBlueTarget);
    assertEquals(ColorSensor.ColorType.BLUE, colorSensor.currentColor());
    verify(mockHardwareSensor, times(1)).getColor();
  }

  /** Test a positive match for the color blue (as measured by the sensor) */
  @Test
  public void testKindaBlue() {
    Color blueColor = new Color(0.16, 0.4, 0.45);
    when(mockHardwareSensor.getColor()).thenReturn(blueColor);
    assertEquals(ColorSensor.ColorType.BLUE, colorSensor.currentColor());
    verify(mockHardwareSensor, times(1)).getColor();
  }

  /** Test that a color far aways from the wheel is correctly identified as UNKNOWN */
  @Test
  public void testWayOff() {
    Color invalidColor = new Color(0.7, 0.7, 0.7);
    when(mockHardwareSensor.getColor()).thenReturn(invalidColor);
    assertEquals(ColorSensor.ColorType.UNKNOWN, colorSensor.currentColor());
    verify(mockHardwareSensor, times(1)).getColor();
  }

  /**
   * Test that a color is rejected if the color matching algorithm doesn't have enough confidence in
   * the result
   */
  @Test
  public void testConfidenceFailure() {
    // Hand-selected to be "almost blue" - as in, the sensor detects blue but with low confidence.
    // There may be a better way to test the confidence intervals, but this tests the code path
    // on our end, at least, if not our color targets.
    Color almostBlue = new Color(0.2, 0.2, 0.4);
    when(mockHardwareSensor.getColor()).thenReturn(almostBlue);
    assertEquals(ColorSensor.ColorType.UNKNOWN, colorSensor.currentColor());
    verify(mockHardwareSensor, times(1)).getColor();
  }
}
