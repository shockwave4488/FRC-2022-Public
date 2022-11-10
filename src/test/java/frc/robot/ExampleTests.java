package frc.robot;

import static org.junit.Assert.*;

import org.junit.Ignore;
import org.junit.Test;

public class ExampleTests {

  @Test
  public void test() {
    assertTrue(true);
  }

  @Test
  public void testTwo() {
    assertTrue(true);
  }

  @Ignore("In case you need to ignore a failing test...")
  @Test
  public void testSkip() {
    assertTrue(false);
  }
}
