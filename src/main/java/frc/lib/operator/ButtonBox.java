package frc.lib.operator;

import edu.wpi.first.wpilibj.XboxController;

public class ButtonBox {
  public XboxController button_box;

  /*
   * This is the ButtonBox class, which grabs raw booleans and number values
   * from an Xbox controller (that is how it is configured) and turns them into more obviously named
   * functions, which then are assigned to control different functions of the
   * robot.
   */

  public ButtonBox(int port) {
    button_box = new XboxController(port);
  }

  // ----------------------------------------
  // -----------GENERAL FUNCTIONS------------
  // ----------------------------------------

  public boolean button1() {
    return button_box.getRawButton(1);
  }

  public boolean button2() {
    return button_box.getRawButton(2);
  }

  public boolean button3() {
    return button_box.getRawButton(3);
  }

  public boolean button4() {
    return button_box.getRawButton(4);
  }

  public boolean button5() {
    return button_box.getRawButton(5);
  }

  public boolean button6() {
    return button_box.getRawButton(6);
  }

  public boolean button7() {
    return button_box.getRawButton(7);
  }

  public boolean button8() {
    return button_box.getRawButton(8);
  }

  public boolean button9() {
    return button_box.getRawButton(9);
  }

  public boolean button10() {
    return button_box.getRawButton(10);
  }

  public boolean button11() {
    return button_box.getRawButton(11);
  }

  public boolean button12() {
    return button_box.getRawButton(12);
  }

  public boolean button13() {
    return button_box.getRawButton(13);
  }

  public boolean button14() {
    return button_box.getRawButton(14);
  }

  public boolean button15() {
    return button_box.getRawButton(15);
  }

  public boolean button16() {
    return button_box.getRawButton(16);
  }
}
