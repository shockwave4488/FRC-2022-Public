package frc.lib.util.app.math;

import frc.lib.util.app.CSVWritable;
import frc.lib.util.app.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
  double distance(final S other);

  boolean equals(final Object other);

  String toString();

  String toCSV();
}
