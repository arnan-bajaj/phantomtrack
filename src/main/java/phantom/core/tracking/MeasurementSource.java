package phantom.core.tracking;

import java.util.List;

/** Something that produces opponent sightings: a simulated camera now, a real one later. */
public interface MeasurementSource {
  /** Measurements that arrived since the last call. Never null; may be empty. */
  List<Measurement> poll(double nowSec);
}
