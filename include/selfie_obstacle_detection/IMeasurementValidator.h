/**
 * Copyright (c) 2019 Adam Gotlib
 * This code is licensed under MIT license (see LICENSE for details)
 **/

#ifndef I_MEASUREMENT_VALIDATOR_H
#define I_MEASUREMENT_VALIDATOR_H

namespace selfie_obstacle_detection
{

class IMeasurementValidator
{
public:
  virtual ~IMeasurementValidator() { }
  virtual bool isValid(float measurement) = 0;
}; // class IMeasurementValidator

} // namespace selfie_obstacle_detection

#endif /* I_MEASUREMENT_VALIDATOR_H */
