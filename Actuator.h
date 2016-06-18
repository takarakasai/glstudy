
#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "dp_type.h"

class Actuator {
private:
  Dp::Math::real& ovalue_; /* offset value */
  Dp::Math::real& value_;
  Dp::Math::real& veloc_;
  Dp::Math::real& accel_;
  Dp::Math::real& acfrc_; /* actuational force */

  Dp::Math::real& exfrc_; /* external force */

public:
  Actuator(Dp::Math::real& ovalue,
           Dp::Math::real& value,
           Dp::Math::real& veloc,
           Dp::Math::real& accel,
           Dp::Math::real& acfrc,
           Dp::Math::real& exfrc
          ) : ovalue_(ovalue), value_(value), veloc_(veloc), accel_(accel),
              acfrc_(acfrc), exfrc_(exfrc) {};
  virtual ~Actuator() {};

  inline const Dp::Math::real& GetOffsetValue() { return ovalue_;};
  inline const Dp::Math::real& GetValue()       { return value_;};
  inline const Dp::Math::real& GetVeloc()       { return veloc_;};
  inline const Dp::Math::real& GetAccel()       { return accel_;};
  inline const Dp::Math::real& GetActForce()    { return acfrc_;};
  inline const Dp::Math::real& GetExtForce()    { return exfrc_;};

  inline void SetOffsetValue(Dp::Math::real& ovalue) { ovalue_ = ovalue;};
  inline void SetValue(Dp::Math::real& value)        { value_ = value;};
  inline void SetVeloc(Dp::Math::real& veloc)        { veloc_ = veloc;};
  inline void SetAccel(Dp::Math::real& accel)        { accel_ = accel;};
  inline void SetActForce(Dp::Math::real& force)     { acfrc_ = force;};
};

#endif
