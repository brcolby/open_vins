# IMU interval uncertainty regression

`imu_uncertainty.cc` calls the actual propagator and interpolation helper. It
checks that a per-interval acquisition uncertainty scale changes process noise,
while preserving inertial means, transition Jacobians and capture timestamps.
Build it against the same `openvins` library as the embedding adapter. Gestalt's
native CMake recipe enables this target with `GESTALT_NATIVE_REGRESSIONS=ON`.

The fork adds `ImuData.noise_variance_scale`, default 1, and advertises
`OV_IMU_VARIANCE_SCALE_VERSION=1`. Main and fast propagation retain the scale
through interpolated endpoints; ZUPT rejects affected windows. The caller owns
loss limits and must never treat this field as permission for unbounded gaps.
