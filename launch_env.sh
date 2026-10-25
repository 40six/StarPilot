#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# On AGNOS, prefer the managed venv runtime (has required Python deps like pyzmq).
if [ -x /usr/local/venv/bin/python3 ]; then
  export PATH="/usr/local/venv/bin:${PATH}"
fi

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="19.8.1"
fi

if [ -z "$AGNOS_ACCEPTED_VERSIONS" ]; then
  export AGNOS_ACCEPTED_VERSIONS="19.8.1 19.8.2"
fi

export STAGING_ROOT="/data/safe_staging"

# Fisker Ocean fork: FW_VERSIONS is intentionally empty (single-model port, CAN fingerprint
# is authoritative) and FW_QUERY_CONFIG has zero requests. Skip the VIN/FW query at
# launch so card comes up promptly on every launch route.
export SKIP_FW_QUERY=1

# StarPilot variables (only available after StarPilot is installed to /data/openpilot)
if [ -x /data/openpilot/starpilot/system/environment_variables ]; then
  eval "$(/data/openpilot/starpilot/system/environment_variables)"
fi
