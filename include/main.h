/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout the project
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

/**
 * If defined, enables use of PROS LCD functions
 */
#define PROS_USE_SIMPLE_NAMES

/**
 * If defined, some additional literal operators are defined
 */
#define PROS_USE_LITERALS

#include "api.h"

/**
 * Standard library includes
 */
#include <string>
#include <vector>
#include <algorithm>
#include <functional>
#include <memory>

/**
 * Project headers
 */
#include "config.hpp"
#include "utils.hpp"
#include "pid.hpp"
#include "odometry.hpp"
#include "pure_pursuit.hpp"
#include "mechanisms.hpp"
#include "driver_control.hpp"
#include "autonomous.hpp"
#include "calibration.hpp"
#include "telemetry.hpp"

/**
 * Prototypes for competition functions
 */
#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * Use literals for units
 */
using namespace pros::literals;
#endif

#endif  // _PROS_MAIN_H_
