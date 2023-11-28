/*
 * Simple class for representing an action that's executable by a robot.
 *
 * Author: Rick Coogle
 */

#include "robotaction.h"

const double RobotAction::DEFAULT_DURATION = 1000.0;
const double RobotAction::QUARTER_SEC = 250.0;

RobotAction::RobotAction()
{
    type = ACT_NULL;
    dDuration = 0.0;
}
