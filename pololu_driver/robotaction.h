/*
 * Header for a simple class for representing an action that's executable by a robot.
 *
 * Author: Rick Coogle
 */

#ifndef ROBOTACTION_H
#define ROBOTACTION_H

class RobotAction
{

public:
    RobotAction();

    enum ActionType {
        ACT_NULL,
        ACT_FORWARD,
        ACT_BACK,
        ACT_TURN_LEFT,
        ACT_TURN_RIGHT
    };

    ActionType GetType()                 { return type;             }
    void SetType(ActionType newType)     { type = newType;          }
    double GetDuration()                 { return dDuration;        }
    void SetDuration(double newDuration) { dDuration = newDuration; }

    static const double DEFAULT_DURATION;
    static const double QUARTER_SEC;

private:
    ActionType type;    // type of action
    double dDuration;   // duration, in ms
};

#endif // ROBOTACTION_H
