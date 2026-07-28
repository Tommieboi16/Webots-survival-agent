# Self sufficient survival agent

An autonomous creature in Webots that keeps itself alive. It manages hunger, thirst and health simultaneously, forages for what it needs, and runs from a predator. Written in C for the Behavioural Robotics module.

**Food found in 91 percent of trials. Water in 94 percent. Predator escaped in 100 percent.**

## The interesting problem

Any one of these behaviours is easy alone. The difficulty is arbitration: when you are both hungry and thirsty and a predator is closing, which do you do? A fixed priority list produces a creature that starves while topping up water it does not need.

## Arbitration

A six state machine, `EXPLORE`, `SEEK_WATER`, `SEEK_FOOD`, `AVOID_PREDATOR`, `DRINK`, `EAT`, driven by weighted urgency rather than a static ordering:

```c
double health_factor  = (health < 50) ? 1.5 : 1.0;
double thirst_urgency = (100 - thirst) * 1.2 * health_factor;
double hunger_urgency = (100 - hunger) * health_factor;
```

Three consequences fall out of this:

- **Thirst outranks hunger at equal deficit** (the 1.2 coefficient), matching the shorter biological survival window
- **Low health amplifies both drives**, so a wounded creature forages more aggressively rather than dithering
- **Predator detection preempts everything**, transitioning within 0.2 s of detection

Hard floors still apply underneath the weighting: below 30 on either need, that need wins outright.

## Sensing

Two cameras with different jobs, plus a radio:

| Sensor | Purpose |
|--------|---------|
| Overhead camera | Wide scan, computes resource centroids for navigation |
| Forward camera | Confirms a resource is centred before switching to consume |
| Radio receiver | Predator proximity, independent of line of sight |

Detection is colour thresholding in RGB, green for food, cyan for water, red for the predator, with the red channel split left/right so the creature knows which way to flee rather than just that it should.

## Building

Requires Webots. The controller is C:

```
WEBOTSLAB/
  worlds/BR2024Fixed.wbt
  controllers/base/base.c     <- the survival agent
  controllers/red/red.c       <- predator
  controllers/blue/blue.c
```

Open the world in Webots and run. `DEBUG_MODE 1` in `base.c` prints state transitions and resource levels each step.

## Honest limitations

- Colour thresholds are hand tuned, and accuracy drops where resource and threat zones overlap visually. This is the main thing that would not survive contact with a real camera.
- Detection is colour only. No shape or texture, so anything green reads as food.
- Manual threshold tuning does not scale; a learned detector is the obvious successor.
