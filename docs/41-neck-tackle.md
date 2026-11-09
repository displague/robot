# 41 - The neck's pitch on two strands: the chair no longer slips it (P-064, D-066)

docs/33 found that the chair scene overloads the neck. While the trunk swings forward to seat-off, the neck holds
the head level. Its W-mini line peaked at 62 to 88 N against its 37 N rating, and its clutch slipped up to
534 degrees of the motor. docs/36 and docs/37 then saw the brushless layout lose the 0.50 m chair to the same slip.
Today's gauntlet still flags it at 0.45 m (75 N, a safety factor of 3.0, 23 degrees of slip).

Short answers:
- **The brushless layout now stands from 0.50 m.** It still loaded the neck (76 N, 27 degrees of slip). docs/37's
  631-degree slip came before P-063 took the chair's own mass out of the reach.
- **Letting the head follow the trunk does not help.** At half (`SIT_HEAD_LEVEL=0.5`) the neck slips more on the
  20 kg model (106 degrees), and the 0.50 m rise fails.
- **Two strands at the neck's pitch do.** With `tackle_by_axis={"pitch": 2}`, as the elbow's yaw has its own
  count, the slip is 0 everywhere and the line peak falls to 36 to 52 N. It is now the model (D-066).

## The grid

One chair scene each; the neck line's peak (rated 37 N), the clutch slip and the scene's energy.

| Build, seat | As before | Head level 0.75 | Head level 0.5 | **Pitch on 2 strands** |
| --- | --- | --- | --- | --- |
| 20 kg, 0.45 m | 75 N, 23 deg, 1010 J | 79 N, 64 deg | 82 N, 106 deg | **52 N, 0 deg, 987 J** |
| 20 kg, 0.50 m | 71 N, 0 deg, 1272 J | 78 N, 22 deg | 85 N, 554 deg, **does not stand** | **47 N, 0 deg, 1176 J** |
| brushless layout, 0.45 m | 63 N, 0 deg, 1407 J | 64 N, 0 deg | 69 N, 0 deg | **36 N, 0 deg, 1206 J** |
| brushless layout, 0.50 m | 76 N, 27 deg, 1672 J | 74 N, 10 deg | 56 N, 0 deg | **47 N, 0 deg, 1793 J** |

With the tackle in the model (`sim/robot.xml` regenerated):
- **The gauntlet:** all 14 scenes stand. The 0.45 m chair goes from 1025 J to 954 J, and its neck flag is gone.
  Every other scene is within a few joules.
- **The 20 kg grids:** held walk 14 of 14, soft walk 14, held step 14.
- **The chain grid:** 17 of 24, against 18. The same six heavy one-handed walks fail, and the ball rolled off its
  top once more despite the wait for stillness.
- **The brushless layout at 0.50 m:** it stands with and without armrests (1793 J and 1587 J). The flag is now the
  spine pitch line (above its continuous rating, a safety factor of 2.8), not the neck.

## What it costs

- **Speed:** the neck's pitch runs at half speed, for twice the torque. A head nod does not need the speed.
- **Hardware:** one running sheave on the head side of the pitch line, as the waist's two strands have. The knuckle
  generator (`chain.py --preset neck`) does not draw tackles for any joint, so nothing there changes.
- **Line stiffness:** two strands halve the line's stretch per newton, so the pitch is stiffer at the head.

## Not yet

- The neck's roll and yaw stay on one strand. Neither was flagged.
- The sheave as a printed part, for the neck and the waist alike.
- The 0.40 m and 0.35 m chairs with the tackle (docs/33's other limits there are the knee and the balance).
- The spine pitch line on the brushless layout's chair (a safety factor of 2.8).
- The ball's put-down: a flat top does not hold a sphere reliably. A dished top or a lip would.

## Next

docs/37's item on the brushless 0.50 m chair is closed. Next is the spine pitch line the brushless layout's chair now
loads. Check whether a 3:1 tackle on the waist's pitch (2 today) clears it the way the neck's two strands did, on the
gauntlet and the chair seats.
