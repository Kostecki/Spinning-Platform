# Test prints

Small sections cut from the case model (`case/fusion/spinning_platform_case.py`,
`export_test_stl()`), to check fits before printing whole parts. Print in PLA, no supports.

| File | Looks like | Print | Test | Pass if |
|---|---|---|---|---|
| `test-usb-plate.stl` | 42 × 12 mm block, a 10.6 mm tall plate with a rectangular recess and a round hole | as is, bottom on the bed | USB-C board + 2× M2 screws and nuts; flat-top LED pressed in from behind | board sits flush in the recess, screws go through; LED is snug and its rim stops on the back of the wall |
| `test-usb-notch.stl` | thin 50 × 15.5 mm wall strip with a wide notch | flip: flat edge on the bed | slide the USB plate test into the notch | about 1 mm gap above the plate, about 0.5 mm each side, no rubbing |
| `test-pot-wall.stl` | 28 × 25 mm wall with a 7 mm hole and a round boss on one side | standing, bottom edge on the bed | pot from the boss side, nut on the other | shoulder sits flat on the boss, nut gets full grip |
| `test-motor-top.stl` | 46 × 36 × 4 mm plate with a big centre hole, a shallow round recess and 2 small holes | flip: recess side on the bed | motor ears to the plate with 2× M3 from the other side; collar ring into the recess | boss drops into the centre hole, ears sit flat, M3 bites in the pilots; ring fits the recess snugly (don't glue the test) |
| `test-column-insert.stl` | 7.2 mm round post, 10.4 mm tall | insert hole up | press in an M3×5.7 insert, then an M3 screw | insert goes in straight without splitting the post, screw holds |
| `test-bearing-insert.stl` | 11 × 11 × 5 mm block with a hole on one face | hole up | press in an M3×3 insert, then an M3×4 screw | insert sits flush, no bulge or hole on the underside, screw holds |
| `test-hub.stl` | 18 × 18 mm square with a round post and a D-shaped hole | flip: square face on the bed | push onto the motor shaft | slides on without force and without wobble, doesn't slip when turned |
| `collar-ring.stl` | ring, 18 mm outside, 3 mm tall | flat | used with `test-motor-top` | (final part, can be reused) |

Not tested here (dimensions still to come): bearing screw positions, drive pins, load-cell post and pillar.
