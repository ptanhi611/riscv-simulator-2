# no_hazard.s
# This program has ZERO data hazards.
# Each instruction writes to a different register
# and only reads from x0 (which is always 0).

addi x1, x0, 10     # x1 = 10
addi x2, x0, 20     # x2 = 20
addi x3, x0, 30     # x3 = 30