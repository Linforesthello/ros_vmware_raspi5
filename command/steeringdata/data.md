lin@lin-virtual-machine:~/Lin_workspace/command$ python3 measure_steering_ticks.py --unit 4
============================================================
转向电机 ticks/圈测量 — UNIT4
============================================================

准备:
  1. 在 UNIT4 轮子上做个标记
  2. 脚本会发速度=20 让转向电机转
  3. 轮子转一圈回到标记时，按 Ctrl+C

按回车开始...

初始 ticks = 26

发速度=20 给转向电机... 观察轮子，转一圈按 Ctrl+C

实时 ticks:
----------------------------------------
  ticks=  142  total= +116
  ticks=  541  total= +515
  ticks=  918  total= +892
  ticks= 1246  total=+1220
  ticks= 1576  total=+1550
  ticks= 2054  total=+2028
  ticks= 2437  total=+2411
  ticks= 2803  total=+2777
  ticks= 3188  total=+3162
  ticks= 3587  total=+3561
  ticks= 3978  total=+3952
  ticks= 4373  total=+4347
  ticks= 4762  total=+4736
  ticks= 5171  total=+5145
  ticks= 5569  total=+5543
  ticks= 5948  total=+5922
  ticks= 6341  total=+6315
  ticks= 6731  total=+6705
  ticks= 7115  total=+7089
  ticks= 7493  total=+7467
  ticks= 7871  total=+7845
  ticks= 8250  total=+8224
  ticks= 8622  total=+8596
  ticks= 8987  total=+8961
^C
----------------------------------------

结果:
  初始: 26
  最终: 9354
  变化: 9328 ticks
  → UNIT4 每圈 ≈ 9328 ticks
  → 每度 ≈ 25.9 ticks
lin@lin-virtual-machine:~/Lin_workspace/command$ python3 measure_steering_ticks.py --unit 4
============================================================
转向电机 ticks/圈测量 — UNIT4
============================================================

准备:
  1. 在 UNIT4 轮子上做个标记
  2. 脚本会发速度=20 让转向电机转
  3. 轮子转一圈回到标记时，按 Ctrl+C

按回车开始...

初始 ticks = 9481

发速度=20 给转向电机... 观察轮子，转一圈按 Ctrl+C

实时 ticks:
----------------------------------------
  ticks= 9579  total=  +98
  ticks=10068  total= +587
  ticks=10447  total= +966
  ticks=10774  total=+1293
  ticks=11189  total=+1708
  ticks=11609  total=+2128
  ticks=11970  total=+2489
  ticks=12356  total=+2875
  ticks=12742  total=+3261
  ticks=13138  total=+3657
  ticks=13535  total=+4054
  ticks=13927  total=+4446
  ticks=14326  total=+4845
  ticks=14738  total=+5257
  ticks=15123  total=+5642
  ticks=15509  total=+6028
  ticks=15899  total=+6418
  ticks=16284  total=+6803
  ticks=16665  total=+7184
  ticks=17045  total=+7564
  ticks=17422  total=+7941
  ticks=17798  total=+8317
  ticks=18164  total=+8683
  ticks=18532  total=+9051
^C
----------------------------------------

结果:
  初始: 9481
  最终: 18904
  变化: 9423 ticks
  → UNIT4 每圈 ≈ 9423 ticks
  → 每度 ≈ 26.2 ticks
lin@lin-virtual-machine:~/Lin_workspace/command$ 
