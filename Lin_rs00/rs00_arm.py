#!/usr/bin/env python3
"""
RS00Arm — 双关节串联机械臂控制类

封装两个 RS00 电机（肩 + 肘）的底层 CAN 控制，
提供直观的角度制接口，支持指令与反馈的验证。

用法:
    arm = RS00Arm()
    arm.enable()
    arm.set_angles(90, -45).verify()  # 发指令 + 读反馈确认
    arm.disable()

链式:
    RS00Arm().enable().set_angles(90,0).verify().sleep(2).home().disable()

上下文管理器:
    with RS00Arm() as arm:
        arm.set_angles(90, -45).verify()
"""

import time
from rs00_control import motor_enable, motor_disable, motor_control
from rs00_control import get_motor_state, parse_feedback


class RS00Arm:
    """双电机串联机械臂"""

    # 实测机械活动范围（输出轴角度）
    #   肩膀 (ID=1): -40° ~ +220°
    #   肘部 (ID=2): -70° ~ +110°
    DEFAULT_SHOULDER_MIN = -40
    DEFAULT_SHOULDER_MAX = 220
    DEFAULT_ELBOW_MIN = -70
    DEFAULT_ELBOW_MAX = 110

    def __init__(self, iface="can0",
                 shoulder_id=1, elbow_id=2,
                 master_id=0xFD):
        """
        参数:
            iface:       CAN 接口名 (默认 "can0")
            shoulder_id: 肩膀电机 CAN ID (默认 1)
            elbow_id:    肘部电机 CAN ID (默认 2)
            master_id:    主机 ID (默认 0xFD)
        """
        self.iface = iface
        self.shoulder_id = shoulder_id
        self.elbow_id = elbow_id
        self.master_id = master_id
        self._enabled = False

        # 追踪上一次控制参数，用于非破坏性状态查询
        self._last_kp = 10
        self._last_kd = 1

        # ─── 软件零点偏移 ───
        # 用户角度 = 电机原始角度 - 偏移
        # set_zero() 将当前位置设为用户 0°
        self._zero_shoulder = 0.0
        self._zero_elbow = 0.0

        # ─── 追踪最后指令位置（电机空间） ───
        # 用于 get_state() 无感查询，避免发 pos=0 把电机拽走
        self._last_motor_pos_s = 0.0
        self._last_motor_pos_e = 0.0
        self._last_motor_vel = 0.0

        # ─── 软件限位（用户角度空间） ───
        self._shoulder_min = self.DEFAULT_SHOULDER_MIN
        self._shoulder_max = self.DEFAULT_SHOULDER_MAX
        self._elbow_min = self.DEFAULT_ELBOW_MIN
        self._elbow_max = self.DEFAULT_ELBOW_MAX

    # ─── 坐标变换 ───

    def _motor_to_user(self, motor_pos, name):
        """电机原始角度 → 用户角度（去偏移）"""
        offset = self._zero_shoulder if name == 'shoulder' else self._zero_elbow
        return motor_pos - offset

    def _user_to_motor(self, user_pos, name):
        """用户角度 → 电机原始角度（加偏移）"""
        offset = self._zero_shoulder if name == 'shoulder' else self._zero_elbow
        return user_pos + offset

    def _clamp_angles(self, shoulder_deg, elbow_deg):
        """检查角度是否超限，超限则钳位并警告"""
        s = max(self._shoulder_min, min(self._shoulder_max, shoulder_deg))
        e = max(self._elbow_min, min(self._elbow_max, elbow_deg))
        clamped = (s != shoulder_deg) or (e != elbow_deg)
        if clamped:
            print(f"  [WARN] 角度超限！已钳位到安全范围")
            print(f"         肩膀: [{self._shoulder_min}°, {self._shoulder_max}°]  "
                  f"请求={shoulder_deg}° → {s}°")
            print(f"         肘部: [{self._elbow_min}°, {self._elbow_max}°]  "
                  f"请求={elbow_deg}° → {e}°")
        return s, e

    # ─── 设置零点与限位 ───

    def set_zero(self):
        """
        将当前实际位置设为软件零点。

        调用后，所有角度指令和反馈都以此位置为 0°。
        典型用法：把臂摆到机械零点位置，然后调用此方法。
        """
        state = self.get_state()
        if state['shoulder']:
            self._zero_shoulder = state['shoulder']['position']
        if state['elbow']:
            self._zero_elbow = state['elbow']['position']
        print(f"  [ZERO] 肩膀零点={self._zero_shoulder:.1f}°  "
              f"肘部零点={self._zero_elbow:.1f}°")
        # 通知 verify 目标已变化
        self._target_shoulder = 0
        self._target_elbow = 0
        return self

    def set_limits(self, shoulder_min=None, shoulder_max=None,
                   elbow_min=None, elbow_max=None):
        """
        设置运动范围限制。
        不传的参数保持原值。调用 set_zero 后应重新设限位。

        用法:
            arm.set_limits(shoulder_min=-40, shoulder_max=220)
            arm.set_limits(elbow_min=-70, elbow_max=110)
            arm.set_limits(shoulder_min=-90, shoulder_max=90,
                          elbow_min=-135, elbow_max=135)
        """
        if shoulder_min is not None:
            self._shoulder_min = shoulder_min
        if shoulder_max is not None:
            self._shoulder_max = shoulder_max
        if elbow_min is not None:
            self._elbow_min = elbow_min
        if elbow_max is not None:
            self._elbow_max = elbow_max
        print(f"  [LIMIT] 肩膀: [{self._shoulder_min}°, {self._shoulder_max}°]  "
              f"肘部: [{self._elbow_min}°, {self._elbow_max}°]")
        return self

    # ─── 基础控制 ───

    def enable(self):
        """使能两个电机"""
        motor_enable(self.iface, self.shoulder_id, self.master_id)
        motor_enable(self.iface, self.elbow_id, self.master_id)
        self._enabled = True
        return self

    def disable(self):
        """停止两个电机"""
        motor_disable(self.iface, self.shoulder_id, self.master_id)
        motor_disable(self.iface, self.elbow_id, self.master_id)
        self._enabled = False
        return self

    def set_angles(self, shoulder_deg, elbow_deg,
                   vel=0, kp=10, kd=1, torque=0):
        """
        设置肩关节和肘部关节角度（受限位和零点约束）。

        参数:
            shoulder_deg: 肩膀角度 (°)
            elbow_deg:    肘部角度 (°)
            vel:          速度限制 (°/s, 0=不限)
            kp:           位置刚度 (默认 10)
            kd:           阻尼系数
            torque:       前馈力矩 (N.m)
        """
        self._last_kp = kp
        self._last_kd = kd

        # 限位检查
        s_user, e_user = self._clamp_angles(shoulder_deg, elbow_deg)

        # 坐标变换：用户角度 → 电机原始角度
        s_motor = self._user_to_motor(s_user, 'shoulder')
        e_motor = self._user_to_motor(e_user, 'elbow')

        # 保存电机空间最后指令位置（用于无感查询）
        self._last_motor_pos_s = s_motor
        self._last_motor_pos_e = e_motor
        self._last_motor_vel = vel

        motor_control(self.iface, self.shoulder_id, self.master_id,
                      pos=s_motor, vel=vel, kp=kp, kd=kd, torque=torque)
        motor_control(self.iface, self.elbow_id, self.master_id,
                      pos=e_motor, vel=vel, kp=kp, kd=kd, torque=torque)

        # 保存用户空间目标供 verify 使用
        self._target_shoulder = s_user
        self._target_elbow = e_user
        return self

    # ─── 单关节控制 ───

    def set_shoulder(self, deg, vel=0, kp=10, kd=1, torque=0):
        """单独设置肩膀角度 (°)"""
        self._last_kp = kp
        self._last_kd = kd
        s_user = max(self._shoulder_min, min(self._shoulder_max, deg))
        if s_user != deg:
            print(f"  [WARN] 肩膀超限! [{self._shoulder_min}°, {self._shoulder_max}°]  "
                  f"请求={deg}° → {s_user}°")
        self._target_shoulder = s_user
        s_motor = self._user_to_motor(s_user, 'shoulder')
        self._last_motor_pos_s = s_motor
        self._last_motor_vel = vel
        motor_control(self.iface, self.shoulder_id, self.master_id,
                      pos=s_motor, vel=vel, kp=kp, kd=kd, torque=torque)
        return self

    def set_elbow(self, deg, vel=0, kp=10, kd=1, torque=0):
        """单独设置肘部角度 (°)"""
        self._last_kp = kp
        self._last_kd = kd
        e_user = max(self._elbow_min, min(self._elbow_max, deg))
        if e_user != deg:
            print(f"  [WARN] 肘部超限! [{self._elbow_min}°, {self._elbow_max}°]  "
                  f"请求={deg}° → {e_user}°")
        self._target_elbow = e_user
        e_motor = self._user_to_motor(e_user, 'elbow')
        self._last_motor_pos_e = e_motor
        self._last_motor_vel = vel
        motor_control(self.iface, self.elbow_id, self.master_id,
                      pos=e_motor, vel=vel, kp=kp, kd=kd, torque=torque)
        return self

    # ─── 快捷操作 ───

    def home(self, vel=30):
        """归零: 回到 (0°, 0°)"""
        return self.set_angles(0, 0, vel=vel, kp=self._last_kp, kd=self._last_kd)

    def stop(self):
        """紧急停止（立即 disable 两个电机）"""
        return self.disable()

    def sleep(self, seconds):
        """等待（支持链式调用）"""
        time.sleep(seconds)
        return self

    # ─── 状态查询 ───

    def is_enabled(self):
        """是否已使能"""
        return self._enabled

    def get_state(self, timeout=0.15):
        """
        读取两个电机的当前状态（已转换到用户空间角度）。

        用上次指令的位置/刚度发送查询，不改变电机行为。

        返回:
            {
                "shoulder": {"position": °, "velocity": °/s,
                             "torque": N.m, "temperature": °C} | None,
                "elbow":    {...},
            }
        """
        s = get_motor_state(self.iface, self.shoulder_id, self.master_id,
                            timeout, kp_hold=self._last_kp, kd_hold=self._last_kd,
                            pos_hold=self._last_motor_pos_s,
                            vel_hold=self._last_motor_vel)
        e = get_motor_state(self.iface, self.elbow_id, self.master_id,
                            timeout, kp_hold=self._last_kp, kd_hold=self._last_kd,
                            pos_hold=self._last_motor_pos_e,
                            vel_hold=self._last_motor_vel)
        # 转换到用户空间角度
        if s:
            s["position"] = round(self._motor_to_user(s["position"], 'shoulder'), 1)
        if e:
            e["position"] = round(self._motor_to_user(e["position"], 'elbow'), 1)
        return {"shoulder": s, "elbow": e}

    def verify(self, settle_time=2, tolerance=5, timeout=0.15):
        """
        验证最近一次指令是否到位。

        等待 settle_time 秒让电机稳定，然后读反馈并与目标对比。

        参数:
            settle_time: 等待电机到位的时间 (秒)
            tolerance:   允许误差 (°)
            timeout:     状态查询超时

        返回:
            {
                "success": bool,           # 两个电机都在 tolerance 内?
                "shoulder": {"target":°, "actual":°, "error":°} | None,
                "elbow":   同上,
            }
        """
        if settle_time > 0:
            time.sleep(settle_time)

        state = self.get_state(timeout=timeout)
        results = {}
        all_ok = True

        for name, target in [
            ("shoulder", getattr(self, '_target_shoulder', 0)),
            ("elbow", getattr(self, '_target_elbow', 0)),
        ]:
            s = state.get(name)
            if s is None:
                results[name] = None
                all_ok = False
                print(f"  [VERIFY] {name}: ⚠️  无应答")
                continue

            actual = s["position"]
            error = abs(actual - target)
            ok = error <= tolerance
            if not ok:
                all_ok = False

            icon = "✅" if ok else "⚠️"
            print(f"  [VERIFY] {name}: 目标={target}°  实际={actual}°  "
                  f"误差={error:.1f}°  {icon}")
            results[name] = {
                "target": target,
                "actual": actual,
                "error": round(error, 1),
                "success": ok,
            }

        results["success"] = all_ok

        # 如果是自测模式，顺带印温度
        for name in ("shoulder", "elbow"):
            s = state.get(name)
            if s:
                print(f"           {name} 温度={s['temperature']}°C  "
                      f"力矩={s['torque']}Nm")

        return results

    def __repr__(self):
        return (f"<RS00Arm iface={self.iface} "
                f"肩#{self.shoulder_id} 肘#{self.elbow_id} "
                f"限位肩[{self._shoulder_min}°,{self._shoulder_max}°] "
                f"肘[{self._elbow_min}°,{self._elbow_max}°] "
                f"kp={self._last_kp}>")

    # ─── 上下文管理器支持 ───

    def __enter__(self):
        self.enable()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disable()
        return False  # 不吞异常


# ─── 自测（python3 rs00_arm.py 直接运行） ───
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="RS00Arm 双机械臂控制")
    parser.add_argument("--iface", default="can0", help="CAN 接口")
    parser.add_argument("--shoulder", type=int, default=1,
                        help="肩膀电机 CAN ID")
    parser.add_argument("--elbow", type=int, default=2,
                        help="肘部电机 CAN ID")
    parser.add_argument("--no-setup", action="store_true",
                        help="不配置 CAN 接口")
    args = parser.parse_args()

    if not args.no_setup:
        from rs00_control import setup_can, select_device
        dev = select_device()
        if dev:
            setup_can(dev, interface=args.iface)

    print(f"\n🔧 初始化...")
    arm = RS00Arm(iface=args.iface,
                  shoulder_id=args.shoulder,
                  elbow_id=args.elbow)
    print(f"   {arm}")
    # 显示默认限位
    print(f"   默认限位: "
          f"肩 [{arm._shoulder_min}°, {arm._shoulder_max}°]  "
          f"肘 [{arm._elbow_min}°, {arm._elbow_max}°]")

    arm.enable()

    print(f"\n① 标零: 将当前位置设为 0°")
    arm.set_zero()

    print(f"\n② 肩膀 90°, 肘部 -45°")
    arm.set_angles(90, -45, kp=10, kd=1).verify(settle_time=3)

    print(f"\n③ 测试超限保护: 请求肩膀 500°")
    arm.set_shoulder(500, kp=10, kd=1).verify(settle_time=2)

    print(f"\n④ 归零")
    arm.home().verify(settle_time=3)

    print(f"\n⑤ 停止")
    arm.disable()
    print(f"\n✅ 完成")
