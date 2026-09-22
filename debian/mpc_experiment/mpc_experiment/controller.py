"""[20260915 实验] 带位置伺服滞后模型的线性关节 MPC；无 SDK / 网络接口。

c[k+1] = c[k] + dt * u[k]                 (位置指令的速度 u)
q[k+1] = (1-alpha)*q[k] + alpha*c[k+1]    (简化位置伺服响应)

alpha=1 且 c[k]=q[k] 时退化成理想运动学积分器。实际使用 0<alpha<1
时，每轮用反馈 q 重建预测初值；不是预先求一条曲线再播放。
约束针对预测模型/位置指令，不是对未辨识真机动态的安全保证。
"""
from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np
import osqp
from scipy import sparse


class MPCError(RuntimeError):
    """实验立即中止；不得将失败/过期解继续发送给任何执行器。"""


def finite(value, shape, name):
    x = np.asarray(value, dtype=float)
    if x.shape != shape or not np.isfinite(x).all():
        raise ValueError(f"{name}: 必须是形状 {shape} 的有限数值")
    return x


@dataclass(frozen=True)
class Settings:
    dt: float = 0.02
    horizon: int = 15
    response_alpha: float = 0.4
    tracking_weight: float = 10.0
    velocity_weight: float = 0.001
    delta_velocity_weight: float = 0.002
    solver_time_limit_s: float = 0.01
    constraint_tolerance: float = 2e-5

    def __post_init__(self):
        if type(self.horizon) is not int or not 2 <= self.horizon <= 100:
            raise ValueError("horizon 必须是 2..100 的整数")
        for name in ("dt", "response_alpha", "tracking_weight", "velocity_weight",
                     "delta_velocity_weight", "solver_time_limit_s", "constraint_tolerance"):
            x = getattr(self, name)
            if not np.isfinite(x) or x <= 0:
                raise ValueError(f"{name} 必须是有限正数")
        if self.response_alpha > 1:
            raise ValueError("response_alpha 必须 <= 1")


class JointMPC:
    """固定稀疏 QP，在线只更新参考/初值/边界，OSQP 热启动。

    使用单位 deg, s。limits 已由调用方取共享限位、URDF 和余量的交集。
    单轴教学案例和 7 轴模型使用相同实现；不内置机器人安全常量。
    """

    def __init__(self, limits, velocity_limit, acceleration_limit, settings=Settings()):
        self.settings = settings
        bounds = np.asarray(limits, dtype=float)
        if (bounds.ndim != 2 or bounds.shape[1] != 2 or not bounds.shape[0]
                or not np.isfinite(bounds).all() or np.any(bounds[:, 0] >= bounds[:, 1])):
            raise ValueError("limits 必须是非空 n×2 有限上下界")
        self.limits = bounds.copy()
        self.n = n = len(bounds)
        self.vmax = np.broadcast_to(velocity_limit, (n,)).astype(float).copy()
        self.amax = np.broadcast_to(acceleration_limit, (n,)).astype(float).copy()
        if (not np.isfinite(self.vmax).all() or not np.isfinite(self.amax).all()
                or np.any(self.vmax <= 0) or np.any(self.amax <= 0)):
            raise ValueError("速度/加速度限值必须是有限正数")
        s = settings
        N = s.horizon
        eye = sparse.eye(n, format="csc")
        self.L = sparse.kron(np.tril(np.ones((N, N))) * s.dt, eye, format="csc")
        a = 1.0 - s.response_alpha
        self.decay = np.repeat(a ** np.arange(1, N + 1), n)
        kernel = np.zeros((N, N))
        for i in range(N):
            for j in range(i + 1):
                kernel[i, j] = s.dt * (1.0 - a ** (i - j + 1))
        self.G = sparse.kron(kernel, eye, format="csc")
        self.D = sparse.kron(sparse.eye(N) - sparse.eye(N, k=-1), eye, format="csc")
        identity = sparse.eye(N * n, format="csc")
        # 终端可停止：避免“短视地冲到上界才发现来不及刹车”。这不是全局可达证明。
        # 将位置约束行统一到 deg/s 量级，避免 .02 系数导致边界问题病态。
        self.A = sparse.vstack((identity, self.D, self.L/s.dt, self.G/s.dt,
                                identity[-n:]), format="csc")
        P = 2 * (s.tracking_weight * (self.G.T @ self.G)
                 + s.velocity_weight * identity
                 + s.delta_velocity_weight * (self.D.T @ self.D))
        self.solver = osqp.OSQP()
        self.solver.setup(P=sparse.triu(P, format="csc"), q=np.zeros(N*n), A=self.A,
                          l=np.full(4*N*n+n, -np.inf), u=np.full(4*N*n+n, np.inf),
                          verbose=False, warm_starting=True, polishing=True, rho=1.0,
                          eps_abs=1e-6, eps_rel=0, max_iter=20000,
                          adaptive_rho_interval=50, time_limit=s.solver_time_limit_s,
                          check_termination=10)

    def step(self, measured_q, previous_command, previous_velocity, reference):
        started = time.perf_counter()
        s, n, N = self.settings, self.n, self.settings.horizon
        measured = finite(measured_q, (n,), "反馈")
        command = finite(previous_command, (n,), "上次位置指令")
        previous_v = finite(previous_velocity, (n,), "上次指令速度")
        ref = finite(reference, (N, n), "预测参考")
        tol = s.constraint_tolerance
        for label, q in (("反馈", measured), ("上次指令", command)):
            if np.any(q < self.limits[:, 0] - tol) or np.any(q > self.limits[:, 1] + tol):
                raise MPCError(f"{label}越过实验限位，停止而非夹紧数据后继续")
        if np.any(np.abs(previous_v) > self.vmax + tol):
            raise MPCError("上次指令速度超限")
        c0 = np.tile(command, N)
        qbase = self.decay * np.tile(measured, N) + (1-self.decay)*c0
        d0 = np.concatenate((previous_v, np.zeros((N-1)*n)))
        linear = 2 * (s.tracking_weight * self.G.T @ (qbase-ref.ravel())
                      - s.delta_velocity_weight * self.D.T @ d0)
        vlim, dvlim = np.tile(self.vmax, N), np.tile(self.amax*s.dt, N)
        lo, hi = np.tile(self.limits[:, 0], N), np.tile(self.limits[:, 1], N)
        lower = np.concatenate((-vlim, d0-dvlim, (lo-c0)/s.dt, (lo-qbase)/s.dt, np.zeros(n)))
        upper = np.concatenate((vlim, d0+dvlim, (hi-c0)/s.dt, (hi-qbase)/s.dt, np.zeros(n)))
        self.solver.update(q=np.asarray(linear).ravel(), l=lower, u=upper)
        result = self.solver.solve(raise_error=False)
        if result.info.status_val != 1 or result.x is None or not np.isfinite(result.x).all():
            info = result.info
            raise MPCError(
                "QP 未可靠求解: "
                f"{info.status}; "
                f"iter={getattr(info, 'iter', None)}, "
                f"run_time_ms={1000.0 * float(getattr(info, 'run_time', 0.0)):.3f}, "
                f"prim_res={getattr(info, 'prim_res', None)}, "
                f"dual_res={getattr(info, 'dual_res', None)}; "
                "不执行旧解"
            )
        values = self.A @ result.x
        residual = float(max(np.max(lower-values), np.max(values-upper), 0.0))
        if residual > tol:
            raise MPCError(f"QP 约束残差 {residual:g} 超容差")
        velocities = result.x.reshape(N, n)
        predicted = (qbase + self.G @ result.x).reshape(N, n)
        next_command = command + s.dt * velocities[0]
        return {
            "command_deg": next_command, "velocity_deg_s": velocities[0].copy(),
            "predicted_q_deg": predicted,
            "predicted_command_deg": (c0 + self.L @ result.x).reshape(N, n),
            "solve_wall_ms": (time.perf_counter()-started)*1000,
            "solver_iterations": int(result.info.iter),
            "constraint_residual": residual, "status": result.info.status,
        }
