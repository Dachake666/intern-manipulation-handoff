import bisect
import copy
import math


def gap(a, b):
    return max(abs(float(x) - float(y)) for x, y in zip(a, b))


def scurve(distance, vmax, amax, jmax):
    L = float(distance)
    V = float(vmax)
    A = float(amax)
    J = float(jmax)

    if L <= 0:
        return [], 0.0

    if V <= A * A / J:
        tj_v = math.sqrt(V / J)
        ta_v = 0.0
        ap_v = J * tj_v
    else:
        tj_v = A / J
        ta_v = V / A - tj_v
        ap_v = A

    xacc_v = ap_v * (
        tj_v * tj_v
        + 1.5 * tj_v * ta_v
        + 0.5 * ta_v * ta_v
    )

    if L >= 2.0 * xacc_v:
        tj = tj_v
        ta = ta_v
        tv = (L - 2.0 * xacc_v) / V
    else:
        tj_a = A / J
        L_to_A = 2.0 * A * tj_a * tj_a

        if L >= L_to_A:
            tj = tj_a
            ta = (
                -3.0 * tj
                + math.sqrt(tj * tj + 4.0 * L / A)
            ) / 2.0
            ta = max(0.0, ta)
        else:
            tj = (L / (2.0 * J)) ** (1.0 / 3.0)
            ta = 0.0

        tv = 0.0

    phases = [
        (+J, tj),
        (0.0, ta),
        (-J, tj),
        (0.0, tv),
        (-J, tj),
        (0.0, ta),
        (+J, tj),
    ]

    return phases, sum(t for _, t in phases)


def scurve_state(phases, t):
    x = v = a = 0.0
    elapsed = 0.0

    for j, T in phases:
        if T <= 0:
            continue

        tau = min(T, max(0.0, t - elapsed))

        if tau > 0:
            x2 = x + v*tau + 0.5*a*tau*tau + j*tau**3/6.0
            v2 = v + a*tau + 0.5*j*tau*tau
            a2 = a + j*tau

            x, v, a = x2, v2, a2

        if t <= elapsed + T:
            return x, v, a

        elapsed += T

    return x, v, a


def quintic(p0, p1, m0, m1, h, u):
    u2 = u*u
    u3 = u2*u
    u4 = u3*u
    u5 = u4*u

    h00 = 1 - 10*u3 + 15*u4 - 6*u5
    h10 = u - 6*u3 + 8*u4 - 3*u5
    h20 = 0.5*(u2 - 3*u3 + 3*u4 - u5)

    h01 = 10*u3 - 15*u4 + 6*u5
    h11 = -4*u3 + 7*u4 - 3*u5
    h21 = 0.5*(u3 - 2*u4 + u5)

    # q''(s)=0 at every anchor => C2 across knots.
    return [
        h00*a
        + h10*h*da
        + h20*h*h*0.0
        + h01*b
        + h11*h*db
        + h21*h*h*0.0
        for a, b, da, db
        in zip(p0, p1, m0, m1)
    ]


class SmoothPath:
    def __init__(
        self,
        start_q,
        block,
        *,
        tension=0.45,
        corridor_deg=0.08,
        lut_per_segment=16,
    ):
        self.q = [list(map(float, start_q))]
        self.meta = [copy.deepcopy(block[0])]

        for item in block:
            q = list(map(float, item["q_sdk_deg"]))

            if gap(self.q[-1], q) <= 1e-12:
                self.meta[-1] = copy.deepcopy(item)
                continue

            self.q.append(q)
            self.meta.append(copy.deepcopy(item))

        if len(self.q) < 2:
            self.length = 0.0
            self.max_corridor_dev = 0.0
            return

        self.s = [0.0]
        for a, b in zip(self.q[:-1], self.q[1:]):
            self.s.append(self.s[-1] + gap(a, b))

        n = len(self.q)

        self.m = [[0.0]*7 for _ in range(n)]

        for i in range(1, n-1):
            den = self.s[i+1] - self.s[i-1]
            if den <= 1e-12:
                continue

            self.m[i] = [
                tension * (b-a) / den
                for a, b in zip(self.q[i-1], self.q[i+1])
            ]

        # block boundaries: stationary
        self.m[0] = [0.0]*7
        self.m[-1] = [0.0]*7

        self.max_corridor_dev = 0.0

        # Verify spline remains close to corresponding verified line segment.
        for i in range(n-1):
            h = self.s[i+1] - self.s[i]

            for k in range(1, 16):
                u = k / 16.0

                qs = quintic(
                    self.q[i], self.q[i+1],
                    self.m[i], self.m[i+1],
                    h, u,
                )

                qlin = [
                    a + u*(b-a)
                    for a, b in zip(self.q[i], self.q[i+1])
                ]

                d = gap(qs, qlin)
                self.max_corridor_dev = max(
                    self.max_corridor_dev, d
                )

        if self.max_corridor_dev > corridor_deg:
            raise RuntimeError(
                f"C2 spline corridor violation: "
                f"{self.max_corridor_dev:.4f}° > "
                f"{corridor_deg:.4f}°"
            )

        # Arc-length LUT of the smooth curve.
        self.arc = [0.0]
        self.lut_q = [list(self.q[0])]
        self.lut_meta = [copy.deepcopy(self.meta[0])]

        prev = list(self.q[0])

        for i in range(n-1):
            h = self.s[i+1] - self.s[i]

            for k in range(1, lut_per_segment + 1):
                u = k / float(lut_per_segment)

                qx = quintic(
                    self.q[i], self.q[i+1],
                    self.m[i], self.m[i+1],
                    h, u,
                )

                self.arc.append(
                    self.arc[-1] + gap(prev, qx)
                )
                self.lut_q.append(qx)
                self.lut_meta.append(
                    copy.deepcopy(self.meta[i+1])
                )

                prev = qx

        self.length = self.arc[-1]

    def sample(self, spos):
        if self.length <= 1e-12:
            item = copy.deepcopy(self.meta[-1])
            item["q_sdk_deg"] = list(self.q[-1])
            return item

        if spos <= 0:
            item = copy.deepcopy(self.lut_meta[0])
            item["q_sdk_deg"] = list(self.lut_q[0])
            return item

        if spos >= self.length:
            item = copy.deepcopy(self.meta[-1])
            item["q_sdk_deg"] = list(self.q[-1])
            return item

        j = bisect.bisect_right(self.arc, spos)
        j = max(1, min(j, len(self.arc)-1))

        a0 = self.arc[j-1]
        a1 = self.arc[j]

        alpha = (
            (spos-a0)/(a1-a0)
            if a1 > a0
            else 1.0
        )

        q0 = self.lut_q[j-1]
        q1 = self.lut_q[j]

        qx = [
            a + alpha*(b-a)
            for a, b in zip(q0, q1)
        ]

        item = copy.deepcopy(self.lut_meta[j])
        item["q_sdk_deg"] = qx
        item["vaj3_generated"] = True
        return item


def joint_metrics(start_q, items, dt):
    if not items:
        return {
            "max_v": 0.0,
            "max_a": 0.0,
            "max_j": 0.0,
        }

    q0 = list(map(float, start_q))

    qs = [list(q0), list(q0), list(q0)]
    qs += [
        list(map(float, w["q_sdk_deg"]))
        for w in items
    ]

    qe = list(qs[-1])
    qs += [list(qe), list(qe), list(qe)]

    vel = [
        [(b-a)/dt for a, b in zip(x, y)]
        for x, y in zip(qs[:-1], qs[1:])
    ]

    acc = [
        [(b-a)/dt for a, b in zip(x, y)]
        for x, y in zip(vel[:-1], vel[1:])
    ]

    jerk = [
        [(b-a)/dt for a, b in zip(x, y)]
        for x, y in zip(acc[:-1], acc[1:])
    ]

    def peak(rows):
        return max(
            (abs(x) for row in rows for x in row),
            default=0.0,
        )

    return {
        "max_v": peak(vel),
        "max_a": peak(acc),
        "max_j": peak(jerk),
    }


def retime_block(
    start_q,
    block,
    *,
    dt,
    vmax,
    amax,
    jmax,
    tension,
    corridor_deg,
):
    path = SmoothPath(
        start_q,
        block,
        tension=tension,
        corridor_deg=corridor_deg,
    )

    if path.length <= 1e-12:
        item = copy.deepcopy(block[-1])
        item["q_sdk_deg"] = list(map(
            float, block[-1]["q_sdk_deg"]
        ))

        return [item], {
            "frames": 1,
            "duration_s": dt,
            "path_length_deg": 0.0,
            "corridor_dev_deg": path.max_corridor_dev,
            "max_v_deg_s": 0.0,
            "max_a_deg_s2": 0.0,
            "max_j_deg_s3": 0.0,
            "time_scale": 1.0,
        }

    phases, T0 = scurve(
        path.length,
        vmax,
        amax,
        jmax,
    )

    end_s, _, _ = scurve_state(phases, T0)
    if end_s <= 0 or T0 <= 0:
        raise RuntimeError("invalid VAJ3 base profile")

    scale = 1.0

    for _ in range(12):
        target_T = T0 * scale
        n = max(1, math.ceil(target_T / dt))

        actual_T = n * dt
        actual_scale = actual_T / T0

        items = []

        for k in range(1, n+1):
            t_real = k * dt
            t_nom = min(T0, t_real / actual_scale)

            ss, _, _ = scurve_state(
                phases, t_nom
            )

            ss = path.length * ss / end_s
            ss = min(path.length, max(0.0, ss))

            items.append(path.sample(ss))

        # endpoint must remain exact.
        items[-1]["q_sdk_deg"] = list(map(
            float, block[-1]["q_sdk_deg"]
        ))

        m = joint_metrics(start_q, items, dt)

        rv = m["max_v"] / vmax
        ra = m["max_a"] / amax
        rj = m["max_j"] / jmax

        required = max(
            1.0,
            rv,
            math.sqrt(max(0.0, ra)),
            max(0.0, rj) ** (1.0/3.0),
        )

        if required <= 1.001:
            return items, {
                "frames": len(items),
                "duration_s": len(items)*dt,
                "path_length_deg": path.length,
                "corridor_dev_deg":
                    path.max_corridor_dev,
                "max_v_deg_s": m["max_v"],
                "max_a_deg_s2": m["max_a"],
                "max_j_deg_s3": m["max_j"],
                "time_scale": actual_scale,
            }

        scale = actual_scale * required * 1.04

    raise RuntimeError(
        "VAJ3 failed to satisfy joint-space limits "
        "within 12 retiming iterations"
    )


def retime_plan(
    runtime_plan,
    initial_q,
    *,
    dt,
    vmax,
    amax,
    jmax,
    tension=0.45,
    corridor_deg=0.08,
):
    result = copy.deepcopy(runtime_plan)

    out = []
    block = []
    stats = []

    current_q = list(map(float, initial_q))

    def flush():
        nonlocal block, current_q

        if not block:
            return

        generated, info = retime_block(
            current_q,
            block,
            dt=dt,
            vmax=vmax,
            amax=amax,
            jmax=jmax,
            tension=tension,
            corridor_deg=corridor_deg,
        )

        out.extend(generated)
        current_q = list(generated[-1]["q_sdk_deg"])

        info["block_index"] = len(stats)
        info["source_anchor_count"] = len(block)
        stats.append(info)

        block = []

    for item in result["waypoints"]:
        if "q_sdk_deg" in item:
            block.append(item)
        else:
            flush()
            out.append(copy.deepcopy(item))

    flush()

    result["waypoints"] = out
    result["stream_policy"]["period_s"] = float(dt)
    result["stream_policy"]["max_velocity_deg_s"] = float(vmax)
    result["stream_policy"]["max_acceleration_deg_s2"] = float(amax)
    result["stream_policy"]["max_jerk_deg_s3"] = float(jmax)

    return result, {
        "block_count": len(stats),
        "blocks": stats,
        "motion_frames": sum(x["frames"] for x in stats),
        "duration_s": sum(x["duration_s"] for x in stats),
        "max_v_deg_s": max(
            (x["max_v_deg_s"] for x in stats),
            default=0.0,
        ),
        "max_a_deg_s2": max(
            (x["max_a_deg_s2"] for x in stats),
            default=0.0,
        ),
        "max_j_deg_s3": max(
            (x["max_j_deg_s3"] for x in stats),
            default=0.0,
        ),
        "max_corridor_dev_deg": max(
            (x["corridor_dev_deg"] for x in stats),
            default=0.0,
        ),
    }
