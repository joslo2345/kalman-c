"""Generate the frozen benchmark scenarios S1-S5 into tests/vectors/.

Usage: python scripts/gen_vectors.py [output-dir]

The output is deterministic: a splitmix64 generator with Box-Muller, written
in plain Python, so every language repo can regenerate identical files.
The scenarios are FROZEN once results are published. Regenerating with
different parameters invalidates every number collected from them.

File format (whitespace-separated text, one keyword per line):

    kalman-vectors 1
    id <ID>
    description <free text>
    n <state size>
    m <measurement size>
    steps <steps per trial>
    trials <number of independent trials>
    dt <time step>
    measurement linear | range_bearing <sensor_x> <sensor_y>
    F <n*n values, row-major>
    H <m*n values, row-major; zeros for range_bearing>
    Q <n*n values>
    R <m*m values>
    x0 <n values: the filter's initial estimate>
    P0 <n*n values: the filter's initial covariance>
    data rows | zero
    <trial> <k> <truth: n values> <z: m values>      (one row per step, if "rows")

"data zero" means every true state and every measurement is zero.
The truth row k is the true state after step k's prediction, and z is the
measurement of it; a filter runs predict(F) then update(z) for each row.
"""

import math
import os
import sys

MASK = (1 << 64) - 1


class Rng:
    """splitmix64 + Box-Muller, matching tests/scenarios/rng.c."""

    def __init__(self, seed):
        self.state = seed & MASK
        self.spare = None

    def _next(self):
        self.state = (self.state + 0x9E3779B97F4A7C15) & MASK
        z = self.state
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & MASK
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & MASK
        return z ^ (z >> 31)

    def uniform(self):
        return ((self._next() >> 11) + 0.5) / 9007199254740992.0

    def gauss(self):
        if self.spare is not None:
            s, self.spare = self.spare, None
            return s
        u1, u2 = self.uniform(), self.uniform()
        mag = math.sqrt(-2.0 * math.log(u1))
        self.spare = mag * math.sin(2 * math.pi * u2)
        return mag * math.cos(2 * math.pi * u2)


# ---- Small dense linear algebra on lists ----

def identity(n):
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


def zeros(r, c):
    return [[0.0] * c for _ in range(r)]


def matvec(A, x):
    return [sum(a * b for a, b in zip(row, x)) for row in A]


def cholesky(A):
    n = len(A)
    L = zeros(n, n)
    for j in range(n):
        d = A[j][j] - sum(L[j][p] ** 2 for p in range(j))
        if d < 0:
            raise ValueError("matrix is not positive semi-definite")
        L[j][j] = math.sqrt(d)
        for i in range(j + 1, n):
            s = A[i][j] - sum(L[i][p] * L[j][p] for p in range(j))
            L[i][j] = s / L[j][j] if L[j][j] > 0 else 0.0
    return L


def sample(rng, mean, cov):
    """mean + L g, with L the Cholesky factor of cov (semi-definite allowed)."""
    L = cholesky(cov)
    g = [rng.gauss() for _ in mean]
    return [m + v for m, v in zip(mean, matvec(L, g))]


def cv_model(axes, dt, q):
    """Constant velocity with continuous white-noise acceleration.
    State: positions (axes) then velocities (axes)."""
    n = 2 * axes
    F = identity(n)
    Q = zeros(n, n)
    for a in range(axes):
        p, v = a, a + axes
        F[p][v] = dt
        Q[p][p] = q * dt ** 3 / 3
        Q[p][v] = Q[v][p] = q * dt ** 2 / 2
        Q[v][v] = q * dt
    return F, Q


def diag(values):
    D = zeros(len(values), len(values))
    for i, v in enumerate(values):
        D[i][i] = v
    return D


# ---- Writing ----

def fmt_exact(v):
    return repr(float(v))  # shortest round-trip representation


def fmt_data(v):
    return "%.10g" % v


def write_header(f, sc):
    flat = lambda M: " ".join(fmt_exact(v) for row in M for v in row)
    vec = lambda x: " ".join(fmt_exact(v) for v in x)
    f.write("kalman-vectors 1\n")
    f.write(f"id {sc['id']}\n")
    f.write(f"description {sc['description']}\n")
    f.write(f"n {sc['n']}\nm {sc['m']}\nsteps {sc['steps']}\ntrials {sc['trials']}\n")
    f.write(f"dt {fmt_exact(sc['dt'])}\n")
    f.write(f"measurement {sc['measurement']}\n")
    for key in ("F", "H", "Q", "R", "P0"):
        if key == "P0":
            f.write(f"x0 {vec(sc['x0'])}\n")
        f.write(f"{key} {flat(sc[key])}\n")


def simulate(f, sc, seeds, h):
    """Simulate truth and measurements for each trial and write the data rows."""
    f.write("data rows\n")
    F, Q, R = sc["F"], sc["Q"], sc["R"]
    zero_n = [0.0] * sc["n"]
    zero_m = [0.0] * sc["m"]
    for trial, seed in enumerate(seeds):
        rng = Rng(seed)
        x = sample(rng, sc["x0"], sc["P0"])
        for k in range(sc["steps"]):
            w = sample(rng, zero_n, Q)
            x = [a + b for a, b in zip(matvec(F, x), w)]
            v = sample(rng, zero_m, R)
            z = [a + b for a, b in zip(h(x), v)]
            row = [str(trial), str(k)] + [fmt_data(t) for t in x] + [fmt_data(t) for t in z]
            f.write(" ".join(row) + "\n")


def linear_h(H):
    return lambda x: matvec(H, x)


def range_bearing_h(sx, sy):
    def h(x):
        dx, dy = x[0] - sx, x[1] - sy
        if dy <= 1.0:
            raise ValueError("S3 trajectory approached the bearing wrap; change the scenario")
        return [math.hypot(dx, dy), math.atan2(dy, dx)]
    return h


# ---- Scenarios ----

def s1():
    F, Q = cv_model(1, 0.1, 0.1)
    return dict(id="S1", description="1D constant velocity, position measured",
                n=2, m=1, steps=10000, trials=1, dt=0.1, measurement="linear",
                F=F, H=[[1.0, 0.0]], Q=Q, R=[[0.25]],
                x0=[0.0, 1.0], P0=diag([1.0, 0.25])), [101]


def s2():
    F, Q = cv_model(2, 0.1, 0.1)
    H = [[1.0, 0, 0, 0], [0, 1.0, 0, 0]]
    return dict(id="S2", description="2D constant velocity, positions measured",
                n=4, m=2, steps=10000, trials=1, dt=0.1, measurement="linear",
                F=F, H=H, Q=Q, R=diag([0.25, 0.25]),
                x0=[0.0, 0.0, 1.0, 0.5], P0=diag([1.0, 1.0, 0.25, 0.25])), [202]


def s3():
    F, Q = cv_model(2, 0.1, 0.01)
    return dict(id="S3", description="2D constant velocity, range-bearing from a sensor at the origin",
                n=4, m=2, steps=500, trials=200, dt=0.1, measurement="range_bearing 0 0",
                F=F, H=zeros(2, 4), Q=Q, R=diag([1.0, 1e-4]),
                x0=[60.0, 60.0, -1.0, 0.3], P0=diag([4.0, 4.0, 0.04, 0.04])), \
        [3000 + t for t in range(200)]


def s4():
    F, Q = cv_model(2, 0.1, 100.0)
    H = [[1.0, 0, 0, 0], [0, 1.0, 0, 0]]
    return dict(id="S4", description="Ill-conditioned: tiny R, large Q, stationary target",
                n=4, m=2, steps=1000000, trials=1, dt=0.1, measurement="linear",
                F=F, H=H, Q=Q, R=diag([1e-8, 1e-8]),
                x0=[0.0] * 4, P0=identity(4)), None


def s5():
    """15-state INS error-state model, linearized for a stationary, level IMU:
    [dp(3), dv(3), dphi(3), b_accel(3), b_gyro(3)], GNSS position and velocity."""
    n, dt, g = 15, 0.01, 9.81
    A = zeros(n, n)
    f = [0.0, 0.0, -g]  # specific force
    skew_f = [[0, -f[2], f[1]], [f[2], 0, -f[0]], [-f[1], f[0], 0]]
    for i in range(3):
        A[i][3 + i] = 1.0          # dp' = dv
        A[3 + i][9 + i] = 1.0      # dv' += b_accel
        A[6 + i][12 + i] = 1.0     # dphi' = b_gyro
        for j in range(3):
            A[3 + i][6 + j] = -skew_f[i][j]  # dv' = -[f x] dphi
    F = [[(1.0 if i == j else 0.0) + A[i][j] * dt for j in range(n)] for i in range(n)]
    Q = diag([0.0] * 3 + [0.05 ** 2 * dt] * 3 + [0.005 ** 2 * dt] * 3
             + [1e-4 ** 2 * dt] * 3 + [1e-5 ** 2 * dt] * 3)
    H = zeros(6, n)
    for i in range(6):
        H[i][i] = 1.0
    P0 = diag([1.0] * 3 + [0.01] * 3 + [1e-4] * 3 + [1e-4] * 3 + [1e-6] * 3)
    return dict(id="S5", description="15-state INS error-state, GNSS position and velocity",
                n=n, m=6, steps=10000, trials=1, dt=dt, measurement="linear",
                F=F, H=H, Q=Q, R=diag([1.0] * 3 + [0.01] * 3),
                x0=[0.0] * n, P0=P0), [505]


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "..", "tests", "vectors")
    os.makedirs(out, exist_ok=True)
    for make in (s1, s2, s3, s4, s5):
        sc, seeds = make()
        path = os.path.join(out, f"{sc['id']}.txt")
        with open(path, "w", newline="\n") as f:
            write_header(f, sc)
            if seeds is None:
                f.write("data zero\n")
            else:
                h = range_bearing_h(0.0, 0.0) if sc["measurement"].startswith("range_bearing") \
                    else linear_h(sc["H"])
                simulate(f, sc, seeds, h)
        print(f"wrote {path} ({os.path.getsize(path) / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
