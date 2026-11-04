#!/usr/bin/env python3
"""Independent validator for the universal planar-sweep miter model.

No SolidCore code is imported or reused. Geometry is implemented from
first principles; the model under test is

    G(v, w, alpha) = P1(v) + alpha*n1(v) - P2(w) - alpha*n2(w) = 0
    X(alpha)       = P1(v) + alpha*n1(v) = P2(w) + alpha*n2(w)

with n = rot90(t) (left normal), signed curvature kappa from
kappa = cross(P', P'') / |P'|^3, and the extended path pieces (original
curve plus a straight G1 tangent extension at the joint, which is what
SmBSplineCurve::CreateExtendedCurve(.., SM_CT_G1, ..) builds).

Checks per case:
  * rail residual |G|
  * independent closest-point signed distances d1, d2 equal alpha
  * F = d1 - d2
  * distance to the current SolidCore cutting line (tangent at the joint)
  * analytic special cases where they exist
  * representation / subdivision / reparameterisation invariance
  * reach (|alpha| < 1/|kappa|) failure
  * straight-extension vs natural-continuation difference
"""

import math
import random

Vec = tuple  # (x, y)

# ----------------------------------------------------------------------------- vec

def vadd(a, b): return (a[0] + b[0], a[1] + b[1])
def vsub(a, b): return (a[0] - b[0], a[1] - b[1])
def vmul(a, s): return (a[0] * s, a[1] * s)
def vdot(a, b): return a[0] * b[0] + a[1] * b[1]
def vcross(a, b): return a[0] * b[1] - a[1] * b[0]
def vnorm(a): return math.hypot(a[0], a[1])
def vunit(a):
    n = vnorm(a)
    if n == 0.0:
        raise ZeroDivisionError("zero vector")
    return (a[0] / n, a[1] / n)
def rot90(a): return (-a[1], a[0])          # left normal for N = +z
def dist(a, b): return vnorm(vsub(a, b))

# ----------------------------------------------------------------------------- curves

class Curve:
    """A planar curve parameterised on an arbitrary interval (no clamping)."""
    def eval(self, u): raise NotImplementedError
    def ders(self, u): raise NotImplementedError
    def deriv1(self, u): return self.ders(u)[1]
    def deriv2(self, u): return self.ders(u)[2]

    def tangent(self, u):
        return vunit(self.ders(u)[1])

    def normal(self, u):
        return rot90(self.tangent(u))

    def curvature(self, u):
        _, d1, d2 = self.ders(u)
        n = vnorm(d1)
        return vcross(d1, d2) / (n * n * n)


class Line(Curve):
    def __init__(self, a, b):
        self.a, self.b = a, b

    def eval(self, u):
        return vadd(self.a, vmul(vsub(self.b, self.a), u))

    def ders(self, u):
        d = vsub(self.b, self.a)
        return self.eval(u), d, (0.0, 0.0)


class CircleArc(Curve):
    """C + R (cos a, sin a), a = a0 + u (a1 - a0).  No clamping (natural
    continuation is the same circle); a1 > a0 is counter-clockwise."""
    def __init__(self, c, r, a0, a1):
        self.c, self.r, self.a0, self.a1 = c, r, a0, a1

    def eval(self, u):
        a = self.a0 + u * (self.a1 - self.a0)
        return (self.c[0] + self.r * math.cos(a),
                self.c[1] + self.r * math.sin(a))

    def ders(self, u):
        da = self.a1 - self.a0
        a = self.a0 + u * da
        ca, sa = math.cos(a), math.sin(a)
        p = (self.c[0] + self.r * ca, self.c[1] + self.r * sa)
        d1 = (-self.r * da * sa, self.r * da * ca)
        d2 = (-self.r * da * da * ca, -self.r * da * da * sa)
        return p, d1, d2


class QuadBezier(Curve):
    def __init__(self, p0, p1, p2):
        self.p = [p0, p1, p2]

    def eval(self, u):
        p0, p1, p2 = self.p
        s = 1.0 - u
        return (s * s * p0[0] + 2 * s * u * p1[0] + u * u * p2[0],
                s * s * p0[1] + 2 * s * u * p1[1] + u * u * p2[1])

    def ders(self, u):
        p0, p1, p2 = self.p
        s = 1.0 - u
        p = self.eval(u)
        d1 = (2 * s * (p1[0] - p0[0]) + 2 * u * (p2[0] - p1[0]),
              2 * s * (p1[1] - p0[1]) + 2 * u * (p2[1] - p1[1]))
        d2 = (2 * (p2[0] - 2 * p1[0] + p0[0]),
              2 * (p2[1] - 2 * p1[1] + p0[1]))
        return p, d1, d2


class CubicBezier(Curve):
    def __init__(self, p0, p1, p2, p3):
        self.p = [p0, p1, p2, p3]

    def eval(self, u):
        p0, p1, p2, p3 = self.p
        s = 1.0 - u
        return (s**3 * p0[0] + 3 * s * s * u * p1[0] + 3 * s * u * u * p2[0] + u**3 * p3[0],
                s**3 * p0[1] + 3 * s * s * u * p1[1] + 3 * s * u * u * p2[1] + u**3 * p3[1])

    def ders(self, u):
        p0, p1, p2, p3 = self.p
        s = 1.0 - u
        p = self.eval(u)
        d1 = (3 * s * s * (p1[0] - p0[0]) + 6 * s * u * (p2[0] - p1[0]) + 3 * u * u * (p3[0] - p2[0]),
              3 * s * s * (p1[1] - p0[1]) + 6 * s * u * (p2[1] - p1[1]) + 3 * u * u * (p3[1] - p2[1]))
        d2 = (6 * s * (p2[0] - 2 * p1[0] + p0[0]) + 6 * u * (p3[0] - 2 * p2[0] + p1[0]),
              6 * s * (p2[1] - 2 * p1[1] + p0[1]) + 6 * u * (p3[1] - 2 * p2[1] + p1[1]))
        return p, d1, d2


def _find_span(n, p, u, U):
    if u >= U[n + 1]:
        return n
    if u <= U[p]:
        return p
    low, high = p, n + 1
    mid = (low + high) // 2
    while u < U[mid] or u >= U[mid + 1]:
        if u < U[mid]:
            high = mid
        else:
            low = mid
        mid = (low + high) // 2
    return mid


def _eval_bs(ctrl, U, p, u):
    n = len(ctrl) - 1
    if u <= U[p]:
        u = U[p]
    if u >= U[n + 1]:
        u = U[n + 1]
    span = _find_span(n, p, u, U)
    d = [list(ctrl[span - p + i]) for i in range(p + 1)]
    for r in range(1, p + 1):
        for j in range(p, r - 1, -1):
            left = U[span - p + j]
            right = U[span + 1 + j - r]
            den = right - left
            alpha = (u - left) / den if den != 0.0 else 0.0
            d[j][0] = d[j - 1][0] + alpha * (d[j][0] - d[j - 1][0])
            d[j][1] = d[j - 1][1] + alpha * (d[j][1] - d[j - 1][1])
    return (d[p][0], d[p][1])


class BSpline(Curve):
    def __init__(self, ctrl, knots, degree):
        self.ctrl = [tuple(c) for c in ctrl]
        self.U = list(knots)
        self.p = degree
        self.d = self._derive(self.ctrl, self.U, self.p)
        self.dd = self._derive(*self.d) if self.d else None

    @staticmethod
    def _derive(ctrl, U, p):
        if p == 0:
            return None
        q = []
        for i in range(len(ctrl) - 1):
            den = U[i + p + 1] - U[i + 1]
            q.append((p * (ctrl[i + 1][0] - ctrl[i][0]) / den,
                      p * (ctrl[i + 1][1] - ctrl[i][1]) / den))
        return q, U[1:-1], p - 1

    def eval(self, u):
        return _eval_bs(self.ctrl, self.U, self.p, u)

    def ders(self, u):
        p = self.eval(u)
        d1 = _eval_bs(*self.d, u) if self.d else (0.0, 0.0)
        d2 = _eval_bs(*self.dd, u) if self.dd else (0.0, 0.0)
        return p, d1, d2


# ----------------------------------------------------------------------------- pieces

class AffineReparam(Curve):
    """The same geometry with the piece parameter s in [s0, s1] mapped
    affinely onto the base curve's parameter u in [u0, u1]."""
    def __init__(self, c, s0, s1, u0, u1):
        self.c, self.s0, self.s1, self.u0, self.u1 = c, s0, s1, u0, u1

    def ders(self, s):
        m = (self.u1 - self.u0) / (self.s1 - self.s0)
        u = self.u0 + (s - self.s0) * m
        p, d1, d2 = self.c.ders(u)
        return p, vmul(d1, m), vmul(d2, m * m)


class Piece:
    """A path piece with optional straight G1 tangent extensions.

    s in [u0, u1]           -> the base curve
    s in [u0-1, u0]         -> straight extension of length ext_start (world)
    s in [u1, u1+1]         -> straight extension of length ext_end   (world)

    extension='straight' matches SolidCore (SM_CT_G1 is linear).
    extension='natural'  evaluates the base curve outside its interval
                         (only meaningful for line/circle/B-spline).
    """
    def __init__(self, curve, u0, u1, ext_start=0.0, ext_end=0.0, extension="straight"):
        self.c, self.u0, self.u1 = curve, u0, u1
        self.es, self.ee = ext_start, ext_end
        self.mode = extension
        self.p0, self.d0, _ = curve.ders(u0)
        self.p1, self.d1, _ = curve.ders(u1)
        self.t0 = vunit(self.d0)
        self.t1 = vunit(self.d1)

    def ders(self, s):
        if self.u0 <= s <= self.u1:
            return self.c.ders(s)
        if s < self.u0:
            if self.mode == "natural":
                return self.c.ders(s)
            tau = s - (self.u0 - 1.0)          # 0 at the far end, 1 at u0
            p = vadd(self.p0, vmul(self.t0, (tau - 1.0) * self.es))
            return p, vmul(self.t0, self.es), (0.0, 0.0)
        if self.mode == "natural":
            return self.c.ders(s)
        tau = s - self.u1                       # 0 at u1, 1 at the far end
        p = vadd(self.p1, vmul(self.t1, tau * self.ee))
        return p, vmul(self.t1, self.ee), (0.0, 0.0)

    def eval(self, s):
        return self.ders(s)[0]

    def tangent(self, s):
        return vunit(self.ders(s)[1])

    def normal(self, s):
        return rot90(self.tangent(s))

    def curvature(self, s):
        _, d1, d2 = self.ders(s)
        n = vnorm(d1)
        return vcross(d1, d2) / (n * n * n)

    def end_param(self):
        return self.u1

    def start_param(self):
        return self.u0

    def domain(self):
        return (self.u0 - (1.0 if self.es > 0 else 0.0),
                self.u1 + (1.0 if self.ee > 0 else 0.0))


# ----------------------------------------------------------------------------- solver

def _jac(p1, p2, alpha, v, w):
    _, dP1, ddP1 = p1.ders(v)
    _, dP2, ddP2 = p2.ders(w)
    l1, l2 = vnorm(dP1), vnorm(dP2)
    if l1 < 1e-15 or l2 < 1e-15:
        return None
    n1 = vunit(rot90(dP1))
    n2 = vunit(rot90(dP2))
    k1 = vcross(dP1, ddP1) / (l1 ** 3)
    k2 = vcross(dP2, ddP2) / (l2 ** 3)
    return (n1, n2, k1, k2, vmul(dP1, 1.0 - alpha * k1),
            vmul(dP2, -(1.0 - alpha * k2)))


def solve_at(p1, p2, alpha, v0, w0, tol=1e-13, maxit=40):
    v, w = v0, w0
    for _ in range(maxit):
        j = _jac(p1, p2, alpha, v, w)
        if j is None:
            return None
        n1, n2, k1, k2, Jv, Jw = j
        P1 = p1.eval(v)
        P2 = p2.eval(w)
        G = vsub(vadd(P1, vmul(n1, alpha)), vadd(P2, vmul(n2, alpha)))
        if vnorm(G) < tol:
            return v, w, G
        det = vcross(Jv, Jw)
        if abs(det) < 1e-15:
            return None
        dv = vcross(vmul(G, -1.0), Jw) / det
        dw = vcross(Jv, vmul(G, -1.0)) / det
        # damped update
        step = 1.0
        for _ls in range(20):
            vn, wn = v + step * dv, w + step * dw
            jn = _jac(p1, p2, alpha, vn, wn)
            if jn is None:
                step *= 0.5
                continue
            n1n, n2n, _, _, _, _ = jn
            Gn = vsub(vadd(p1.eval(vn), vmul(n1n, alpha)),
                      vadd(p2.eval(wn), vmul(n2n, alpha)))
            if vnorm(Gn) < vnorm(G) or vnorm(Gn) < tol:
                v, w = vn, wn
                break
            step *= 0.5
        else:
            return None
    return None


def tangent_dir(p1, p2, alpha, v, w):
    j = _jac(p1, p2, alpha, v, w)
    if j is None:
        return None
    n1, n2, _, _, Jv, Jw = j
    rhs = vsub(n2, n1)
    det = vcross(Jv, Jw)
    if abs(det) < 1e-15:
        return None
    dv = vcross(rhs, Jw) / det
    dw = vcross(Jv, rhs) / det
    return dv, dw


def trace(p1, p2, a_min, a_max, h=0.01, reach=math.inf):
    """Trace X(alpha) both ways from alpha = 0.

    Returns (points, info).  A stop in one direction (the rails become
    tangent: a fold, or the seam leaves the piece) does not abort the other.
    """
    v, w = p1.end_param(), p2.start_param()
    j0 = _jac(p1, p2, 0.0, v, w)
    if j0 is None:
        return [], {"events": [{"kind": "singular at the joint"}], "min_abs_det": 0.0}
    n1, n2, _, _, Jv, Jw = j0
    det = vcross(Jv, Jw)
    if abs(det) < 1e-15:
        return [], {"events": [{"kind": "singular at the joint"}], "min_abs_det": 0.0}

    out = [(0.0, v, w, p1.eval(v))]
    events = []
    min_abs_det = abs(det)
    max_reach = reach
    for sign in (+1.0, -1.0):
        v, w = p1.end_param(), p2.start_param()
        alpha = 0.0
        j = _jac(p1, p2, alpha, v, w)
        if j is None:
            events.append({"kind": "singular", "direction": sign})
            continue
        n1, n2, _, _, Jv, Jw = j
        rhs = vsub(n2, n1)
        det = vcross(Jv, Jw)
        if abs(det) < 1e-15:
            events.append({"kind": "singular", "direction": sign})
            continue
        dv = vcross(rhs, Jw) / det
        dw = vcross(Jv, rhs) / det
        step = h
        stop = None
        while True:
            target = alpha + sign * step
            if sign > 0 and target > a_max + 1e-15:
                break
            if sign < 0 and target < a_min - 1e-15:
                break
            if abs(target) >= max_reach:
                stop = {"kind": "reach", "alpha": target}
                break
            # predictor
            vp, wp = v + dv * sign * step, w + dw * sign * step
            sol = solve_at(p1, p2, target, vp, wp)
            if sol is None:
                step *= 0.5
                if step < 1e-9:
                    stop = {"kind": "fold", "alpha": target, "min_abs_det": min_abs_det}
                    break
                continue
            vn, wn, G = sol
            # local determinant and reach
            j = _jac(p1, p2, target, vn, wn)
            if j is None:
                stop = {"kind": "degenerate jacobian", "alpha": target}
                break
            n1, n2, k1, k2, Jv, Jw = j
            det = vcross(Jv, Jw)
            min_abs_det = min(min_abs_det, abs(det))
            reach1 = 1.0 / abs(k1) if k1 != 0.0 else math.inf
            reach2 = 1.0 / abs(k2) if k2 != 0.0 else math.inf
            if abs(target) > min(reach1, reach2) + 1e-12:
                stop = {"kind": "reach", "alpha": target,
                        "reach": min(reach1, reach2)}
                break
            v, w = vn, wn
            alpha = target
            rhs = vsub(n2, n1)
            if abs(det) < 1e-15:
                stop = {"kind": "singular off the joint", "alpha": alpha}
                break
            dv = vcross(rhs, Jw) / det
            dw = vcross(Jv, rhs) / det
            X = vadd(p1.eval(v), vmul(n1, alpha))   # X = P1 + alpha n1
            out.append((alpha, v, w, X))
            step = min(step * 1.5, h * 8.0)
        if stop is not None:
            events.append(stop)
    out.sort(key=lambda t: t[0])
    return out, {"events": events, "min_abs_det": min_abs_det}


def sample_at(p1, p2, alphas, h=0.002):
    """Solve at the requested alpha values, using the trace as an initial guess."""
    lo = min([0.0] + list(alphas))
    hi = max([0.0] + list(alphas))
    pts, _ = trace(p1, p2, lo, hi, h=h)
    out = []
    for a in alphas:
        if a == 0.0:
            X = p1.eval(p1.end_param())
            out.append((0.0, p1.end_param(), p2.start_param(), X))
            continue
        if not pts:
            continue
        best = min(pts, key=lambda t: abs(t[0] - a))
        sol = solve_at(p1, p2, a, best[1], best[2])
        if sol is None:
            continue
        v, w, G = sol
        X = vadd(p1.eval(v), vmul(p1.normal(v), a))
        out.append((a, v, w, X))
    return out


# ----------------------------------------------------------------------------- independent checks

def _golden(f, a, b, tol=1e-14, it=200):
    gr = (math.sqrt(5.0) - 1.0) / 2.0
    c, d = b - gr * (b - a), a + gr * (b - a)
    fc, fd = f(c), f(d)
    for _ in range(it):
        if b - a < tol:
            break
        if fc < fd:
            b, d, fd = d, c, fc
            c = b - gr * (b - a)
            fc = f(c)
        else:
            a, c, fc = c, d, fd
            d = a + gr * (b - a)
            fd = f(d)
    return (a + b) * 0.5


def closest(piece, X, samples=400):
    a, b = piece.domain()
    best_s, best_d = a, math.inf
    for i in range(samples + 1):
        s = a + (b - a) * i / samples
        d = dist(piece.eval(s), X)
        if d < best_d:
            best_s, best_d = s, d
    lo = a + (b - a) * max(0, (best_s - a) / (b - a) - 1.0 / samples)
    hi = a + (b - a) * min(1, (best_s - a) / (b - a) + 1.0 / samples)
    s = _golden(lambda t: dist(piece.eval(t), X), lo, hi)
    # a few Newton steps on g(s) = (P - X) . P'
    for _ in range(30):
        P, dP, ddP = piece.ders(s)
        g = vdot(vsub(P, X), dP)
        gp = vdot(dP, dP) + vdot(vsub(P, X), ddP)
        if abs(gp) < 1e-18:
            break
        ds = -g / gp
        sn = s + ds
        if sn < a or sn > b:
            break
        s = sn
        if abs(ds) < 1e-15:
            break
    P, dP, _ = piece.ders(s)
    n = vunit(rot90(dP))
    signed = vdot(vsub(X, P), n)
    return signed, s


def stats(xs):
    if not xs:
        return 0.0
    return max(abs(x) for x in xs)


# ----------------------------------------------------------------------------- cases

def make_line_line():
    p1 = Piece(Line((-2.0, 0.0), (0.0, 0.0)), 0.0, 1.0, 0.0, 3.0)
    p2 = Piece(Line((0.0, 0.0), (0.0, 2.0)), 0.0, 1.0, 3.0, 0.0)
    return p1, p2, "line-line", 0.6


def make_line_arc():
    # P1: line from (-2,0) to (0,0), end tangent +x
    p1 = Piece(Line((-2.0, 0.0), (0.0, 0.0)), 0.0, 1.0, 0.0, 3.0)
    # P2: CCW circle arc centre (-1,0), R=1, starts at (0,0) with tangent +y
    arc = CircleArc((-1.0, 0.0), 1.0, 0.0, math.pi * 0.5)
    p2 = Piece(arc, 0.0, 1.0, 3.0, 0.0)
    return p1, p2, "line-arc", 0.9


def make_arc_arc():
    # P1: CCW quarter arc centre (0,1), R=1, ends at (0,0) with tangent +x
    a1 = CircleArc((0.0, 1.0), 1.0, -math.pi, -math.pi * 0.5)
    p1 = Piece(a1, 0.0, 1.0, 0.0, 3.0)
    # P2: CCW quarter arc centre (-1,0), R=1, starts at (0,0) with tangent +y
    a2 = CircleArc((-1.0, 0.0), 1.0, 0.0, math.pi * 0.5)
    p2 = Piece(a2, 0.0, 1.0, 3.0, 0.0)
    return p1, p2, "arc-arc", 0.9


def make_arc_arc_asym():
    # P1: CCW quarter arc centre (0,1), R=1, ends at (0,0) with tangent +x
    a1 = CircleArc((0.0, 1.0), 1.0, -math.pi, -math.pi * 0.5)
    p1 = Piece(a1, 0.0, 1.0, 0.0, 3.0)
    # P2: CCW quarter arc centre (-1.4,0), R=1.4, starts at (0,0) with tangent +y
    a2 = CircleArc((-1.4, 0.0), 1.4, 0.0, math.pi * 0.5)
    p2 = Piece(a2, 0.0, 1.0, 3.0, 0.0)
    return p1, p2, "arc-arc-asym", 0.9


def make_quad_quad():
    b1 = QuadBezier((-2.0, 1.0), (-1.0, 0.0), (0.0, 0.0))   # end tangent +x
    b2 = QuadBezier((0.0, 0.0), (0.0, 1.0), (1.0, 2.0))     # start tangent +y
    p1 = Piece(b1, 0.0, 1.0, 0.0, 3.0)
    p2 = Piece(b2, 0.0, 1.0, 3.0, 0.0)
    return p1, p2, "quad-quad", 0.6


def make_cubic_cubic():
    b1 = CubicBezier((-2.0, 1.5), (-1.2, 0.2), (-0.6, 0.0), (0.0, 0.0))
    b2 = CubicBezier((0.0, 0.0), (0.0, 0.6), (0.2, 1.2), (1.5, 2.0))
    p1 = Piece(b1, 0.0, 1.0, 0.0, 3.0)
    p2 = Piece(b2, 0.0, 1.0, 3.0, 0.0)
    return p1, p2, "cubic-cubic", 0.6


def cubic_as_bspline():
    ctrl = [(-2.0, 1.5), (-1.2, 0.2), (-0.6, 0.0), (0.0, 0.0)]
    return BSpline(ctrl, [0, 0, 0, 0, 1, 1, 1, 1], 3)


def cubic_as_bspline2():
    ctrl = [(0.0, 0.0), (0.0, 0.6), (0.2, 1.2), (1.5, 2.0)]
    return BSpline(ctrl, [0, 0, 0, 0, 1, 1, 1, 1], 3)


def make_bspline_bspline():
    b1 = cubic_as_bspline()
    b2 = cubic_as_bspline2()
    p1 = Piece(b1, 0.0, 1.0, 0.0, 3.0)
    p2 = Piece(b2, 0.0, 1.0, 3.0, 0.0)
    return p1, p2, "bspline-bspline", 0.6


def make_freeform_bspline():
    # clamped cubic, 6 control points, interior knots 1 and 2
    ctrl = [(-3.0, 1.8), (-2.2, 0.4), (-1.0, -0.1), (-0.4, 0.2), (-0.1, 0.0), (0.0, 0.0)]
    U = [0, 0, 0, 0, 1, 2, 3, 3, 3, 3]
    return BSpline(ctrl, U, 3)


def make_freeform_bspline2():
    ctrl = [(0.0, 0.0), (0.1, 0.0), (0.4, 0.2), (1.0, 0.1), (2.2, 0.4), (3.0, 1.8)]
    U = [0, 0, 0, 0, 1, 2, 3, 3, 3, 3]
    return BSpline(ctrl, U, 3)


def split_cubic(b):
    p0, p1, p2, p3 = b.p
    def lerp(a, c, t=0.5): return vadd(a, vmul(vsub(c, a), t))
    a = lerp(p0, p1); d = lerp(p1, p2); f = lerp(p2, p3)
    e = lerp(a, d); g = lerp(d, f); h = lerp(e, g)
    return CubicBezier(p0, a, e, h), CubicBezier(h, g, f, p3)


# ----------------------------------------------------------------------------- reporting

def report_case(name, p1, p2, a_max, h=0.005):
    pts, info = trace(p1, p2, -a_max, a_max, h=h)
    rows = []
    for alpha, v, w, X in pts:
        d1, s1 = closest(p1, X)
        d2, s2 = closest(p2, X)
        F = d1 - d2
        # current SolidCore cutting line: tangent at the joint, direction t2-t1
        t1 = p1.tangent(p1.end_param())
        t2 = p2.tangent(p2.start_param())
        O = p1.eval(p1.end_param())
        dir_line = vunit(vsub(t2, t1))
        dist_line = abs(vcross(dir_line, vsub(X, O)))
        rows.append((alpha, X, d1, d2, F, dist_line))
    err_d1 = stats([r[2] - r[0] for r in rows])
    err_d2 = stats([r[3] - r[0] for r in rows])
    err_F = stats([r[4] for r in rows])
    err_line = stats([r[5] for r in rows])
    # O(alpha^2) fit on the current-line deviation (curved cases only)
    fit = None
    nz = [(abs(r[0]), r[5]) for r in rows if abs(r[0]) > 1e-6]
    if len(nz) > 5:
        sxy = sum(a * b for a, b in nz)
        sxx = sum(a * a * a * a for a, b in nz)
        # fit dev = c * alpha^2: c = sum(a^2 * dev) / sum(a^4)
        c = sum((a * a) * b for a, b in nz) / sxx if sxx > 0 else 0.0
        resid = max(abs(b - c * a * a) for a, b in nz)
        fit = (c, resid)
    return {
        "name": name, "points": len(rows), "events": info["events"],
        "alpha_range": (rows[0][0] if rows else None, rows[-1][0] if rows else None),
        "max_|d1-alpha|": err_d1, "max_|d2-alpha|": err_d2, "max_|F|": err_F,
        "max_dist_current_line": err_line, "quad_fit": fit,
        "min_abs_det": info.get("min_abs_det"),
        "rows": rows,
    }


def fmt(x):
    if x is None:
        return "   -   "
    return f"{x:.3e}"


def fmt_events(events):
    if not events:
        return ""
    out = []
    for e in events:
        k = e.get("kind", "?")
        if "alpha" in e:
            out.append(f"{k}@{e['alpha']:+.3f}")
        else:
            out.append(k)
    return "; ".join(out)


def main():
    random.seed(1)
    cases = [
        make_line_line(),
        make_line_arc(),
        make_arc_arc(),
        make_arc_arc_asym(),
        make_quad_quad(),
        make_cubic_cubic(),
        make_bspline_bspline(),
    ]
    results = []
    print("=" * 100)
    print("CORE CASES: rail solve G=0, independent signed-distance check, current-line deviation")
    print("=" * 100)
    print(f"{'case':18} {'pts':>4} {'a-range':>16} {'|d1-a|':>10} {'|d2-a|':>10} "
          f"{'|F|':>10} {'d_line':>10} {'min|det|':>10} err")
    for p1, p2, name, a_max in cases:
        r = report_case(name, p1, p2, a_max)
        results.append(r)
        ar = f"[{r['alpha_range'][0]:+.2f},{r['alpha_range'][1]:+.2f}]"
        print(f"{name:18} {r['points']:4d} {ar:>16} {r['max_|d1-alpha|']:10.2e} "
              f"{r['max_|d2-alpha|']:10.2e} {r['max_|F|']:10.2e} "
              f"{r['max_dist_current_line']:10.2e} {r['min_abs_det']:10.2e} {fmt_events(r['events'])}")
        pos = [row for row in r["rows"] if row[0] > 1e-6]
        if pos:
            a, X, d1, d2, F, dl = pos[-1]
            print(f"   O(alpha^2) check on the positive side: "
                  f"d_line({a:.3f}) / alpha^2 = {dl / (a * a):.3f}")

    # ---- analytic checks -----------------------------------------------------
    print("\n" + "=" * 100)
    print("ANALYTIC CHECKS")
    print("=" * 100)

    # line-line: the seam is the bisector line through O along t2-t1
    p1, p2, _, a_max = make_line_line()
    pts, _ = trace(p1, p2, -a_max, a_max, h=0.01)
    O = p1.eval(p1.end_param())
    d = vunit(vsub(p2.tangent(p2.start_param()), p1.tangent(p1.end_param())))
    ll = max(abs(vcross(d, vsub(X, O))) for _, _, _, X in pts)
    print(f"line-line: max distance to the exact bisector line = {ll:.3e}")

    # line-arc: on the original arc side, d1 - (R - |X-C|) = 0 (parabola)
    p1, p2, _, a_max = make_line_arc()
    pts, _ = trace(p1, p2, 0.05, a_max, h=0.01)   # inner side: on the arc
    C, R = (-1.0, 0.0), 1.0
    par = max(abs((X[0] - O[0]) * 0 + (X[1] - 0.0) - (R - dist(X, C)))
              for _, _, _, X in pts)  # line d = (X-O).n1 = y (n1 = +y)
    print(f"line-arc: max |d_line - (R - |X-C|)| on the arc side = {par:.3e}")

    # arc-arc: signed distance equation R1 - |X-C1| = R2 - |X-C2|
    p1, p2, _, a_max = make_arc_arc()
    pts, _ = trace(p1, p2, 0.02, a_max, h=0.01)
    C1, R1, C2, R2 = (0.0, 1.0), 1.0, (-1.0, 0.0), 1.0
    aa = max(abs((R1 - dist(X, C1)) - (R2 - dist(X, C2))) for _, _, _, X in pts)
    print(f"arc-arc: max |(R1-|X-C1|) - (R2-|X-C2|)| = {aa:.3e}")
    # the symmetric pair's seam is exactly the current cutting line
    O = p1.eval(p1.end_param())
    d = vunit(vsub(p2.tangent(p2.start_param()), p1.tangent(p1.end_param())))
    sym = max(abs(vcross(d, vsub(X, O))) for _, _, _, X in pts)
    print(f"arc-arc (symmetric): max distance to the current line = {sym:.3e}")
    p1, p2, _, a_max = make_arc_arc_asym()
    pts, _ = trace(p1, p2, 0.02, a_max, h=0.01)
    C1, R1, C2, R2 = (0.0, 1.0), 1.0, (-1.4, 0.0), 1.4
    aa = max(abs((R1 - dist(X, C1)) - (R2 - dist(X, C2))) for _, _, _, X in pts)
    print(f"arc-arc-asym: max |(R1-|X-C1|) - (R2-|X-C2|)| = {aa:.3e}")

    # ---- representation invariance ------------------------------------------
    print("\n" + "=" * 100)
    print("INVARIANCE")
    print("=" * 100)
    p1, p2, _, a_max = make_cubic_cubic()
    base, _ = trace(p1, p2, -a_max, a_max, h=0.005)
    # same geometry as B-spline
    p1b = Piece(cubic_as_bspline(), 0.0, 1.0, 0.0, 3.0)
    p2b = Piece(cubic_as_bspline2(), 0.0, 1.0, 3.0, 0.0)
    bs, _ = trace(p1b, p2b, -a_max, a_max, h=0.005)
    def point_set_diff(A, B):
        # A, B are lists of (alpha, v, w, X); compare X at matching alpha
        bmap = {round(a, 12): X for a, _, _, X in B}
        mx = 0.0
        for a, _, _, X in A:
            key = round(a, 12)
            if key in bmap:
                mx = max(mx, dist(X, bmap[key]))
        return mx
    print(f"cubic vs its exact B-spline representation: max point diff = {point_set_diff(base, bs):.3e}")

    # split the cubic at u = 0.5: the corner piece is the second half
    b1 = CubicBezier((-2.0, 1.5), (-1.2, 0.2), (-0.6, 0.0), (0.0, 0.0))
    left, right = split_cubic(b1)   # left is the first half, right ends at O
    p1s = Piece(right, 0.0, 1.0, 0.0, 3.0)
    _, p2c, _, a_max = make_cubic_cubic()
    sp, _ = trace(p1s, p2c, -a_max, a_max, h=0.005)
    print(f"cubic split at u=0.5 (corner face unchanged): max point diff = {point_set_diff(base, sp):.3e}")

    # affine reparameterisation of the line: the same image, u in [0,1] vs [2,5]
    line1 = Line((-2.0, 0.0), (0.0, 0.0))
    line2 = Line((0.0, 0.0), (0.0, 2.0))
    p1l = Piece(line1, 0.0, 1.0, 0.0, 3.0)
    p2l = Piece(line2, 0.0, 1.0, 3.0, 0.0)
    p1r = Piece(AffineReparam(line1, 2.0, 5.0, 0.0, 1.0), 2.0, 5.0, 0.0, 3.0)
    p2r = Piece(AffineReparam(line2, 2.0, 5.0, 0.0, 1.0), 2.0, 5.0, 3.0, 0.0)
    grid = [-0.5, -0.3, -0.1, 0.1, 0.3, 0.5]
    a0 = sample_at(p1l, p2l, grid)
    a1 = sample_at(p1r, p2r, grid)
    bm = {round(a, 9): X for a, _, _, X in a1}
    xd = max(dist(X, bm[round(a, 9)]) for a, _, _, X in a0 if round(a, 9) in bm)
    print(f"line reparameterised (u in [2,5]): max point diff = {xd:.3e}")

    # ---- binding limits ------------------------------------------------------
    print("\n" + "=" * 100)
    print("BINDING LIMITS: rail fold (t1 parallel t2) vs curvature cusp (1 - alpha*kappa = 0)")
    print("=" * 100)
    for a in (0.90, 0.99, 1.01):
        p1, p2, _, _ = make_arc_arc()
        pts, info = trace(p1, p2, 0.0, a, h=0.002)
        if pts:
            ax, _, _, X = pts[-1]
            d1, _ = closest(p1, X)
            d2, _ = closest(p2, X)
            print(f"arc-arc-sym alpha target {a:.2f}: reached {ax:.6f}, "
                  f"d1-alpha {d1-ax:+.2e}, d2-alpha {d2-ax:+.2e}, "
                  f"events={fmt_events(info['events'])}")
    # analytic folds
    print(f"line-arc fold (offset circle tangent to the line): alpha = 0.5")
    print(f"arc-arc-sym fold (two equal offset circles): alpha = 1 - sqrt(2)/2 = "
          f"{1 - math.sqrt(2)/2:.6f}")
    print("At both folds the curvature factor 1 - alpha*kappa is 0.5 and 0.707;"
          " the determinant vanishes because t1 x t2 = 0, not the cusp.")
    print("The curvature cusp (alpha = 1 for R = 1) is therefore not the binding limit here.")

    # ---- straight extension vs natural continuation --------------------------
    print("\n" + "=" * 100)
    print("EXTENSION POLICY: straight G1 (SolidCore) vs natural curve continuation")
    print("(asymmetric arc-arc: R1=1 centre (0,1), R2=1.4 centre (-1.4,0))")
    print("=" * 100)

    def curvature3(x0, x1, x2):
        d = 2 * (x0[0] * (x1[1] - x2[1]) + x1[0] * (x2[1] - x0[1])
                 + x2[0] * (x0[1] - x1[1]))
        if abs(d) < 1e-18:
            return 0.0
        ux = ((x0[0]**2 + x0[1]**2) * (x1[1] - x2[1])
              + (x1[0]**2 + x1[1]**2) * (x2[1] - x0[1])
              + (x2[0]**2 + x2[1]**2) * (x0[1] - x1[1])) / d
        uy = ((x0[0]**2 + x0[1]**2) * (x2[0] - x1[0])
              + (x1[0]**2 + x1[1]**2) * (x0[0] - x2[0])
              + (x2[0]**2 + x2[1]**2) * (x1[0] - x0[0])) / d
        r = math.hypot(x1[0] - ux, x1[1] - uy)
        sign = 1.0 if vcross(vsub(x1, x0), vsub(x2, x1)) > 0 else -1.0
        return sign / r if r > 0 else 0.0

    look = {}
    for mode in ("straight", "natural"):
        p1 = Piece(CircleArc((0.0, 1.0), 1.0, -math.pi, -math.pi * 0.5),
                   0.0, 1.0, 0.0, 3.0, extension=mode)
        p2 = Piece(CircleArc((-1.4, 0.0), 1.4, 0.0, math.pi * 0.5),
                   0.0, 1.0, 3.0, 0.0, extension=mode)
        pts = sample_at(p1, p2, [-0.4, -0.2, 0.2], h=0.0005)
        look[mode] = {round(a, 3): X for a, _, _, X in pts}
        # curvature of gamma on each side of the joint
        for a in (-0.2, 0.2):
            st = sample_at(p1, p2, [a - 0.01, a, a + 0.01], h=0.0005)
            k = curvature3(st[0][3], st[1][3], st[2][3]) if len(st) == 3 else None
            print(f"mode={mode:9s} alpha={a:+.1f}: curvature(gamma) = {k!r}")
    d = dist(look["straight"][-0.4], look["natural"][-0.4]) if (
        -0.4 in look["straight"] and -0.4 in look["natural"]) else float('nan')
    print(f"seam at alpha=-0.4: straight X={look['straight'].get(-0.4)}")
    print(f"seam at alpha=-0.4: natural  X={look['natural'].get(-0.4)}")
    print(f"straight-vs-natural difference at alpha=-0.4 = {d:.3e}")
    print("straight mode: alpha<0 seam lies on the straight tangent extensions (kappa=0)")
    print("natural  mode: alpha<0 seam lies on the circle continuations (kappa!=0)")
    print("=> the G1 straight extension is part of the construction, not of the path.")

    print("\ndone")


if __name__ == "__main__":
    main()
