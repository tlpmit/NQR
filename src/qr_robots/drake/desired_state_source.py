"""Generic desired-state source for Drake hardware stations."""

from __future__ import annotations

import numpy as np

from pydrake.all import (
    AbstractValue,
    BasicVector,
    Context,
    LeafSystem,
    PiecewisePolynomial,
)


class DesiredStateSource(LeafSystem):
    """
    Drake LeafSystem that outputs desired [q, qdot] for each joint group.

    *groups* is an ordered list of (group_name, num_positions); the system
    declares one ``<group_name>_desired_state`` output port per group, all
    reading slices of a single trajectory over the concatenated q vector.

    The trajectory is stored in Drake abstract state so that the caching
    system is correctly invalidated when the trajectory changes.
    Call set_trajectory() or set_hold() between AdvanceTo() invocations.
    """

    def __init__(self, groups: list[tuple[str, int]]):
        super().__init__()

        self._groups = list(groups)
        self._n_total = sum(n for _, n in self._groups)

        # Abstract state: a mutable dict carrying trajectory info.
        self._state_idx = self.DeclareAbstractState(
            AbstractValue.Make({
                "traj_q":    None,   # PiecewisePolynomial | None
                "traj_qdot": None,   # PiecewisePolynomial | None
                "q_hold":    np.zeros(self._n_total),
                "t_offset":  0.0,
            })
        )
        # Must include time_ticket because _eval reads context.get_time() to
        # evaluate the trajectory — without it the output is cached at the
        # value computed when the abstract state last changed and never updates.
        _prereqs = {self.abstract_state_ticket(self._state_idx), self.time_ticket()}

        start = 0
        for name, n in self._groups:
            sl = slice(start, start + n)

            def _calc(ctx, out: BasicVector, sl=sl):
                q, qd = self._eval(ctx)
                out.SetFromVector(np.concatenate([q[sl], qd[sl]]))

            self.DeclareVectorOutputPort(
                f"{name}_desired_state", size=n * 2, calc=_calc,
                prerequisites_of_calc=_prereqs)
            start += n

    @property
    def n_total(self) -> int:
        return self._n_total

    # ── internal ──────────────────────────────────────────────────────────────

    def _eval(self, context: Context) -> tuple[np.ndarray, np.ndarray]:
        s = context.get_abstract_state(self._state_idx).get_value()
        t = context.get_time()
        if s["traj_q"] is not None:
            t_rel = float(np.clip(t - s["t_offset"], 0.0, s["traj_q"].end_time()))
            q = s["traj_q"].value(t_rel).ravel()
            qdot = (s["traj_qdot"].value(t_rel).ravel()
                    if t_rel < s["traj_qdot"].end_time()
                    else np.zeros(self._n_total))
        else:
            q = s["q_hold"]
            qdot = np.zeros(self._n_total)
        return q, qdot

    # ── public API (call between AdvanceTo invocations) ───────────────────────

    def set_trajectory(self, root_ctx: Context,
                       traj_q: PiecewisePolynomial,
                       traj_qdot: PiecewisePolynomial,
                       t_offset: float):
        s = (self.GetMyContextFromRoot(root_ctx)
             .get_mutable_abstract_state(self._state_idx)
             .get_mutable_value())
        s["traj_q"] = traj_q
        s["traj_qdot"] = traj_qdot
        s["t_offset"] = t_offset

    def set_hold(self, root_ctx: Context, q: np.ndarray):
        s = (self.GetMyContextFromRoot(root_ctx)
             .get_mutable_abstract_state(self._state_idx)
             .get_mutable_value())
        s["q_hold"] = q.copy()
        s["traj_q"] = None
        s["traj_qdot"] = None

    def get_desired_q(self, root_ctx: Context) -> np.ndarray:
        s = (self.GetMyContextFromRoot(root_ctx)
             .get_abstract_state(self._state_idx).get_value())
        t = root_ctx.get_time()
        if s["traj_q"] is not None:
            t_rel = float(np.clip(t - s["t_offset"], 0.0, s["traj_q"].end_time()))
            return s["traj_q"].value(t_rel).ravel()
        return s["q_hold"].copy()
