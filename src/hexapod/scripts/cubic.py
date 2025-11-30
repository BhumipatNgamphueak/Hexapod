# reference

"""Trajectory generator combining trapezoidal (SUVAT) profiles with cubic-Hermite interpolation.

Usage:
	- Call `generate_trajectory(waypoints, vmax, amax, dt, stop_at_waypoints)` to get
	  times, positions, velocities sampled at `dt` seconds.

Features:
	- Plans per-segment durations using trapezoidal/triangular velocity profiles.
	- Builds smooth cubic-Hermite polynomials for each segment using boundary
	  velocities which can be set to zero (stop at waypoints) or estimated for
	  through-motion.

This file is self-contained and uses numpy.
"""

from typing import Tuple, Optional
import numpy as np


def plan_trapezoid_durations(s: float, u: float = 0.0, v: float = 0.0,
							 vmax: float = 1.0, amax: float = 1.0) -> dict:
	"""Plan a trapezoidal (or triangular if insufficient distance) profile.

	Returns a dict with keys: t_acc, t_cruise, t_dec, T, vmax_reached, s_acc, s_dec
	"""
	if s <= 0:
		return dict(t_acc=0.0, t_cruise=0.0, t_dec=0.0, T=0.0,
					vmax_reached=False, s_acc=0.0, s_dec=0.0)

	# nominal accel/decel to/from requested vmax
	t_acc = max(0.0, (vmax - u) / amax)
	t_dec = max(0.0, (vmax - v) / amax)
	s_acc = u * t_acc + 0.5 * amax * t_acc ** 2
	s_dec = vmax * t_dec - 0.5 * amax * t_dec ** 2  # using decel from vmax to v

	if s_acc + s_dec <= s:
		# can reach vmax
		s_cruise = s - s_acc - s_dec
		t_cruise = s_cruise / vmax if vmax > 0 else 0.0
		T = t_acc + t_cruise + t_dec
		return dict(t_acc=t_acc, t_cruise=t_cruise, t_dec=t_dec, T=T,
					vmax_reached=True, s_acc=s_acc, s_dec=s_dec)
	# triangular profile: compute peak velocity vp
	vp = np.sqrt(max(0.0, (2 * amax * s + u * u + v * v) / 2.0))
	t_acc = max(0.0, (vp - u) / amax)
	t_dec = max(0.0, (vp - v) / amax)
	t_cruise = 0.0
	T = t_acc + t_dec
	# recompute s_acc/s_dec for info
	s_acc = u * t_acc + 0.5 * amax * t_acc ** 2
	s_dec = vp * t_dec - 0.5 * amax * t_dec ** 2
	return dict(t_acc=t_acc, t_cruise=t_cruise, t_dec=t_dec, T=T,
				vmax_reached=False, s_acc=s_acc, s_dec=s_dec)


def _hermite_segment(p0: np.ndarray, p1: np.ndarray,
					 v0: np.ndarray, v1: np.ndarray,
					 T: float, dt: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
	"""Generate samples for one segment using cubic Hermite interpolation.

	Returns: times (relative from 0), positions, velocities (same shape as positions)
	"""
	if T <= 0:
		return np.array([], dtype=float), np.zeros((0, p0.size)), np.zeros((0, p0.size))

	times = np.arange(0.0, T, dt)
	if times.size == 0 or times[-1] + 1e-12 < T:
		times = np.append(times, T)

	t = times
	tau = t / T
	tau2 = tau * tau
	tau3 = tau2 * tau

	h00 = 2 * tau3 - 3 * tau2 + 1
	h10 = tau3 - 2 * tau2 + tau
	h01 = -2 * tau3 + 3 * tau2
	h11 = tau3 - tau2

	p0 = p0.reshape(1, -1)
	p1 = p1.reshape(1, -1)
	v0 = v0.reshape(1, -1)
	v1 = v1.reshape(1, -1)

	positions = (h00.reshape(-1, 1) * p0
				 + h10.reshape(-1, 1) * (T * v0)
				 + h01.reshape(-1, 1) * p1
				 + h11.reshape(-1, 1) * (T * v1))

	# velocity derivative of hermite basis
	dh00 = (6 * tau2 - 6 * tau) / T
	dh10 = (3 * tau2 - 4 * tau + 1) / T
	dh01 = (-6 * tau2 + 6 * tau) / T
	dh11 = (3 * tau2 - 2 * tau) / T

	velocities = (dh00.reshape(-1, 1) * p0
				  + dh10.reshape(-1, 1) * (T * v0)
				  + dh01.reshape(-1, 1) * p1
				  + dh11.reshape(-1, 1) * (T * v1))

	return times, positions, velocities


def generate_trajectory(waypoints, vmax: float = 1.0, amax: float = 1.0,
						dt: float = 0.01, stop_at_waypoints: bool = True,
						min_dt: Optional[float] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
	"""Generate a time-parameterized trajectory through waypoints.

	Parameters:
		waypoints: iterable of points (N x D) or 1D list/array (N,)
		vmax: maximum speed (units/s)
		amax: maximum acceleration (units/s^2)
		dt: sampling timestep for returned trajectory
		stop_at_waypoints: if True, velocity at each waypoint is zero, otherwise
			interior waypoint velocities are estimated to allow smooth passage.

	Returns:
		times (M,), positions (M, D), velocities (M, D)
	"""
	if min_dt is None:
		min_dt = dt

	wp = np.asarray(waypoints, dtype=float)
	if wp.ndim == 1:
		wp = wp.reshape(-1, 1)

	N, D = wp.shape
	if N < 2:
		raise ValueError("Need at least two waypoints")

	# segment distances
	vecs = wp[1:] - wp[:-1]
	dists = np.linalg.norm(vecs, axis=1)

	# plan durations per segment using trapezoid (start/end velocities initially 0)
	seg_info = [plan_trapezoid_durations(s=float(d), u=0.0, v=0.0, vmax=vmax, amax=amax)
				for d in dists]
	durations = np.array([info['T'] for info in seg_info])

	# ensure non-zero duration for tiny segments
	durations = np.maximum(durations, min_dt)

	# estimate waypoint velocities
	velocities = np.zeros((N, D), dtype=float)
	if not stop_at_waypoints:
		# interior points: central difference using adjacent segment durations
		for i in range(1, N - 1):
			Tprev = durations[i - 1]
			Tnext = durations[i]
			if Tprev + Tnext <= 0:
				velocities[i] = 0.0
			else:
				v_est = (wp[i + 1] - wp[i - 1]) / (Tprev + Tnext)
				# clamp magnitude to vmax
				mag = np.linalg.norm(v_est)
				if mag > vmax and mag > 0:
					v_est = v_est * (vmax / mag)
				velocities[i] = v_est
		# endpoints: keep zero or set same as neighbor
		velocities[0] = np.zeros(D)
		velocities[-1] = np.zeros(D)

	# Build trajectory by sampling each segment
	all_times = []
	all_pos = []
	all_vel = []
	time_offset = 0.0

	for i in range(N - 1):
		p0 = wp[i]
		p1 = wp[i + 1]
		v0 = velocities[i]
		v1 = velocities[i + 1]
		T = float(durations[i])

		t_rel, pos_seg, vel_seg = _hermite_segment(p0, p1, v0, v1, T, dt)
		if t_rel.size == 0:
			time_offset += 0.0
			continue

		all_times.append(time_offset + t_rel)
		all_pos.append(pos_seg)
		all_vel.append(vel_seg)
		time_offset += T

	if len(all_times) == 0:
		return np.array([]), np.zeros((0, D)), np.zeros((0, D))

	times = np.concatenate(all_times)
	positions = np.vstack(all_pos)
	velocities = np.vstack(all_vel)

	return times, positions, velocities


def plot_trajectory(waypoints, times, positions, velocities, title="Trajectory"):
	"""Plot path and trajectory visualizations.
	
	For 1D: shows position vs time, velocity vs time
	For 2D: shows XY path, position vs time, velocity vs time
	For 3D+: shows first 3 dimensions
	"""
	try:
		import matplotlib.pyplot as plt
	except ImportError:
		print("matplotlib not available - skipping plots")
		print("Install with: python -m pip install matplotlib")
		return
	
	wp = np.asarray(waypoints)
	if wp.ndim == 1:
		wp = wp.reshape(-1, 1)
	
	N, D = wp.shape
	speeds = np.linalg.norm(velocities, axis=1)
	
	if D == 1:
		# 1D trajectory
		fig, axes = plt.subplots(2, 1, figsize=(10, 8))
		fig.suptitle(f"{title} (1D)", fontsize=14, fontweight='bold')
		
		# Position vs time
		axes[0].plot(times, positions[:, 0], 'b-', linewidth=2, label='Position')
		axes[0].plot([wp[i, 0] for i in range(N)], 
					 [0] * N, 'ro', markersize=10, label='Waypoints', zorder=5)
		axes[0].set_xlabel('Time (s)', fontsize=11)
		axes[0].set_ylabel('Position', fontsize=11)
		axes[0].set_title('Position vs Time')
		axes[0].grid(True, alpha=0.3)
		axes[0].legend()
		
		# Velocity vs time
		axes[1].plot(times, velocities[:, 0], 'g-', linewidth=2, label='Velocity')
		axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.3)
		axes[1].set_xlabel('Time (s)', fontsize=11)
		axes[1].set_ylabel('Velocity', fontsize=11)
		axes[1].set_title('Velocity vs Time')
		axes[1].grid(True, alpha=0.3)
		axes[1].legend()
		
	elif D == 2:
		# 2D trajectory
		fig = plt.figure(figsize=(14, 10))
		fig.suptitle(f"{title} (2D)", fontsize=14, fontweight='bold')
		
		# XY path
		ax1 = plt.subplot(2, 2, 1)
		ax1.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2, label='Trajectory')
		ax1.plot(wp[:, 0], wp[:, 1], 'ro', markersize=10, label='Waypoints', zorder=5)
		# Add arrows to show direction
		arrow_step = max(1, len(positions) // 10)
		for i in range(0, len(positions) - arrow_step, arrow_step):
			dx = positions[i + arrow_step, 0] - positions[i, 0]
			dy = positions[i + arrow_step, 1] - positions[i, 1]
			ax1.arrow(positions[i, 0], positions[i, 1], dx, dy,
					 head_width=0.05, head_length=0.05, fc='blue', ec='blue', alpha=0.4)
		ax1.set_xlabel('X Position', fontsize=11)
		ax1.set_ylabel('Y Position', fontsize=11)
		ax1.set_title('Path (XY Space)')
		ax1.grid(True, alpha=0.3)
		ax1.legend()
		ax1.axis('equal')
		
		# X and Y vs time
		ax2 = plt.subplot(2, 2, 2)
		ax2.plot(times, positions[:, 0], 'r-', linewidth=2, label='X Position')
		ax2.plot(times, positions[:, 1], 'b-', linewidth=2, label='Y Position')
		ax2.set_xlabel('Time (s)', fontsize=11)
		ax2.set_ylabel('Position', fontsize=11)
		ax2.set_title('X, Y vs Time')
		ax2.grid(True, alpha=0.3)
		ax2.legend()
		
		# Velocity components vs time
		ax3 = plt.subplot(2, 2, 3)
		ax3.plot(times, velocities[:, 0], 'r-', linewidth=2, label='Vx')
		ax3.plot(times, velocities[:, 1], 'b-', linewidth=2, label='Vy')
		ax3.set_xlabel('Time (s)', fontsize=11)
		ax3.set_ylabel('Velocity', fontsize=11)
		ax3.set_title('Velocity Components vs Time')
		ax3.grid(True, alpha=0.3)
		ax3.legend()
		
		# Speed vs time
		ax4 = plt.subplot(2, 2, 4)
		ax4.plot(times, speeds, 'g-', linewidth=2, label='Speed')
		ax4.set_xlabel('Time (s)', fontsize=11)
		ax4.set_ylabel('Speed', fontsize=11)
		ax4.set_title('Speed vs Time')
		ax4.grid(True, alpha=0.3)
		ax4.legend()
		
	else:
		# 3D+ trajectory (show first 3 dimensions)
		fig = plt.figure(figsize=(14, 10))
		fig.suptitle(f"{title} (3D)", fontsize=14, fontweight='bold')
		
		# 3D path
		ax1 = plt.subplot(2, 2, 1, projection='3d')
		ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', linewidth=2, label='Trajectory')
		ax1.scatter(wp[:, 0], wp[:, 1], wp[:, 2], c='r', s=100, label='Waypoints', zorder=5)
		ax1.set_xlabel('X', fontsize=10)
		ax1.set_ylabel('Y', fontsize=10)
		ax1.set_zlabel('Z', fontsize=10)
		ax1.set_title('Path (XYZ Space)')
		ax1.legend()
		
		# Position components vs time
		ax2 = plt.subplot(2, 2, 2)
		ax2.plot(times, positions[:, 0], 'r-', linewidth=2, label='X')
		ax2.plot(times, positions[:, 1], 'g-', linewidth=2, label='Y')
		ax2.plot(times, positions[:, 2], 'b-', linewidth=2, label='Z')
		ax2.set_xlabel('Time (s)', fontsize=11)
		ax2.set_ylabel('Position', fontsize=11)
		ax2.set_title('Position Components vs Time')
		ax2.grid(True, alpha=0.3)
		ax2.legend()
		
		# Velocity components vs time
		ax3 = plt.subplot(2, 2, 3)
		ax3.plot(times, velocities[:, 0], 'r-', linewidth=2, label='Vx')
		ax3.plot(times, velocities[:, 1], 'g-', linewidth=2, label='Vy')
		ax3.plot(times, velocities[:, 2], 'b-', linewidth=2, label='Vz')
		ax3.set_xlabel('Time (s)', fontsize=11)
		ax3.set_ylabel('Velocity', fontsize=11)
		ax3.set_title('Velocity Components vs Time')
		ax3.grid(True, alpha=0.3)
		ax3.legend()
		
		# Speed vs time
		ax4 = plt.subplot(2, 2, 4)
		ax4.plot(times, speeds, 'purple', linewidth=2, label='Speed')
		ax4.set_xlabel('Time (s)', fontsize=11)
		ax4.set_ylabel('Speed', fontsize=11)
		ax4.set_title('Speed vs Time')
		ax4.grid(True, alpha=0.3)
		ax4.legend()
	
	plt.tight_layout()
	plt.show()


def generate_gait_path(L1=None, L2=None, step_height=0.3, num_waypoints=8):
	"""Generate a walking gait path with stance and swing phases that forms a closed loop.
	
	Parameters:
		L1: stance phase length (horizontal distance, moving RIGHT), random if None
		L2: swing phase length (horizontal distance, moving LEFT/backwards), random if None
		    If None, L2 will be set equal to L1 to create a closed loop
		step_height: maximum height of swing phase arc
		num_waypoints: number of waypoints in swing phase arc
		
	Returns:
		waypoints: Nx2 array with (x, y) coordinates (closed loop)
		L1: actual L1 used
		L2: actual L2 used
	"""
	if L1 is None:
		L1 = np.random.uniform(2.0, 4.0)
	if L2 is None:
		L2 = L1  # Make L2 equal to L1 to create a closed loop
	
	waypoints = []
	
	# Stance phase: straight line on ground moving RIGHT (positive X direction)
	x_start = 0.0
	waypoints.append([x_start, 0.0])
	waypoints.append([x_start + L1, 0.0])
	
	# Swing phase: arc in the air moving LEFT/BACKWARDS (negative X direction)
	x_swing_start = x_start + L1  # Start of swing at end of stance
	x_swing_end = x_swing_start - L2  # Move BACKWARDS (subtract L2)
	
	# Create arc waypoints using parabolic shape, moving backwards
	for i in range(1, num_waypoints):
		t = i / num_waypoints  # parameter from 0 to 1
		x = x_swing_start - t * L2  # SUBTRACT to go backwards (left)
		# Parabolic arc: peaks at middle
		y = step_height * 4 * t * (1 - t)  # peaks at t=0.5 with height=step_height
		waypoints.append([x, y])
	
	# End of swing phase back on ground (completes the loop when L1 == L2)
	waypoints.append([x_swing_end, 0.0])
	
	return np.array(waypoints), L1, L2


if __name__ == "__main__":
	# Simple demo: 1D waypoints and a 2D example
	import pprint

	print("Running demo trajectory generation...")

	# 1D stop-at-waypoints demo
	wps1 = [0.0, 1.0, 2.0, 1.5]
	t, p, v = generate_trajectory(wps1, vmax=1.0, amax=1.0, dt=0.05, stop_at_waypoints=True)
	print("1D demo: {} samples".format(t.size))
	print("First 6 samples (t, pos, vel):")
	for i in range(min(6, t.size)):
		print(f"{t[i]:.3f}, {p[i,0]:.4f}, {v[i,0]:.4f}")

	# 2D through-waypoints demo (no stop)
	wps2 = np.array([[0.0, 0.0], [1.0, 0.2], [2.0, -0.1], [3.0, 0.0]])
	t2, p2, v2 = generate_trajectory(wps2, vmax=1.2, amax=1.0, dt=0.05, stop_at_waypoints=False)
	print("\n2D pass-through demo: {} samples".format(t2.size))
	print("First 6 samples (t, pos_x, pos_y, speed):")
	for i in range(min(6, t2.size)):
		speed = np.linalg.norm(v2[i])
		print(f"{t2[i]:.3f}, {p2[i,0]:.4f}, {p2[i,1]:.4f}, {speed:.4f}")
	
	# Gait trajectory demo (like the picture)
	print("\n" + "="*60)
	print("GAIT TRAJECTORY DEMO (Stance + Swing Phases) - CLOSED LOOP")
	print("="*60)
	wps_gait, L1, L2 = generate_gait_path()
	print(f"Generated gait path with L1={L1:.2f}, L2={L2:.2f}")
	print(f"Stance phase: 0.0 -> {L1:.2f} (on ground, moving RIGHT)")
	print(f"Swing phase: {L1:.2f} -> {L1-L2:.2f} (arc in air, moving LEFT/backwards)")
	if abs(L1 - L2) < 0.01:
		print(f"✓ CLOSED LOOP: L1 == L2, path returns to start (0, 0)")
	else:
		print(f"⚠ Open path: L1 != L2, ends at ({L1-L2:.2f}, 0)")
	
	t_gait, p_gait, v_gait = generate_trajectory(
		wps_gait, vmax=2.0, amax=3.0, dt=0.01, stop_at_waypoints=False
	)
	print(f"\nTrajectory: {t_gait.size} samples over {t_gait[-1]:.2f} seconds")
	
	# Plot all trajectories
	print("\nGenerating plots...")
	# plot_trajectory(wps1, t, p, v, title="1D Stop-at-Waypoints")
	# plot_trajectory(wps2, t2, p2, v2, title="2D Pass-Through")
	plot_trajectory(wps_gait, t_gait, p_gait, v_gait, title=f"Gait Pattern (L1={L1:.2f}, L2={L2:.2f})")
