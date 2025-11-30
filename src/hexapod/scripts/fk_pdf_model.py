#!/usr/bin/env python3
"""
Pure PDF Kinematic Model - Hexapod Leg Kinematics
Based on: Canberk Suat Gurel — Hexapod Modelling, Path Planning and Control

This module implements the EXACT kinematic model from the research paper
using Denavit-Hartenberg (DH) parameters. This model is completely separate
from the URDF-aware model and does NOT include any URDF offsets.

Reference:
    - Research Paper: Canberk Suat Gurel (Section 3.3.1, Figure 3.3.1)
    - MATLAB Implementation: Appendix / Code Section
    - DH Convention: Standard DH (Craig)

Link Lengths (from URDF, but used in PDF frames):
    L1 = 0.0785 m  (Hip to Knee)
    L2 = 0.12 m    (Knee to Ankle)
    L3 = 0.1899 m  (Ankle to End Effector)

DH Parameters (EXACT from paper MATLAB code):
    Joint 1 (Hip):   a1=L1, α1=+90°, d1=0, θ1_offset=+90°
    Joint 2 (Knee):  a2=L2, α2=0°,   d2=0, θ2_offset=0°
    Joint 3 (Ankle): a3=L3, α3=-90°, d3=0, θ3_offset=-90°
    Frame 4:         a4=0,  α4=+90°, d4=0, θ4=-90° (fixed)

CRITICAL: This model operates in the PDF coordinate frame, NOT the URDF frame.
The output must be transformed by T_base_leg to get URDF positions.
"""

import numpy as np
import math
from typing import Tuple, List, Optional


# Link lengths (meters) - from URDF geometry
L1 = 0.0785   # Hip to Knee (computed from knee_joint offsets)
L2 = 0.12     # Knee to Ankle (computed from ankle_joint offsets)
L3 = 0.1899   # Ankle to End Effector (computed from foot offsets)

# Numerical tolerances
POSITION_TOLERANCE = 1e-4      # 0.1 mm - for IK verification
Z_AXIS_TOLERANCE = 1e-4        # Tolerance for z-axis projection in IK
NUMERICAL_EPSILON = 1e-6       # For numerical derivatives (Jacobian)
SINGULARITY_THRESHOLD = 1e-8   # Minimum norm to avoid division by zero
DEGENERATE_THRESHOLD = 1e-9    # Threshold for degenerate configurations


def dh_matrix(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    """
    Compute homogeneous transformation matrix using standard DH parameters.

    Reference: Research paper Section 3.3.1, MATLAB DH function

    DH Convention (Craig):
        T = Rot_Z(theta) * Trans_Z(d) * Trans_X(a) * Rot_X(alpha)

    Args:
        a: Link length (along x_{i-1})
        alpha: Link twist (rotation around x_{i-1})
        d: Link offset (along z_i)
        theta: Joint angle (rotation around z_i)

    Returns:
        4x4 homogeneous transformation matrix

    Reference Page: Section 3.3.1, Equation 3.1
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct,    -st*ca,  st*sa,   a*ct],
        [st,     ct*ca, -ct*sa,   a*st],
        [0,      sa,     ca,      d],
        [0,      0,      0,       1]
    ])


def fk_pdf(theta1: float, theta2: float, theta3: float) -> np.ndarray:
    """
    Forward Kinematics using EXACT PDF model from research paper.

    This function computes the end effector position in the PDF coordinate frame
    using the DH parameters EXACTLY as specified in the MATLAB code.

    DH Parameters (from paper):
        H01 = DH(L1, +90°, 0, θ1 + 90°)
        H12 = DH(L2,   0°, 0, θ2)
        H23 = DH(L3, -90°, 0, θ3 - 90°)
        H34 = DH(0,  +90°, 0, -90°)

    Args:
        theta1: Hip joint angle (radians)
        theta2: Knee joint angle (radians)
        theta3: Ankle joint angle (radians)

    Returns:
        3D position vector [x, y, z] in PDF frame

    Reference:
        - Paper Section 3.3.1, Figure 3.3.1
        - MATLAB code: FK function with DH parameters

    CRITICAL: Output is in PDF frame, NOT URDF frame!
    """
    # DH transformations EXACTLY from paper (MATLAB code)
    H01 = dh_matrix(L1, np.pi/2, 0, theta1 + np.pi/2)      # α1 = +90°, θ1_offset = +90°
    H12 = dh_matrix(L2, 0,       0, theta2)                 # α2 = 0°,   θ2_offset = 0°
    H23 = dh_matrix(L3, -np.pi/2, 0, theta3 - np.pi/2)     # α3 = -90°, θ3_offset = -90°
    H34 = dh_matrix(0,  np.pi/2,  0, -np.pi/2)             # α4 = +90°, θ4 = -90° (fixed)

    # Complete transformation from base to end effector
    H04 = H01 @ H12 @ H23 @ H34

    # Extract position (first 3 elements of last column)
    position = H04[0:3, 3]

    return position


def ik_pdf_analytical(
    px: float,
    py: float,
    pz: float,
    elbow_up: bool = True,
    z_tolerance: float = Z_AXIS_TOLERANCE,
) -> Optional[np.ndarray]:
    """
    Geometric Inverse Kinematics (PDF Model, same solution as old analytical IK)

    ใช้ geometry ตามรูป: L1, L2, L3, r1, r2, φ1, φ2, φ3

    ขั้นตอน (เฟรม PDF base เดิม):
    1) หา θ1 จากมุมในมุมมอง top view
           φ = atan2(py, px)
           θ1 = φ - π/2                (ตาม DH: atan2(Py, Px) = θ1 + π/2)

    2) เปลี่ยนเป็นปัญหา 2D (side view) ของข้อ 2–3
           ρ  = sqrt(px² + py²)
           r2 = ρ - L1                 (ระยะแนวนอนจากเข่าไปฉายปลายขา)
           r1 = sqrt(r2² + pz²)        (ระยะจากเข่าไปปลายขา)
           φ2 = atan2(pz, r2)

    3) ใช้ Law of Cosines ตามสมการในรูป
           φ1 = acos( (L3² - L2² - r1²) / (-2 L2 r1) )
           θ2(elbow-up)   = φ1 + φ2
           θ2(elbow-down) = φ2 - φ1

           φ3 = acos( (r1² - L2² - L3²) / (-2 L2 L3) )

       mapping ไปเป็น joint angle ของโมเดล PDF:
           สำหรับ elbow-up:
               θ3 = φ3 - π/2
           สำหรับ elbow-down:
               θ3 = (π - φ3) + π/2 = 3π/2 - φ3

    Args:
        px, py, pz : ตำแหน่งปลายขาในเฟรม PDF base (เหมือน output ของ fk_pdf)
        elbow_up   : True = ใช้ branch ข้อศอกงอขึ้น (เหมือนของเดิม),
                     False = branch อีกด้าน
        z_tolerance: ไว้เผื่อใช้ในอนาคต (ปัจจุบันไม่ใช้)

    Returns:
        np.ndarray [θ1, θ2, θ3] (rad) หรือ None ถ้าเอื้อมไม่ถึง
    """

    # 0) ป้องกันกรณี degenerate
    if np.linalg.norm([px, py, pz]) < SINGULARITY_THRESHOLD:
        return None

    # 1) θ1 จาก top view (เหมือนของเดิม เพื่อให้ตรงกับ DH)
    phi = math.atan2(py, px)
    theta1 = phi - math.pi / 2.0

    # 2) geometry ในระนาบด้านข้าง
    rho = math.hypot(px, py)          # ระยะในระนาบ X0–Y0
    r2 = rho - L1                     # แนวนอนจากเข่าไปฉายปลายขา
    r1 = math.hypot(r2, pz)           # ระยะจากเข่าไปปลายขา

    if r1 < SINGULARITY_THRESHOLD:
        return None

    phi2 = math.atan2(pz, r2)         # มุมของ r1 กับแกน r2

    # 3) คำนวณ φ1 และ φ3 ตามสมการที่คุณแนบมา
    c1 = (L3**2 - L2**2 - r1**2) / (-2.0 * L2 * r1)
    c3 = (r1**2 - L2**2 - L3**2) / (-2.0 * L2 * L3)

    # ตรวจ reachability
    if c1 < -1.0 - 1e-6 or c1 > 1.0 + 1e-6:
        return None
    if c3 < -1.0 - 1e-6 or c3 > 1.0 + 1e-6:
        return None

    # clamp ป้องกัน numerical error
    c1 = max(-1.0, min(1.0, c1))
    c3 = max(-1.0, min(1.0, c3))

    phi1 = math.acos(c1)
    phi3 = math.acos(c3)

    # 4) หา θ2, θ3 ตาม branch ที่เลือก
    if elbow_up:
        # ตามสมการในรูป:
        #   θ2 = φ1 + φ2
        #   θ3 = φ3 - π/2
        theta2 = phi1 + phi2
        theta3 = phi3 - math.pi / 2.0
    else:
        # solution อีกด้าน:
        #   θ2 = φ2 - φ1
        #   θ3 = (π - φ3) + π/2 = 3π/2 - φ3
        theta2 = phi2 - phi1
        theta3 = (math.pi - phi3) + math.pi / 2.0

    candidate = np.array([theta1, theta2, theta3], dtype=float)

    # 5) ตรวจสอบย้อนกลับกับ FK เหมือนฟังก์ชันเดิม
    p_check = fk_pdf(*candidate)
    err = np.linalg.norm(p_check - np.array([px, py, pz], dtype=float))
    if err > POSITION_TOLERANCE:
        return None

    return candidate



def ik_pdf_both_solutions(
    px: float,
    py: float,
    pz: float
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """
    Compute BOTH elbow-up and elbow-down solutions for IK.

    Args:
        px, py, pz: Target position in PDF frame

    Returns:
        (elbow_up_solution, elbow_down_solution)
        Each is [θ1, θ2, θ3] if reachable, None otherwise

    Reference: Paper Section 3.3.2 (discusses multiple solutions)
    """
    sol_up = ik_pdf_analytical(px, py, pz, elbow_up=True)
    sol_down = ik_pdf_analytical(px, py, pz, elbow_up=False)

    return sol_up, sol_down


def is_within_workspace(px: float, py: float, pz: float) -> bool:
    """
    Check if a target position is within the workspace.

    This is a conservative estimate. Due to the complex DH structure
    with offsets and twists, the actual workspace is more complex
    than a simple spherical shell.

    Args:
        px, py, pz: Target position in PDF frame

    Returns:
        True if within workspace, False otherwise

    Reference: Paper Section 3.4 (Workspace Analysis)
    """
    # Distance from origin
    d = np.sqrt(px**2 + py**2 + pz**2)

    # Maximum reach (fully extended)
    max_reach = L1 + L2 + L3

    # Minimum reach is harder to compute due to DH offsets
    # For conservative estimate, use a small value
    min_reach = 0.01  # 10mm minimum

    # Basic spherical check
    if d < min_reach or d > max_reach:
        return False

    # More sophisticated check would require full IK solving
    # For now, return True if within spherical bounds
    return True


def compute_jacobian_pdf(theta1: float, theta2: float, theta3: float) -> np.ndarray:
    """
    Compute Jacobian matrix for PDF model using numerical differentiation.

    Args:
        theta1, theta2, theta3: Joint angles (radians)

    Returns:
        3x3 Jacobian matrix J where v = J * theta_dot

    Reference: Paper Section 3.5 (Velocity Kinematics)
    """
    J = np.zeros((3, 3))

    pos = fk_pdf(theta1, theta2, theta3)

    # Partial derivative with respect to theta1
    pos_theta1 = fk_pdf(theta1 + NUMERICAL_EPSILON, theta2, theta3)
    J[:, 0] = (pos_theta1 - pos) / NUMERICAL_EPSILON

    # Partial derivative with respect to theta2
    pos_theta2 = fk_pdf(theta1, theta2 + NUMERICAL_EPSILON, theta3)
    J[:, 1] = (pos_theta2 - pos) / NUMERICAL_EPSILON

    # Partial derivative with respect to theta3
    pos_theta3 = fk_pdf(theta1, theta2, theta3 + NUMERICAL_EPSILON)
    J[:, 2] = (pos_theta3 - pos) / NUMERICAL_EPSILON

    return J


# ============================================================================
# HELPER FUNCTIONS FOR DEBUGGING AND VALIDATION
# ============================================================================

def validate_fk_ik_round_trip(theta, tolerance=POSITION_TOLERANCE):
    pos = fk_pdf(*theta)

    # ลองทั้งสอง branch
    sol_up  = ik_pdf_analytical(*pos, elbow_up=True)
    sol_down = ik_pdf_analytical(*pos, elbow_up=False)

    best_err = float('inf')
    for sol in (sol_up, sol_down):
        if sol is None:
            continue
        pos_rec = fk_pdf(*sol)
        err = np.linalg.norm(pos - pos_rec)
        best_err = min(best_err, err)

    if best_err == float('inf'):
        return False, best_err  # หาไม่ได้ทั้งสอง branch

    return best_err < tolerance, best_err



def print_dh_table():
    """
    Print DH parameter table for reference.

    Reference: Paper Table 3.1 (DH Parameters)
    """
    print("=" * 70)
    print("DH PARAMETERS - PDF Model (from research paper)")
    print("=" * 70)
    print(f"{'Joint':<8} {'a (m)':<12} {'α (deg)':<12} {'d (m)':<12} {'θ offset (deg)':<18}")
    print("-" * 70)
    print(f"{'1 (Hip)':<8} {L1:<12.4f} {'+90':<12} {'0':<12} {'+90':<18}")
    print(f"{'2 (Knee)':<8} {L2:<12.4f} {'0':<12} {'0':<12} {'0':<18}")
    print(f"{'3 (Ankle)':<8} {L3:<12.4f} {'-90':<12} {'0':<12} {'-90':<18}")
    print(f"{'4 (Frame)':<8} {'0':<12} {'+90':<12} {'0':<12} {'-90 (fixed)':<18}")
    print("=" * 70)
    print(f"Total reach: {L1 + L2 + L3:.4f} m = {(L1 + L2 + L3)*1000:.2f} mm")
    print("=" * 70)


if __name__ == '__main__':
    """
    Test the PDF kinematic model
    """
    print_dh_table()
    print()

    # Test FK at zero position
    print("Test 1: FK at [0, 0, 0]")
    pos_zero = fk_pdf(0, 0, 0)
    print(f"Position: {pos_zero}")
    print(f"  X: {pos_zero[0]:.6f} m")
    print(f"  Y: {pos_zero[1]:.6f} m")
    print(f"  Z: {pos_zero[2]:.6f} m")
    print()

    # Test FK at sample configuration
    print("Test 2: FK at [0, π/4, -π/4]")
    pos_sample = fk_pdf(0, np.pi/4, -np.pi/4)
    print(f"Position: {pos_sample}")
    print(f"  X: {pos_sample[0]:.6f} m")
    print(f"  Y: {pos_sample[1]:.6f} m")
    print(f"  Z: {pos_sample[2]:.6f} m")
    print()

    # Test IK with reachable position (from FK result)
    print("Test 3: IK round-trip test")
    # Use the position from Test 2 as our target
    target = np.array([0.0, 0.163353, -0.105047])
    print(f"Target position: [{target[0]:.6f}, {target[1]:.6f}, {target[2]:.6f}]")
    theta_ik = ik_pdf_analytical(*target, elbow_up=True)
    if theta_ik is not None:
        print(f"Solution: {theta_ik}")
        print(f"  θ1: {np.degrees(theta_ik[0]):.2f}°")
        print(f"  θ2: {np.degrees(theta_ik[1]):.2f}°")
        print(f"  θ3: {np.degrees(theta_ik[2]):.2f}°")

        # Verify round trip
        pos_verify = fk_pdf(*theta_ik)
        error = np.linalg.norm(target - pos_verify)
        print(f"Round-trip error: {error:.6f} m")
        if error < 1e-3:
            print(f"  ✓ Within tolerance!")
    else:
        print("Target unreachable!")
    print()

    # Test workspace
    print("Test 4: Workspace check")
    print(f"Target {target}: {'REACHABLE' if is_within_workspace(*target) else 'UNREACHABLE'}")
    unreachable = np.array([0.5, 0.5, 0.5])
    print(f"Target {unreachable}: {'REACHABLE' if is_within_workspace(*unreachable) else 'UNREACHABLE'}")
