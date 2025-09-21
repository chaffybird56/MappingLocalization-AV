# Autonomous Vehicles — Occupancy Grid Mapper

> A compact 2‑D occupancy‑grid mapper for a teaching autonomous vehicle (MacAEV). LiDAR scans are fused with wheel/IMU odometry to estimate obstacle occupancy in real time and publish a `nav_msgs/OccupancyGrid` map.

---

## 🚗 Overview

This project implements the classical **occupancy‑grid mapping** pipeline on a small autonomous platform. The mapper consumes:

* **Localization:** wheel odometry for $x,y$ translation and **IMU yaw** for heading ($\theta$)
* **Perception:** 2‑D **LiDAR** scan returns (ranges and angles)

and produces a planar grid in the fixed **odometry** frame (`odom`) whose cells encode the probability of being **occupied**, **free**, or **unknown**.

**Big picture.** The vehicle drives, the LiDAR sends out many straight‑line beams each cycle, and the mapper converts what those beams “saw” into colored squares on a grid (free along the beam, occupied where it hits). Accumulating evidence over time sharpens the map.

Repository layout:

* `OccupancyGridMapping_.py` — ROS node (Python) that subscribes to Odometry and LaserScan, updates log‑odds per cell, and publishes a `nav_msgs/OccupancyGrid`.
* `expermient_cpp_.txt` — wheel‑odometry (C++) logic used in a prior lab; included for context.
* `Activity4_.bag` — example rosbag recorded during mapping (for playback/inspection).

---

## 🔧 Platform & Frames (ROS tf)

**Quick ROS primer (context).**

* **ROS** provides a pub/sub system for robot data. Programs are **nodes**; they exchange typed **messages** on named **topics** (e.g., `/scan` for LiDAR, `/odom` for wheel+IMU pose).
* A **TF tree** tracks coordinate frames (e.g., `odom → base_link → laser`). TF lets any node transform points between frames with correct timing.
* A **map** in this project is a `nav_msgs/OccupancyGrid` (a 2‑D array with metadata).

**Frames used here.**

* **`odom`** — fixed world frame for the session (the grid is published here).
* **`base_link`** — body frame attached to the vehicle’s chassis.
* **`laser`** — LiDAR frame (static transform from `base_link`).

**Pose state.** The vehicle pose in `odom` is $X=[x\ y\ \theta]^T$. Wheel odometry supplies $x,y$; IMU provides $\theta$.

![Fig 1 — Vehicle pose & frames (odom/base\_link/laser)](https://github.com/user-attachments/assets/557e761b-5474-4ca3-ae2d-1319fa63c426) <sub><b>Fig 1.</b> Coordinate frames and pose definition. The map is attached to <code>odom</code>; LiDAR originates in <code>laser</code>.</sub>

---

## 🧭 Localization (Wheel + IMU yaw)

**What “odometry” means here.** Odometry estimates pose by integrating measured motion: wheel travel gives a forward speed; steering sets the turn rate; IMU yaw provides absolute heading. Heading from the IMU avoids accumulating gyro drift and stabilizes geometry for mapping.

**Kinematic bicycle model (continuous time):**

$$
\dot x = v_s\cos\theta,\qquad
\dot y = v_s\sin\theta,\qquad
\dot\theta = \frac{v_s}{\ell}\tan\delta,
$$

with speed $v_s$, steering angle $\delta$, and wheelbase $\ell$.

**Discrete updates (Euler) with IMU yaw for heading:**

$$
\begin{aligned}
 x_{k+1}&=x_k + \Delta t_k\,v_{s,k}\cos\theta_k,\\
 y_{k+1}&=y_k + \Delta t_k\,v_{s,k}\sin\theta_k,\\
 \theta_k&=\text{yaw}^{\mathrm{IMU}}_{k}-\theta_0\, .
\end{aligned}
$$

The C++ excerpt in `expermient_cpp_.txt` publishes both a TF ($\text{odom}\to\text{base\_link}$) and `nav_msgs/Odometry` with this fusion.

---

## 🗺️ Occupancy Grid Mapping — Intuition first

An **occupancy grid** divides the plane into small cells (meters per cell). Each cell stores a belief $p(m_{ij})$ that something solid occupies that square.

**How LiDAR evidence maps to cells.** For any cell center, the algorithm picks the **closest LiDAR beam** to that direction and reasons:

* along the beam **before** the hit → **free**;
* the **cell containing the hit** → **occupied**;
* just beyond the hit → **unknown**.

![Fig 2 — Grid in odom, LiDAR rays in laser, and projection to cells](https://github.com/user-attachments/assets/fb3e2a15-66c8-4108-8ccc-a82362c7f249) <sub><b>Fig 2.</b> Cells are updated by projecting LiDAR rays from <code>laser</code> into the grid in <code>odom</code>.</sub>

![Fig 3 — Inverse sensor model: along‑ray cells free, hit cell occupied](https://github.com/user-attachments/assets/616cf8ff-7911-4044-98bc-3daf7c56bf57) <sub><b>Fig 3.</b> Inverse sensor model: free along the ray (before range), occupied at the terminal cell, unknown past the hit.</sub>

---

## 🧮 From probabilities to log‑odds (and back)

Directly adding probabilities is awkward. Each cell instead holds **log‑odds**

$$
\ell=\log\frac{p}{1-p}\, .
$$

The recursive update per cell is

$$
\ell_t\big(m_{ij}\big) = \ell_{t-1}\big(m_{ij}\big) + \underbrace{\ell\big(m_{ij}\mid z_t,x_t\big)}_{\text{inverse sensor contribution}} - \ell_0\big(m_{ij}\big)\, .
$$

With an uninformative prior ($p_0 = 0.5 \Rightarrow \ell_0 = 0$), this becomes adding a negative constant for **free** cells and a positive constant for **occupied** cells, with clamping to $[\ell_{\min},\ell_{\max}]$. Probability recovery uses

$$
 p = \frac{1}{1+e^{-\ell}}\, .
$$

---

## 🧩 Mapper pipeline (what the node actually does)

1. **Subscribe** to `nav_msgs/Odometry` (pose in `odom`) and `sensor_msgs/LaserScan` (ranges $r_n$, angles $\alpha_n$).
2. **Cache pose** $x_k$ on odom callback; **process scan** on laser callback.
3. For each **cell center** $c_{ij}$: transform to the LiDAR frame, select nearest beam, compare distance $d=\lVert c_{\text{laser}}\rVert$ to measured range $r$, and add the appropriate log‑odds increment (free / occupied / unknown). Clamp values.
4. **Publish** `nav_msgs/OccupancyGrid`: set `info.resolution`, `info.width/height`, `info.origin` (pose of cell (0,0) in `odom`), and row‑major `data` where **100** = occupied, **0** = free, **−1** = unknown.

### Minimal pseudocode (annotated)

```python
# OccupancyGridMapping_.py (sketch)
def laser_cb(scan):
    pose = latest_odom_pose  # (x, y, theta) in 'odom'
    T_odom_laser = tf_odom_to_laser(pose)
    for (i, j) in all_grid_cells:
        c_odom  = origin_xy + np.array([i, j]) * res
        c_laser = inv(T_odom_laser) @ to_homog(c_odom)
        phi     = atan2(c_laser.y, c_laser.x)
        n       = nearest_index(phi, scan.angle_min, scan.angle_increment)
        r       = scan.ranges[n]
        d       = hypot(c_laser.x, c_laser.y)
        if hit_inside_cell(r, d, res):
            L[i, j] = clamp(L[i, j] + logit(p_occ))
        elif r > d + margin:
            L[i, j] = clamp(L[i, j] + logit(p_free))
        # else unknown
    publish_grid(L)
```

---

## 🧪 Expected behavior (qualitative)

Observed during lab playback and consistent with the documents:

* **Free‑space fans** accumulate as the platform moves, carving corridors of free cells.
* **Obstacle rims** appear at beam terminations (walls, carts, cardboard).
* **Unknown regions** persist where no beam ever passed (behind obstacles, unobserved corners).
* **Heading stabilization** from IMU yaw reduces rotational drift; small translation drift may misalign loop closures (expected without scan‑matching).
* **Resolution trade‑off:** $0.05\!\text{ m}$–$0.10\!\text{ m}$ per cell balances crispness vs. compute.

Suggested visuals to add later (replace with real uploads):

```markdown
![RViz — live mapping](RVIZ_SCREENSHOT_URL)
![Final occupancy grid](FINAL_MAP_URL)
```

---

## 🧰 Practical parameters (typical)

* `resolution`: **0.05–0.10 m/cell**
* `p_occ`: **0.65–0.75**  → $\ell_{\text{occ}} = \log\!\frac{p_{\text{occ}}}{1-p_{\text{occ}}}$
* `p_free`: **0.30–0.45** → $\ell_{\text{free}} = \log\!\frac{p_{\text{free}}}{1-p_{\text{free}}}$ (negative)
* `l_min`, `l_max`: **\[−4.0, +4.0]**
* `margin`: \~ **0.5 × resolution**
* `frame_id`: `'odom'` for the published grid; ensure TF tree is consistent

---

## 🧾 Message & data conventions

* Grid indexing is **row‑major**: `data[row*width + col]`.
* **Origin** (`info.origin`) stores the pose of cell (0,0) in `odom`. Sliding windows require updating this origin.
* **Unknown** cells (−1) remain until evidence arrives; no prior is injected.

---

## 🗣️ Glossary (quick context)

* **LiDAR beam** — a ray with known angle and measured range.
* **Inverse sensor model** — rule mapping a beam’s geometry to cell beliefs.
* **Log‑odds** — $\ell=\log\tfrac{p}{1-p}$; turns probabilistic fusion into addition.
* **Bresenham / ray‑tracing** — integer grid traversal visiting all crossed cells.
* **TF** — ROS transform system between frames (e.g., `odom→base_link→laser`).

---

## 📚 How this ties together

1. **Pose** from wheel+IMU determines the LiDAR origin in `odom`.
2. **Each scan** is a bundle of rays; intersecting them with the grid yields free and occupied evidence.
3. **Log‑odds** accumulate evidence over time, resisting noise and enabling correction.
4. The published **OccupancyGrid** feeds planners, localizers, and RViz visualization.

---

## License

Released under the **MIT License** — see [LICENSE](LICENSE).
