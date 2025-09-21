# Autonomous Vehicles — Occupancy Grid Mapper

> A compact 2‑D occupancy‑grid mapper for a teaching autonomous vehicle (MacAEV). LiDAR scans are fused with wheel/IMU odometry to estimate obstacle occupancy in real time and publish a `nav_msgs/OccupancyGrid` map.

---

## 🚗 Overview

This project implements the classical **occupancy grid mapping** pipeline on a small autonomous platform. The mapper consumes:

* **Localization:** wheel odometry for $x,y$ translation and **IMU yaw** for heading ($\theta$)
* **Perception:** 2‑D **LiDAR** scan returns (ranges and angles)

and produces a planar grid in the fixed **odometry** frame (`odom`) whose cells encode the probability of being **occupied**, **free**, or **unknown**.

The repository contains:

* `OccupancyGridMapping_.py` – ROS node (Python) that subscribes to Odometry and LaserScan, updates log‑odds for each cell, and publishes a `nav_msgs/OccupancyGrid`.
* `expermient_cpp_.txt` – the wheel‑odometry (C++) logic used in a prior lab; included here for context.
* `Activity4_.bag` – example rosbag recorded during mapping (for playback/inspection).

---

## 🔧 Platform & Frames (ROS tf)

Mapping operates in a consistent set of coordinate frames:

* **`odom`** – fixed world frame for the session (map is published in this frame).
* **`base_link`** – body frame attached to the vehicle’s rear axle center.
* **`laser`** – LiDAR frame (known static transform to `base_link`).

**Pose state.** The vehicle pose in `odom` is $X=[x\ y\ \theta]^T$. Wheel odometry supplies $x,y$; IMU supplies $\theta$.

<!-- Fig 1 (from lab Fig 5): kinematic bicycle pose definition in odom/base_link. Replace the URL below after dragging the image into the README. -->

<img width="716" height="422" alt="SCR-20250921-lraz" src="https://github.com/user-attachments/assets/557e761b-5474-4ca3-ae2d-1319fa63c426" />

---

## 🧭 Localization (Wheel + IMU yaw)

The kinematic bicycle model gives the continuous‑time motion

$$
\dot x = v_s\cos\theta,\quad\dot y = v_s\sin\theta,\quad \dot\theta = \frac{v_s}{\ell}\tan\delta,
$$

with speed $v_s$, steering angle $\delta$, and wheelbase $\ell$. Discrete integration (Euler) updates translation, while **heading** is read directly from the IMU yaw (after subtracting the initial bias $\theta_0$):

$$
\begin{aligned}
 x_{k+1}&=x_k + \Delta t_k\,v_{s,k}\cos\theta_k,\\
 y_{k+1}&=y_k + \Delta t_k\,v_{s,k}\sin\theta_k,\\
 \theta_k&=\text{yaw}^{\text{IMU}}_k-\theta_0.
\end{aligned}
$$

This reduces **drift** from integrating angular rate and keeps `odom→base_link` consistent over short horizons.

> The C++ snippet in `expermient_cpp_.txt` shows this wheel/IMU fusion and publishes both a TF (`odom→base_link`) and a `nav_msgs/Odometry` message.

---

## 🗺️ Occupancy Grid Mapping – Intuition

An **occupancy grid** discretizes the plane into cells $m_{ij}$ of size **resolution** (meters/cell). Each cell stores $p(m_{ij})$, the probability of being occupied.

* If $p(m_{ij})\ge p_{\text{occ}}$, the cell is **occupied**.
* If $p(m_{ij})\le p_{\text{free}}$, the cell is **free**.
* Otherwise it is **unknown**.

**Ray casting (inverse sensor model).** For the current pose $x_k=(\theta_k, d^o_{b,k})$ and LiDAR scan $z_k$, each cell chooses the **closest ray** to the line from the LiDAR origin to the cell center. Conditioned on that ray:

* the cell is **occupied** if the measured range terminates **inside** the cell,
* the cell is **free** if the measured range is **beyond** the cell,
* otherwise **unknown**.

<!-- Fig 2 (from lab Fig 6): grid & frames with ray projection. Replace URL after dropping the image. -->

<img width="745" height="531" alt="SCR-20250921-lrei" src="https://github.com/user-attachments/assets/fb3e2a15-66c8-4108-8ccc-a82362c7f249" />

<!-- Fig 3 (from lab Fig 7): inverse sensor model (free/occupied/unknown coloring). Replace URL after dropping the image. -->

<img width="722" height="562" alt="SCR-20250921-lrfe" src="https://github.com/user-attachments/assets/616cf8ff-7911-4044-98bc-3daf7c56bf57" />

### Log‑odds update (probabilistic fusion)

Using log‑odds $\ell(y)=\log\tfrac{p(y)}{1-p(y)}$, the recursive update per cell is

$$
\ell\big(m_{ij}\mid z_{0:k},x_{0:k}\big) \;=\; \underbrace{\ell\big(m_{ij}\mid z_k,x_k\big)}_{\text{inverse sensor}}\; +\; \underbrace{\ell\big(m_{ij}\mid z_{0:k-1},x_{0:k-1}\big)}_{\text{prior}} \; -\; \ell(m_{ij}).
$$

With an **uninformative prior** $p(m_{ij})=0.5\Rightarrow\ell(m_{ij})=0$, this becomes a simple accumulate‑and‑threshold scheme. Probabilities recover as
$p(y)=\frac{1}{1+e^{-\ell(y)}}.$

---

## 🧩 Mapper Pipeline (what the node does)

1. **Subscribe** to `nav_msgs/Odometry` (pose in `odom`) and `sensor_msgs/LaserScan` (ranges $r_n$, angles $\alpha_n$).
2. **Cache pose** $x_k$ on odom callback; **process scan** on laser callback.
3. For each cell center $c_{ij}$:

   * Transform $c_{ij}$ into the LiDAR frame using `odom→base_link→laser` tf.
   * Find closest beam index $n^*$ to the bearing of $c_{ij}$.
   * Apply the **inverse sensor model** with constants $p_{\text{occ}}, p_{\text{free}}, p_{\text{unk}}=0.5$.
   * **Accumulate log‑odds** and clamp to $[\ell_{\min},\ell_{\max}]$ to avoid saturation.
4. **Publish** a `nav_msgs/OccupancyGrid` with width/height/resolution/origin, converting log‑odds to {100=occupied, 0=free, −1=unknown}.

### Minimal pseudocode

```python
# inside OccupancyGridMapping_.py
for each lidar_callback(scan):
    pose = latest_odom_pose  # (x, y, theta) in 'odom'
    for each cell (i,j):
        c_odom = origin + (i*res, j*res)
        c_laser = tf_odom_to_laser(pose).inverse() @ c_odom
        phi = atan2(c_laser.y, c_laser.x)
        n = nearest_ray_index(phi, scan.angle_min, scan.angle_increment)
        r = scan.ranges[n]
        d = hypot(c_laser.x, c_laser.y)
        if hit_inside_cell(r, d, res):       l_ij += logit(p_occ)
        elif r > d + margin:                 l_ij += logit(p_free)
        else:                                l_ij += logit(0.5)
    publish_occupancy_grid(L = l)
```

> The full implementation in `OccupancyGridMapping_.py` batches these steps efficiently, respects message timestamps, and uses row‑major indexing for `info.width × info.height`.

---

## 🧪 Results (bag playback & expected behavior)

* During a hallway drive with cardboard obstacles, the map progressively **carves free space** along traversed beams and **marks obstacles** where returns terminate.
* IMU‑based heading significantly reduces yaw drift; residual translation drift appears as slight mis‑alignment on loop closures (expected without scan‑matching).

Consider adding the following visuals (drop them into the README and replace the placeholders):

* **RViz screenshot** showing the `Map` display, `TF` tree (`odom` & `base_link`), and the evolving grid.
* **Final map snapshot** after a loop.

Placeholders to replace once images are uploaded:

* `RVIZ_SCREENSHOT_URL`
* `FINAL_MAP_URL`

Example Markdown (leave until links are available):

```markdown
![RViz – live mapping](RVIZ_SCREENSHOT_URL)
![Final occupancy grid](FINAL_MAP_URL)
```

---

## 📁 Message & Parameter Notes

* **`nav_msgs/OccupancyGrid`** fields used: `info.resolution` (m/cell), `info.width/height` (cols/rows), `info.origin` (grid (0,0) in `odom`), `header.frame_id='odom'`.
* **Thresholds**: `p_occ > 0.5`, `p_free < 0.5`, unknown = 0.5. Tune `p_occ`, `p_free`, grid **resolution**, and map **width/height** to balance detail vs compute.
* **Clamping**: cap log‑odds to `[ℓ_min, ℓ_max]` to avoid irrecoverable saturation from repeated updates.

---

## 📷 Figure placeholders (to be replaced with GitHub attachment links)

Replace the three placeholders below using the **drag‑and‑drop** method (GitHub will insert a unique URL). These correspond to Lab Figs 5–7 but are renumbered here as Figs 1–3.

* `FIG1_PLACEHOLDER_URL` – *Vehicle pose & frames (odom vs base\_link)*
* `FIG2_PLACEHOLDER_URL` – *Grid in `odom` and LiDAR projection*
* `FIG3_PLACEHOLDER_URL` – *Inverse sensor model (free/occupied/unknown)*

---

* Why use **IMU yaw** directly? Eliminates integrating noisy gyro and limits drift in heading, which strongly affects mapping geometry.
* Limitations: assumes **static** world and **known** poses; moving obstacles and odometry drift cause artifacts. Next steps include **scan‑matching** (ICP), **loop closure**, or full **SLAM**.

---

## License

Released under the **MIT License** — see [LICENSE](LICENSE).
