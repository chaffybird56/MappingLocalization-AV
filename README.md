# Occupancy Grid Mapping (Autonomous Vehicle Lab)

> Real‑time 2‑D mapping from a LiDAR scan using a Bayesian inverse‑sensor model and ray‑tracing to carve free space. Publishes a ROS `nav_msgs/OccupancyGrid` (−1 = unknown, 0 = free, 100 = occupied).

---

## 🛰️ Overview

This project implements a lightweight **occupancy grid mapper** for an autonomous vehicle platform (MacAEV). The node consumes a **LiDAR** scan and a **vehicle pose** (from odometry/IMU/TF), and updates a discretized 2‑D map in **log‑odds**. Each new scan sharpens beliefs of **free** vs **occupied** cells. The resulting grid can be visualized in RViz or used by planners/localizers.

**What you’ll see**

* A grid in the `map`/`odom` frame with cells colored **free**, **occupied**, or **unknown**.
* A characteristic **free‑space fan** where the laser passes and **occupied rims** at obstacle hits.

**Key ideas**

* **Inverse sensor model** for each beam (cells before hit → free, cell at hit → occupied)
* **Log‑odds** fusion for stable, incremental updates with clamping
* **Ray tracing (Bresenham)** on the grid to mark free cells efficiently

---

## ⚡ Highlights

* **Simple & robust:** One pass per scan beam; constant‑time cell updates.
* **Numerically stable:** Log‑odds accumulation with configurable clamps.
* **ROS‑native output:** `nav_msgs/OccupancyGrid` (row‑major `int8` data, 0–100 with −1 unknown).
* **Portable:** Works with wheel/IMU odometry or pose from TF; agnostic to LiDAR brand as long as a `LaserScan` is provided.

---

## 🧠 Method (short & sweet)

For each cell $m_{ij}$ and scan $z_t$ at robot pose $x_t$, we maintain the **log‑odds** of occupancy

$$
\ell_t(m_{ij}) = \ell_{t-1}(m_{ij}) \; + \; \underbrace{\log\!\frac{p(m_{ij}\mid z_t,x_t)}{1-p(m_{ij}\mid z_t,x_t)}}_{\text{inverse sensor model}} \; - \; \ell_0(m_{ij}).
$$

With prior $\ell_0=0$ (i.e., $p=0.5$), updates reduce to adding **$\ell_{\text{free}}<0$** for along‑ray cells and **$\ell_{\text{occ}}>0$** at the hit cell, then clamping to $[\ell_{\min},\ell_{\max}]$. Probabilities are recovered by $p = 1 - 1/(1+e^{\ell})$.

**Beam processing**

1. Transform the beam origin and direction to the map frame.
2. Compute the hit point from range $r$ and angle $\theta$.
3. **Ray‑trace** from origin → hit through grid cells (Bresenham).
     • Cells **before** the hit: add $\ell_{\text{free}}$.
     • Cell **at** the hit: add $\ell_{\text{occ}}$ and stop.

---

## 🧩 Algorithm (tiny pseudocode)

```text
on LaserScan(scan):
  (sx, sy, syaw) ← sensor pose in map frame
  for k in [0..N):
    r, θ ← scan.ranges[k], scan.angle_min + k*scan.angle_increment
    if r invalid: continue
    hit ← (sx + r*cos(syaw+θ), sy + r*sin(syaw+θ))
    for cell in bresenham((sx,sy), hit):
      if dist(cell,(sx,sy)) < r - δ: add l_free
      else: add l_occ; break
  publish OccupancyGrid(grid)
```

*(Your `OccupancyGridMapping.py` already implements this flow with helpers for world↔map indexing and a Bresenham‑style traversal.)*

---

## 🔎 Inputs & Output

**Inputs**

* `sensor_msgs/LaserScan` (angles/ranges)
* Vehicle pose in `map`/`odom` (from `nav_msgs/Odometry` or TF)

**Output**

* `nav_msgs/OccupancyGrid` with

  * `info.resolution`, `width`, `height`, `origin` (pose of cell (0,0))
  * `data` as row‑major `int8` (−1 unknown, 0 free, 100 occupied)

---

## 📊 Results (what to showcase)

Add a few visuals (drop under `media/` and link below):

<p align="center">
  <img src="media/rviz_overlay.png" width="800" alt="RViz overlay of the occupancy grid">
  <br>
  <sub><b>Fig 1.</b> RViz view: free‑space fan with occupied returns along track barriers.</sub>
</p>

<table>
  <tr>
    <td align="center">
      <img src="media/beam_model.png" width="380" alt="Inverse sensor model cartoon"><br>
      <sub><b>Fig 2.</b> Inverse sensor model: along‑ray <i>free</i>, hit cell <i>occupied</i>.</sub>
    </td>
    <td align="center">
      <img src="media/map_progress.gif" width="380" alt="Map refinement over time"><br>
      <sub><b>Fig 3.</b> Map refinement as scans accumulate.</sub>
    </td>
  </tr>
</table>

---

## 🧭 Design notes

* **Resolution vs. speed:** 0.03–0.10 m/cell is a practical range; smaller cells look sharper but cost CPU.
* **Clamping:** Keep $\ell$ within $[\ell_{\min},\ell_{\max}]$ to avoid saturation from repeated hits.
* **Frame IDs:** Ensure `grid.header.frame_id` matches your fixed frame (`map`/`odom`).
* **Unknown handling:** Start grids as −1; update only what rays touch to avoid biasing unexplored space.

---

## 📎 Code pointers

* **`src/OccupancyGridMapping.py`**
  • `scan_cb` — main update loop
  • `sensor_pose_in_map()` — TF/odometry lookup
  • `bresenham(a,b)` — integer grid traversal
  • `set_free` / `set_occupied` — write row‑major `data`

---

## 🙌 Acknowledgements

Classical occupancy grids with inverse sensor models and log‑odds fusion are described in standard robotics texts and ROS docs. This implementation adapts those ideas for the AV lab setup (MacAEV) and LiDAR scan geometry.
