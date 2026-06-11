#!/usr/bin/env python3
"""Generate swarm_autonomy_city.sdf — a self-contained Gazebo Harmonic urban world for
the Swarm Autonomy swarm demos.

A grid of box "buildings" of varied heights over a ~60 m block, with clear
streets (and a clear spawn corridor along the y=0 line where the drones start).
Box geometry only, so there are no external model dependencies — it just loads.
Built on PX4's default world (same physics / scene / ground / sun / spherical
coords) so the x500's sensors keep working.

    python3 gen_city.py            # writes swarm_autonomy_city.sdf next to this file
"""

from __future__ import annotations

import os

# Deterministic pseudo-random heights/colours (no RNG import needed for repro).
def _h(i, j):
    return 5.0 + ((i * 7 + j * 13) % 5) * 1.75         # 5..12 m (low-rise; swarm flies above)

def _shade(i, j):
    base = 0.45 + ((i * 3 + j * 5) % 4) * 0.08          # grey variation
    return base


def building(cx, cy, w, d, h, shade):
    z = h / 2.0
    return f"""    <model name="b_{int(cx)}_{int(cy)}">
      <static>true</static>
      <pose>{cx} {cy} {z} 0 0 0</pose>
      <link name="link">
        <collision name="c">
          <geometry><box><size>{w} {d} {h}</size></box></geometry>
        </collision>
        <visual name="v">
          <geometry><box><size>{w} {d} {h}</size></box></geometry>
          <material>
            <ambient>{shade} {shade} {shade+0.03} 1</ambient>
            <diffuse>{shade} {shade} {shade+0.03} 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def make_city() -> str:
    blocks = []
    # 5x5 grid of building plots over [-24,24] m, 12 m pitch, 7x7 m footprints.
    coords = range(-24, 25, 12)
    for i, cx in enumerate(coords):
        for j, cy in enumerate(coords):
            # Keep a clear spawn/flight corridor along y=0 (the street the drones launch from).
            if abs(cy) < 6:
                continue
            blocks.append(building(cx, cy, 7.0, 7.0, _h(i, j), _shade(i, j)))
    buildings = "".join(blocks)

    return f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="swarm_autonomy_city">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.6 0.75 0.92 1</background>
      <shadows>true</shadows>
    </scene>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
          <surface><friction><ode/></friction><bounce/><contact/></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry>
          <material>
            <ambient>0.33 0.33 0.35 1</ambient>
            <diffuse>0.33 0.33 0.35 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 -0 0</pose>
        <inertial><pose>0 0 0 0 -0 0</pose><mass>1</mass>
          <inertia><ixx>1</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1</iyy><iyz>0</iyz><izz>1</izz></inertia>
        </inertial>
        <enable_wind>false</enable_wind>
      </link>
      <pose>0 0 0 0 -0 0</pose>
      <self_collide>false</self_collide>
    </model>
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation><range>2000</range><linear>0</linear><constant>1</constant><quadratic>0</quadratic></attenuation>
      <spot><inner_angle>0</inner_angle><outer_angle>0</outer_angle><falloff>0</falloff></spot>
    </light>
{buildings}    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg> 8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
  </world>
</sdf>
"""


if __name__ == "__main__":
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "swarm_autonomy_city.sdf")
    with open(out, "w") as f:
        f.write(make_city())
    n = make_city().count("<model name=\"b_")
    print(f"wrote {out} ({n} buildings)")
