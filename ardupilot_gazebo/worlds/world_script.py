#!/usr/bin/env python
from __future__ import print_function
import random

# Allowed ArUco IDs
ALLOWED_IDS = [18, 35, 48, 50, 62, 72, 129, 138, 162, 201, 226, 264, 288, 392]

# Marker and roof geometry (in meters)
MARKER_SIZE = 0.4        # 40 cm
ROOF_SIZE = 6.2          # roof is 6.2 x 6.2
ROOF_HALF = ROOF_SIZE / 2.0
MARGIN = MARKER_SIZE / 2.0

# Allowed coordinate range for marker centers so they stay fully on the roof
MIN_COORD = -ROOF_HALF + MARGIN
MAX_COORD = ROOF_HALF - MARGIN


def generate_non_overlapping_positions(n, size, min_coord, max_coord, max_attempts=10000):
    """Generate n non-overlapping axis-aligned square centers.

    Overlap check: two squares overlap if both |dx| < size and |dy| < size.
    """
    positions = []
    for _ in range(n):
        for _attempt in range(max_attempts):
            x = random.uniform(min_coord, max_coord)
            y = random.uniform(min_coord, max_coord)

            ok = True
            for (px, py) in positions:
                if abs(x - px) < size and abs(y - py) < size:
                    ok = False
                    break
            if ok:
                positions.append((x, y))
                break
        else:
            # If we somehow fail to place all (shouldn't happen with these numbers),
            # just stop early.
            break
    return positions


def main():
    # Random number of markers between 1 and 8
    n = random.randint(1, 8)

    # Generate non-overlapping (x, y) positions in roof frame
    positions = generate_non_overlapping_positions(
        n, MARKER_SIZE, MIN_COORD, MAX_COORD
    )

    # Randomly choose ArUco IDs for each marker
    marker_ids = [random.choice(ALLOWED_IDS) for _ in range(len(positions))]

    # ----- WORLD HEADER (everything before roof ArUcos) -----
    header = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <wind>
      <linear_velocity>11 11 0</linear_velocity>
    </wind>
    <scene>
      <shadows>0</shadows>
    </scene>
    <gui>
      <camera name="user_camera">
        <pose>-5 0 1 0 0.2 0</pose>
      </camera>
    </gui>
    <physics type="ode">
      <ode>
        <solver>
          <type>quick</type>
          <iters>100</iters>
          <sor>1.0</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>0.1</contact_max_correcting_vel>
          <contact_surface_layer>0.0</contact_surface_layer>
        </constraints>
      </ode>
      <real_time_update_rate>-1</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>

    <include>
      <uri>model://sun</uri>
    </include>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>5000 5000</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
          </surface>
        </collision>

        <visual name="background">
          <pose>0 0 -0.1 0 0 0</pose>
          <cast_shadows>false</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>50 50</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grass</name>
            </script>
          </material>
        </visual>

        <!-- Optional ground ArUco -->
        <visual name="qrcode1">
          <pose>0 0 0.005 0 0 0</pose>
          <cast_shadows>false</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>0.2 0.2</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo2.material</uri>
              <name>Gazebo/Aruco72</name>
            </script>
          </material>
        </visual>

      </link>
    </model>

    <!-- HOUSE MODEL -->
    <model name="house">
      <pose>10 10 0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <!-- WALLS -->
        <collision name="wall_north">
          <pose>0 3 1 0 0 0</pose>
          <geometry>
            <box>
              <size>6 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="wall_north">
          <pose>0 3 1 0 0 0</pose>
          <geometry>
            <box>
              <size>6 0.2 2</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>

        <collision name="wall_south">
          <pose>0 -3 1 0 0 0</pose>
          <geometry>
            <box>
              <size>6 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="wall_south">
          <pose>0 -3 1 0 0 0</pose>
          <geometry>
            <box>
              <size>6 0.2 2</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>

        <collision name="wall_east">
          <pose>3 0 1 0 0 0</pose>
          <geometry>
            <box>
              <size>0.2 6 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="wall_east">
          <pose>3 0 1 0 0 0</pose>
          <geometry>
            <box>
              <size>0.2 6 2</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>

        <collision name="wall_west">
          <pose>-3 0 1 0 0 0</pose>
          <geometry>
            <box>
              <size>0.2 6 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="wall_west">
          <pose>-3 0 1 0 0 0</pose>
          <geometry>
            <box>
              <size>0.2 6 2</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>

        <!-- ROOF -->
        <collision name="roof">
          <pose>0 0 2.2 0 0 0</pose>
          <geometry>
            <box>
              <size>6.2 6.2 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="roof">
          <pose>0 0 2.2 0 0 0</pose>
          <geometry>
            <box>
              <size>6.2 6.2 0.2</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Blue</name>
            </script>
          </material>
        </visual>

        <!-- RANDOM ROOF ARUCO MARKERS -->
"""
    # ----- WORLD FOOTER (everything after the markers) -----
    footer = """      </link>
    </model>

    <model name="iris_demo_test">
      <enable_wind>true</enable_wind>
      <include>
        <uri>model://iris_with_ardupilot</uri>
      </include>
    </model>

  </world>
</sdf>
"""

    # Print header
    print(header, end='')

    # Print each random ArUco visual on the roof
    for i, ((x, y), marker_id) in enumerate(zip(positions, marker_ids), start=1):
        visual_name = "aruco_roof_{}_{}".format(marker_id, i)
        material_name = "Gazebo/Aruco{}".format(marker_id)
        # z is fixed just above the roof: 2.31 like your example
        z = 2.31

        snippet = """        <visual name="{vname}">
          <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
          <cast_shadows>false</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>0.4 0.4</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo2.material</uri>
              <name>{mname}</name>
            </script>
          </material>
        </visual>
""".format(vname=visual_name, x=x, y=y, z=z, mname=material_name)

        print(snippet, end='')

    # Print footer
    print(footer, end='')


if __name__ == "__main__":
    main()

