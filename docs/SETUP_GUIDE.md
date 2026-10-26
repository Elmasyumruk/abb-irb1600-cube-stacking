# ABB IRB 1600 — CoppeliaSim Setup Guide (EENG 428 Project)

This guide covers the two things you still need:
- **Part 1, Step 4 — "Define Robot Kinematics"** (just *verifying* the robot moves correctly)
- **Part 2 — Cube-stacking task** (scene setup + running the Lua script)

---

## PART 1 — STEP 4: Define / Verify Robot Kinematics

"Define Robot Kinematics" here does **not** mean writing math. In CoppeliaSim it means
proving your joint hierarchy works: each joint moves the correct link, in the correct
direction, within sensible limits. Do this:

### 1. Check the hierarchy (scene tree on the left)
Your tree should be a single chain, base at the top, tool at the bottom:
```
base
 └─ joint1  (revolute, vertical axis)
     └─ link1
         └─ joint2 (revolute)
             └─ link2
                 └─ joint3 (revolute)
                     └─ link3
                         └─ joint4 (revolute)
                             └─ link4
                                 └─ joint5 (revolute)
                                     └─ link5
                                         └─ joint6 (revolute)
                                             └─ tool / flange
```
Each link must be the **child** of the joint that drives it. If a link doesn't move with
its joint, drag it onto the correct joint in the tree to re-parent it.

### 2. Set each joint to "Kinematic" mode for testing
Double-click a joint → **Joint** dialog → *Mode = **Inverse kinematics / Passive*** is fine,
but for manual testing the easiest is the **Position** mode. For now, leave them and use the
manual position slider below.

### 3. Test each DOF individually (this is the actual deliverable)
- Click a joint in the tree.
- Open the **Joint dialog** (double-click) → set a **Position** value, or
- Use the toolbar **"Object/item shift–rotate"** tool and drag the joint.
- Confirm: **Axis 1** rotates the whole arm about the vertical; **Axis 2/3** raise/lower the
  arm; **Axis 4/5/6** move the wrist. Watch for any link moving the wrong way or detaching.

### 4. Set joint limits (working ranges)
Double-click each joint → **Joint** dialog → tick *Position is cyclic = off* →
set **Position min** and **range** to match the ABB IRB 1600 working ranges:

| Joint | Min (deg) | Range (deg) |
|-------|-----------|-------------|
| Axis 1 | −180 | 360 |
| Axis 2 | −63  | 199 |
| Axis 3 | −235 | 290 |
| Axis 4 | −200 | 400 |
| Axis 5 | −115 | 230 |
| Axis 6 | −400 | 800 |

> In CoppeliaSim the field is **min** + **range** (range = max − min).

### 5. Take your Part-1 proof
Press **Play ▶**, drag a couple of joints, and capture a **screenshot or short video**
of the arm moving. That screenshot + the hierarchy screenshot are your Part 1 deliverables.

---

## PART 2 — Cube-Stacking Task Setup

### A. Add three cubes
1. Menu **Add → Primitive shape → Cuboid**.
2. Size **0.05 × 0.05 × 0.05 m**. Create **three** of them.
3. Rename them (double-click the name in the tree) to **`cube1`, `cube2`, `cube3`**.
4. Place them on the table in front of the robot, spread out, e.g.
   `cube1 (0.6, -0.2, 0.025)`, `cube2 (0.7, 0.0, 0.025)`, `cube3 (0.6, 0.2, 0.025)`
   (set position via the **Object properties → Position** tab).
5. **Make them STATIC** so the grab/parent trick is clean: double-click the cube →
   **Shape dynamics properties** → *untick* **Body is dynamic**. (Do this for all 3.)

### B. Create the end-effector point ("tip")
1. **Add → Dummy**. Rename it **`tip`**.
2. Drag **`tip`** in the tree so it becomes a **child of the last link / tool flange**.
3. Position it exactly at the tool tip (the point that should touch the cubes). Use
   **Object properties → Position** to zero it relative to the flange, then nudge the Z out
   to the tip. Orient its **Z axis pointing out of the flange** (the approach direction).

### C. Create the IK target ("target")
1. **Add → Dummy**. Rename it **`target`**.
2. Keep it as a **top-level** object (NOT a child of the robot).
3. Leave its **Dummy type = Default**. **Do NOT** use the "Linked dummy" field.
   (The script positions this dummy and solves the IK for you.)

### D. ~~Build the IK group in the GUI~~  →  SKIP THIS — the script does it
> In newer CoppeliaSim the GUI IK dialog changed and the old "tip-target link" was removed.
> **You do not need it.** The `stack_cubes.lua` script builds the whole IK solver in code
> (using the `simIK` library), so there is nothing to click here.
>
> You only need to make sure:
> - the **`tip`** dummy is a **child of `abb_tool`**,
> - the **`target`** dummy exists at the scene root,
> - all **6 joints are in `Kinematic mode`**.

### E. (Optional) Sanity check
You can't drag-test the arm without the script in this version — that's fine. The script
itself moves the target and solves the IK, so just go straight to step F.

### F. Attach and run the stacking script
1. Right-click **`abb_base_robot`** in the tree → **Add → Associated child script →
   *non-threaded* (plain)**.  ⚠️ Not "Threaded" — this version of the script is non-threaded.
2. Open it (double-click the script icon) and paste the contents of **`stack_cubes.lua`**.
3. Adjust the values in `sysCall_init` if needed:
   - `placeX, placeY` = where the tower is built,
   - `cubeH`   = your cube size,
   - `hover`   = clearance height,
   - `speed`   = how fast the tool moves.
4. Press **Play ▶**. The arm should pick the three cubes and stack them.
5. If the arm can't reach a pose, open the script and change `simIK.constraint_pose` to
   `simIK.constraint_position` (drops the tool-orientation requirement — easier to reach).
6. Capture a **video** of the full stack — that's your Part-2 deliverable.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `Object 'tip' not found` (or target/cube) | Name must be **exactly** `tip`, `target`, `cube1`, `cube2`, `cube3` — no spaces, check spelling/case. |
| Arm doesn't move at all | Script must be **non-threaded** and attached to `abb_base_robot`; all 6 joints in **Kinematic mode**. |
| `simIK` error on Play | The IK plugin should load automatically; restart CoppeliaSim if it doesn't. |
| Cube falls / flies off when grabbed | Cube is still dynamic — untick **Body is dynamic** (make it static). |
| Arm jitters or won't reach a pose | Switch `constraint_pose` → `constraint_position` in the script, or raise damping `0.1 → 0.3`. |
| Tower leans / cubes overlap | Set `cubeH` to your real cube edge length and check the table-top Z height. |
| Robot can't reach a cube | Move cubes closer; IRB 1600 reach is ~1.2–1.45 m from axis 1. |
