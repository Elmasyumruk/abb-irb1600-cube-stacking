-- ============================================================
--  ABB IRB 1600 - Cube Stacking Task  (EENG 428 Project, Part 2)
--
--  SELF-CONTAINED IK VERSION (no GUI IK setup needed).
--  Auto-detects ANY number of cubes named cube1, cube2, cube3, ...
--  and measures EACH cube's own height, so cubes of different sizes
--  stack correctly. Cubes are snapped to exact positions for a clean
--  tower even though they are static.
--
--  OBJECTS YOU MUST CREATE IN THE SCENE:
--    * 'tip'    : a Dummy, made a CHILD of the last tool link (abb_tool).
--    * 'target' : a Dummy at the scene ROOT (Dummy type = Default).
--    * 'cube1','cube2','cube3', ... : STATIC cuboids (any number, any size).
--    * Robot base named 'abb_base_robot'.
--  JOINTS: all 6 in 'Kinematic mode'.
--
--  ATTACH as a NON-threaded child script on 'abb_base_robot', paste,
--  then press Play.
-- ============================================================

function sysCall_init()
    sim   = require('sim')
    simIK = require('simIK')

    -- find an object by name anywhere in the scene (nil if missing)
    local function findAlias(alias)
        for _, h in ipairs(sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)) do
            if sim.getObjectAlias(h) == alias then return h end
        end
        return nil
    end
    local function byAlias(alias)
        local h = findAlias(alias)
        if not h then error("Object '" .. alias .. "' not found. Check the name.") end
        return h
    end

    base   = byAlias('abb_base_robot')
    tip    = byAlias('tip')
    target = byAlias('target')

    -- auto-collect cube1, cube2, cube3, ... until one is missing
    cubes = {}
    local i = 1
    while true do
        local h = findAlias('cube' .. i)
        if not h then break end
        cubes[#cubes + 1] = h
        i = i + 1
    end
    if #cubes == 0 then error("No cubes found. Name them cube1, cube2, ...") end

    -- Force ALL joints into KINEMATIC mode so the IK drives them. A joint left in
    -- Dynamic mode sags to a rest pose under physics and fights the IK commands.
    joints = sim.getObjectsInTree(base, sim.object_joint_type, 0)
    for _, j in ipairs(joints) do
        pcall(function() sim.setJointMode(j, sim.jointmode_kinematic or sim.jointmode_passive, 0) end)
    end

    -- Keep the tool pointing straight DOWN (wrist vertical): axes 2, 3 and 5 are
    -- parallel, so the tool's tilt = joint2 + joint3 + joint5. To keep it vertical
    -- we force joint5 = wristConst - joint2 - joint3 every step (see sysCall_actuation).
    -- If the tool is not perfectly vertical, tweak wristConst (try math.rad(-90), or
    -- add/subtract a small offset).
    wristConst = math.rad(90)

    -- ---- build the IK solver in code ----
    ikEnv   = simIK.createEnvironment()
    ikGroup = simIK.createGroup(ikEnv)
    simIK.setGroupCalculation(ikEnv, ikGroup, simIK.method_damped_least_squares, 0.1, 8)
    -- Position-only IK: the IK controls WHERE the tool tip goes (straight onto the
    -- cube's top), while the wrist is kept vertical separately in sysCall_actuation.
    -- This stops the two from fighting, so the arm always grabs from the top.
    simIK.addElementFromScene(ikEnv, ikGroup, base, tip, target, simIK.constraint_position)

    -- ---- settings ----
    speed      = 0.35                  -- m/s tool speed
    settleN    = 8                     -- steps to pause at each waypoint
    downOrient = {math.pi, 0, 0}       -- tool pointing down
    local hover = 0.20                 -- clearance above cube/tower (m)
    local autoTower = true             -- true: auto-pick a cube-free build spot
    local placeX, placeY = 0.55, -0.45 -- manual build spot (used if autoTower=false)
    local towerRadius = 0.65           -- distance from base for the auto build spot
    floorZ  = 0.0                      -- floor height (global: also used at grab time)
    gripGap = 0.01                     -- tool stops this far ABOVE a cube (no clipping)

    -- ---- reachable workspace ----
    -- Reach is measured from the SHOULDER (axis 2, ~0.49 m up), NOT the floor,
    -- so the arm's true reach -- including how HIGH it can stack -- is modelled
    -- correctly (measuring from the floor made it give up far too early).
    local shoulderZ = 0.486           -- IRB 1600 axis-2 height above the base
    local armReach  = 1.35            -- ~spherical reach of the arm from the shoulder
    local minReach  = 0.20            -- too close to the shoulder is not reachable
    local basePos   = sim.getObjectPosition(base, -1)
    local shoulder  = {basePos[1], basePos[2], basePos[3] + shoulderZ}
    local function reach(p)
        local dx, dy, dz = p[1]-shoulder[1], p[2]-shoulder[2], p[3]-shoulder[3]
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    end
    local function reachable(p)
        local d = reach(p)
        return d <= armReach and d >= minReach
    end

    local atan2 = math.atan2 or math.atan

    sim.setObjectPosition(target, -1, sim.getObjectPosition(tip, -1))
    sim.setObjectOrientation(target, -1, downOrient)

    -- minimum travel height above the floor (actual travel height per move is
    -- raised as needed so a carried cube clears the floor AND the growing tower)
    local baseTravel = 0.35

    -- ---- collect reachable cubes with their measured size ----
    local items = {}
    for k = 1, #cubes do
        local c = sim.getObjectPosition(cubes[k], -1)
        if reachable(c) then
            local bb = sim.getShapeBB(cubes[k])          -- {x,y,z} size
            items[#items + 1] = {h = cubes[k], pos = c, height = bb[3],
                                 foot = bb[1]*bb[2],          -- base area (for stack order)
                                 width = math.max(bb[1], bb[2])}  -- widest footprint side
        else
            sim.addLog(sim.verbosity_scriptwarnings,
                'cube'..k..' is OUT OF REACH ('..string.format('%.2f', reach(c))..
                ' m) - skipping it.')
        end
    end

    -- widest FOOTPRINT first -> goes at the BOTTOM (stable); narrower on top.
    -- ties broken by height (taller first) so a wide tall block sits lowest.
    table.sort(items, function(a, b)
        if a.foot == b.foot then return a.height > b.height end
        return a.foot > b.foot
    end)

    -- ---- choose WHERE to build ----
    -- Build in the biggest cube-free direction so the arm never has to reach
    -- BEHIND the tower (which is what made the arm pass through the stack).
    if autoTower and #items > 0 then
        local angs = {}
        local maxWidth = 0
        for _, it in ipairs(items) do
            angs[#angs + 1] = atan2(it.pos[2]-basePos[2], it.pos[1]-basePos[1])
            if it.width > maxWidth then maxWidth = it.width end
        end
        table.sort(angs)
        local bestGap, bestMid = -1, angs[1]
        for i = 1, #angs do
            local a1 = angs[i]
            local a2 = (i < #angs) and angs[i+1] or (angs[1] + 2*math.pi)
            local gap = a2 - a1
            if gap > bestGap then bestGap = gap; bestMid = a1 + gap/2 end
        end
        -- push the build spot far enough out that the biggest block can't
        -- overlap the robot base, but keep it reachable
        local r = math.max(towerRadius, maxWidth/2 + 0.40)
        r = math.min(r, 1.05)
        placeX = basePos[1] + r * math.cos(bestMid)
        placeY = basePos[2] + r * math.sin(bestMid)
        sim.addLog(sim.verbosity_scriptinfos,
            string.format('Auto build spot at (%.2f, %.2f) - clear of cubes and base.', placeX, placeY))
    end
    if not reachable({placeX, placeY, floorZ}) then
        sim.addLog(sim.verbosity_scripterrors,
            'BUILD spot is OUT OF REACH - lower towerRadius or move it closer.')
    end

    -- ---- build waypoints (grab from TOP face, travel in a smooth arc) ----
    waypoints = {}
    -- only 'grab'/'release' waypoints pause (settle); travel points flow through
    local function add(x, y, z, action, cube, rest, settle)
        waypoints[#waypoints + 1] =
            {pos = {x, y, z}, action = action, cube = cube, rest = rest, settle = settle}
    end

    -- arc move around the base, at a given travel height, so the path goes
    -- AROUND the base instead of straight through it. Travel points: no pause.
    local function addArc(x1, y1, x2, y2, z)
        local r1 = math.sqrt((x1-basePos[1])^2 + (y1-basePos[2])^2)
        local r2 = math.sqrt((x2-basePos[1])^2 + (y2-basePos[2])^2)
        local a1 = atan2(y1-basePos[2], x1-basePos[1])
        local a2 = atan2(y2-basePos[2], x2-basePos[1])
        local da = a2 - a1                          -- take the shortest way round
        while da >  math.pi do da = da - 2*math.pi end
        while da < -math.pi do da = da + 2*math.pi end
        local steps = math.max(2, math.ceil(math.abs(da) / 0.12))  -- smooth arc
        for s = 1, steps do
            local t = s / steps
            local r = r1 + (r2 - r1) * t
            local a = a1 + da * t
            add(basePos[1] + r*math.cos(a), basePos[2] + r*math.sin(a), z)
        end
    end

    local startP = sim.getObjectPosition(tip, -1)
    local fromX, fromY = startP[1], startP[2]

    local towerTop = floorZ            -- running top surface of the tower
    local placed = 0
    for n = 1, #items do
        local it = items[n]
        local h  = it.height
        local pickTop  = it.pos[3] + h/2 + gripGap -- stop just ABOVE the cube top
        local stackZ   = towerTop + h/2            -- cube center when placed
        local placeTop = stackZ + h/2 + gripGap    -- stop just ABOVE the placed cube
        -- travel heights: empty arm clears the current tower; loaded arm also
        -- clears the cube it carries -> never underground, never into a cube
        local emptyH  = math.max(baseTravel, towerTop + 0.10)
        local loadedH = math.max(baseTravel, towerTop + h + 0.10)

        -- reachable to pick it, travel it, and place it on the (growing) tower
        local ok = reachable({it.pos[1], it.pos[2], emptyH})
               and reachable({placeX, placeY, loadedH})
               and reachable({placeX, placeY, placeTop})
        if not ok then
            sim.addLog(sim.verbosity_scriptwarnings,
                'Cannot place a cube at tower height '..string.format('%.2f', placeTop)..
                ' m - out of reach, skipping it.')
        else
            local stackRest = {placeX, placeY, stackZ}

            addArc(fromX, fromY, it.pos[1], it.pos[2], emptyH)         -- travel to cube
            add(it.pos[1], it.pos[2], pickTop + hover)                 -- descend above cube
            add(it.pos[1], it.pos[2], pickTop, 'grab', it.h, nil, true)-- grab (pause)
            add(it.pos[1], it.pos[2], loadedH)                        -- lift
            addArc(it.pos[1], it.pos[2], placeX, placeY, loadedH)      -- travel to tower
            add(placeX, placeY, placeTop + hover)                     -- descend above tower
            add(placeX, placeY, placeTop, 'release', it.h, stackRest, true) -- place (pause)
            add(placeX, placeY, loadedH)                             -- retreat
            fromX, fromY = placeX, placeY
            towerTop = towerTop + h
            placed = placed + 1
        end
    end
    sim.addLog(sim.verbosity_scriptinfos,
        'Stacked '..placed..' of '..#cubes..' cubes (widest on the bottom).')

    wpIndex = 1
    settleCount = 0
end

function sysCall_actuation()
    simIK.handleGroup(ikEnv, ikGroup, {syncWorlds = true})  -- arm follows target

    -- force the wrist vertical: joint5 compensates joints 2 and 3 so the tool
    -- always points straight down (90 deg to the ground), whatever the arm pose
    local j2 = sim.getJointPosition(joints[2])
    local j3 = sim.getJointPosition(joints[3])
    sim.setJointPosition(joints[5], wristConst - j2 - j3)
    sim.setJointPosition(joints[4], 0)
    sim.setJointPosition(joints[6], 0)

    if wpIndex > #waypoints then return end

    local wp  = waypoints[wpIndex]
    local cur = sim.getObjectPosition(target, -1)
    local dx, dy, dz = wp.pos[1]-cur[1], wp.pos[2]-cur[2], wp.pos[3]-cur[3]
    local dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    local stepLen = speed * sim.getSimulationTimeStep()

    if dist > stepLen then
        local f = stepLen / dist
        sim.setObjectPosition(target, -1, {cur[1]+dx*f, cur[2]+dy*f, cur[3]+dz*f})
        sim.setObjectOrientation(target, -1, downOrient)
    else
        sim.setObjectPosition(target, -1, wp.pos)
        sim.setObjectOrientation(target, -1, downOrient)
        if wp.settle and settleCount < settleN then
            settleCount = settleCount + 1          -- pause only to grab/release
        else
            settleCount = 0
            if wp.action == 'grab' then
                -- hold the cube just under the tool, but NEVER below the floor
                local h  = sim.getShapeBB(wp.cube)[3]
                local tp = sim.getObjectPosition(tip, -1)
                local cz = tp[3] - gripGap - h/2
                if cz < floorZ + h/2 then cz = floorZ + h/2 end   -- clamp to floor
                sim.setObjectPosition(wp.cube, -1, {tp[1], tp[2], cz})
                sim.setObjectParent(wp.cube, tip, true)
            elseif wp.action == 'release' then
                sim.setObjectParent(wp.cube, -1, true)
                sim.setObjectPosition(wp.cube, -1, wp.rest)
                sim.setObjectOrientation(wp.cube, -1, {0, 0, 0})
            end
            wpIndex = wpIndex + 1
        end
    end
end

function sysCall_cleanup()
    if ikEnv then simIK.eraseEnvironment(ikEnv) end
end
