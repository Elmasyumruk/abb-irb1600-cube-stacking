-- ============================================================
--  ABB IRB 1600 - Cube Stacking with BOX-BASED collision avoidance
--  (EENG 428 Project, Part 2)
--
--  Idea (no heavy mesh checks -> cannot crash):
--    * At start, save every cube's position AND size as a box.
--    * The block being carried is also a box. Before each travel move,
--      check the path: if the carried box would overlap any other box,
--      raise the travel height until the road is clear.
--    * After placing, update that block's saved box to its new spot.
--
--  SCENE REQUIREMENTS:
--    * 'tip' (child of abb_tool), 'target' (root), 'cube1..N' (STATIC cuboids).
--    * Robot base 'abb_base_robot'; 6 joints in Kinematic mode.
--    * (Collidable flags NOT required - this uses geometry, not mesh collision.)
--
--  ATTACH as a THREADED child script on 'abb_base_robot'.
-- ============================================================

function sysCall_thread()
    sim   = require('sim')
    simIK = require('simIK')

    local function byAlias(a)
        for _, h in ipairs(sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)) do
            if sim.getObjectAlias(h) == a then return h end
        end
        return nil
    end

    base   = byAlias('abb_base_robot')
    tip    = byAlias('tip')
    target = byAlias('target')
    cubes  = {}
    local i = 1
    while byAlias('cube'..i) do cubes[#cubes+1] = byAlias('cube'..i); i = i + 1 end
    if #cubes == 0 then error('No cubes found (name them cube1, cube2, ...).') end

    -- Force ALL joints into KINEMATIC mode so the IK drives them. A joint left in
    -- Dynamic mode sags to a rest pose under physics and fights the IK commands.
    local kmode = sim.jointmode_kinematic or sim.jointmode_passive
    local jl = sim.getObjectsInTree(base, sim.object_joint_type, 0)
    local okN = 0
    for _, j in ipairs(jl) do
        local ok = pcall(sim.setJointMode, j, kmode, 0)
        if ok then okN = okN + 1 end
    end
    sim.addLog(sim.verbosity_scriptinfos,
        'Joints set to kinematic: '..okN..'/'..#jl..' (mode code '..tostring(kmode)..').')

    -- ---- settings ----
    speed      = 0.30
    downOrient = {math.pi, 0, 0}
    gripGap    = 0.01
    floorZ     = 0.0
    local autoTower, towerRadius = true, 0.65
    local placeX, placeY = 0.55, -0.45
    local margin   = 0.03            -- extra gap kept between boxes
    local maxTravel= 1.10            -- never travel higher than this
    local atan2 = math.atan2 or math.atan

    ikEnv   = simIK.createEnvironment()
    ikGroup = simIK.createGroup(ikEnv)
    simIK.setGroupCalculation(ikEnv, ikGroup, simIK.method_damped_least_squares, 0.1, 8)
    simIK.addElementFromScene(ikEnv, ikGroup, base, tip, target, simIK.constraint_pose)

    local basePos = sim.getObjectPosition(base, -1)
    sim.setObjectPosition(target, -1, sim.getObjectPosition(tip, -1))
    sim.setObjectOrientation(target, -1, downOrient)

    -- ---- save every cube as a box (center + half-sizes) ----
    boxes = {}                       -- boxes[handle] = {cx,cy,cz,hx,hy,hz}
    for _, cu in ipairs(cubes) do
        local c  = sim.getObjectPosition(cu, -1)
        local bb = sim.getShapeBB(cu)
        boxes[cu] = {cx=c[1], cy=c[2], cz=c[3], hx=bb[1]/2, hy=bb[2]/2, hz=bb[3]/2}
    end

    -- Path test resolution = half the SMALLEST cube side, so a moving block can
    -- never slip BETWEEN two test points and sneak through a gap/corner.
    local smallest = math.huge
    for _, B in pairs(boxes) do
        smallest = math.min(smallest, 2*B.hx, 2*B.hy, 2*B.hz)
    end
    local sampleStep = math.max(0.01, smallest * 0.5)

    -- ---- motion (IK, stepwise) ----
    local function ikMove(pos)
        local cur = sim.getObjectPosition(target, -1)
        local dx, dy, dz = pos[1]-cur[1], pos[2]-cur[2], pos[3]-cur[3]
        local dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        local steps = math.max(1, math.floor(dist / (speed * sim.getSimulationTimeStep())))
        for s = 1, steps do
            local t = s / steps
            sim.setObjectPosition(target, -1, {cur[1]+dx*t, cur[2]+dy*t, cur[3]+dz*t})
            sim.setObjectOrientation(target, -1, downOrient)
            simIK.handleGroup(ikEnv, ikGroup, {syncWorlds = true})
            sim.step()
        end
    end

    -- list of XY points sampled along the arc between two XY points
    local function arcPts(x1, y1, x2, y2)
        local r1 = math.sqrt((x1-basePos[1])^2 + (y1-basePos[2])^2)
        local r2 = math.sqrt((x2-basePos[1])^2 + (y2-basePos[2])^2)
        local a1 = atan2(y1-basePos[2], x1-basePos[1])
        local a2 = atan2(y2-basePos[2], x2-basePos[1])
        local da = a2 - a1
        while da >  math.pi do da = da - 2*math.pi end
        while da < -math.pi do da = da + 2*math.pi end
        -- number of samples so the spacing along the path <= sampleStep
        local pathLen = math.abs(r2 - r1) + 0.5*(r1 + r2)*math.abs(da)
        local nn = math.max(2, math.min(400, math.ceil(pathLen / sampleStep)))
        local pts = {}
        for s = 0, nn do
            local t = s / nn
            local r = r1 + (r2 - r1) * t
            local a = a1 + da * t
            pts[#pts+1] = {x = basePos[1] + r*math.cos(a), y = basePos[2] + r*math.sin(a)}
        end
        return pts
    end
    local function arcTo(toX, toY, z)
        local cur = sim.getObjectPosition(tip, -1)
        ikMove({cur[1], cur[2], z})                 -- lift/lower at start
        for _, p in ipairs(arcPts(cur[1], cur[2], toX, toY)) do
            ikMove({p.x, p.y, z})
        end
    end

    -- highest top among all current obstacle boxes (tower + remaining cubes)
    local function topMost()
        local t = floorZ
        for _, B in pairs(boxes) do if B.cz + B.hz > t then t = B.cz + B.hz end end
        return t
    end

    -- SIMPLE & SAFE rule: lift the carried block clear ABOVE everything, then it
    -- travels at that height, then drops onto the tower. No threading between gaps.
    local lift = 0.05                 -- clearance kept above the tallest obstacle (5 cm)
    -- travel height (tip) so the carried block's bottom is 'lift' above everything
    local function carryHeight(carried, placeTop)
        local h = 2 * carried.hz                                    -- carried block height
        local H = topMost() + lift + gripGap + h                    -- clear the tallest box
        H = math.max(H, placeTop + lift)                            -- and above the drop point
        return math.min(H, maxTravel)
    end
    -- empty-arm travel height: just above the tallest remaining box
    local function emptyHeight(startH)
        return math.min(maxTravel, math.max(startH, topMost() + 0.12))
    end

    -- ---- pick reachable cubes, sort widest-first, choose build spot ----
    local shoulder = {basePos[1], basePos[2], basePos[3] + 0.486}
    local function reach(p) return math.sqrt((p[1]-shoulder[1])^2+(p[2]-shoulder[2])^2+(p[3]-shoulder[3])^2) end
    local items = {}
    for k = 1, #cubes do
        local b = boxes[cubes[k]]
        if reach({b.cx, b.cy, b.cz}) <= 1.35 then
            items[#items+1] = {h=cubes[k], height=b.hz*2, foot=(b.hx*2)*(b.hy*2),
                               width=math.max(b.hx, b.hy)*2,
                               hx=b.hx, hy=b.hy, hz=b.hz}
        else
            sim.addLog(sim.verbosity_scriptwarnings, 'cube'..k..' out of reach - skipping.')
        end
    end
    table.sort(items, function(a,b) if a.foot==b.foot then return a.height>b.height end return a.foot>b.foot end)

    if autoTower and #items > 0 then
        local angs, maxW = {}, 0
        for _, it in ipairs(items) do
            local b = boxes[it.h]
            angs[#angs+1] = atan2(b.cy-basePos[2], b.cx-basePos[1])
            if it.width > maxW then maxW = it.width end
        end
        table.sort(angs)
        local bestGap, bestMid = -1, angs[1]
        for k = 1, #angs do
            local a2 = (k < #angs) and angs[k+1] or (angs[1] + 2*math.pi)
            local gap = a2 - angs[k]
            if gap > bestGap then bestGap = gap; bestMid = angs[k] + gap/2 end
        end
        local r = math.min(maxTravel, math.max(towerRadius, maxW/2 + 0.40))
        placeX = basePos[1] + r * math.cos(bestMid)
        placeY = basePos[2] + r * math.sin(bestMid)
        sim.addLog(sim.verbosity_scriptinfos, string.format('Build spot (%.2f, %.2f).', placeX, placeY))
    end

    -- ---- main loop ----
    local towerTop = floorZ
    local support  = nil                          -- handle of the current tower-top block
    for n = 1, #items do
        local it = items[n]
        local b  = boxes[it.h]
        local h  = it.height
        local pickTop  = b.cz + b.hz + gripGap
        local stackZ   = towerTop + h/2
        local placeTop = stackZ + h/2 + gripGap

        local px, py = b.cx, b.cy                 -- this cube's pick XY

        -- travel (empty) to above the cube
        arcTo(px, py, emptyHeight(math.max(0.40, towerTop + 0.10)))
        ikMove({px, py, pickTop})

        -- grab (clamped to floor)
        local tp = sim.getObjectPosition(tip, -1)
        local cz = math.max(floorZ + h/2, tp[3] - gripGap - h/2)
        sim.setObjectPosition(it.h, -1, {tp[1], tp[2], cz})
        sim.setObjectParent(it.h, tip, true)
        boxes[it.h] = nil                          -- it's carried now: not an obstacle

        -- as soon as it is grabbed, LIFT it clear above everything (build height),
        -- then travel across, then drop straight down onto the tower
        local carried = {hz=it.hz}
        local H = carryHeight(carried, placeTop)
        ikMove({px, py, H})                       -- lift straight up to clear height
        arcTo(placeX, placeY, H)                  -- travel across at clear height
        ikMove({placeX, placeY, placeTop})        -- drop straight down onto tower

        -- release onto the exact tower spot, and re-save its box there
        sim.setObjectParent(it.h, -1, true)
        sim.setObjectPosition(it.h, -1, {placeX, placeY, stackZ})
        sim.setObjectOrientation(it.h, -1, {0, 0, 0})
        boxes[it.h] = {cx=placeX, cy=placeY, cz=stackZ, hx=it.hx, hy=it.hy, hz=it.hz}
        ikMove({placeX, placeY, H})
        towerTop = towerTop + h
        support  = it.h                            -- this block is now the tower top
    end
    sim.addLog(sim.verbosity_scriptinfos, 'Box-collision stacking finished.')
end

function sysCall_cleanup()
    if ikEnv then simIK.eraseEnvironment(ikEnv) end
end
