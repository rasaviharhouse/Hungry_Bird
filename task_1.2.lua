function sysCall_init()
    -- This child script contains computing path and publishing it into the path_planning.py for waypoint navigation.
    

    -- Declaring required handles
    drone_handle = sim.getObjectHandle('eDrone')
    collection_handles= sim.getCollectionHandle('Obstacles')
    
    -- Assigning obstacles handles
    no_of_obstacles = 6
    obstacles_handles = {}
    for i=1,no_of_obstacles do
        table.insert(obstacles_handles,sim.getObjectHandle('obstacle_'..tostring(i)))
    end

    -----------Add other required handles here----------------
    goal_handle1 = sim.getObjectHandle('goal_1')
    goal_handle2 = sim.getObjectHandle('goal_2')
    initial_waypoint_handle = sim.getObjectHandle('initial_waypoint')



    --Hint : Goal handles and other required handles
    ----------------------------------------------------------


    ------------Add the path planning task initial details------------------
    t=simOMPL.createTask('t')
    ss={simOMPL.createStateSpace('6d',simOMPL.StateSpaceType.pose3d,drone_handle,{-2.5,-1.7,0.0},{2,1.7,1.7},1)}
    simOMPL.setStateSpace(t,ss)
    simOMPL.setAlgorithm(t,simOMPL.Algorithm.RRTConnect)
    simOMPL.setCollisionPairs(t,{sim.getObjectHandle('eDrone_outer'),collection_handles})

    --no_of_path_points_required = 50

    compute_path_flag = true
    whycon_ground_z_value = 55.60
    request_path_flag=0
    count_no_of_targets = 0
    count_all_targets = 3
    --Hint : Creating task, statespace creation, algorithm setting and setting collision pairs
    -- Carefull about the bounds and the collision pairs you set.
    --------------------------------------------------------------------------


    --------------------Add your publisher and subscriber nodes here ---------------------


    path_pub=simROS.advertise('/vrep/waypoints', 'geometry_msgs/PoseArray')    -- Publish the computed path under the topic /vrep/waypoints

    request_path=simROS.subscribe('/ros/requestpath', 'std_msgs/Int64', 'requestPATH_callback')



    -- Hint : You will require to subscribe to a topic published by path_planning.py indicating the need of new path once all waypoints are covered. THIS IS IMPORTANT
    ---------------------------------------------------------------------------------------
    

    scale_factor = {-0.1322322775,-0.1322880116,-0.0564927745} -- Add the scale_factor you computed learned from the tutorial of whycon transformation
    no_of_path_points_required = 50 -- Add no of path points you want from one point to another

end

function requestPATH_callback( msg )
    if msg.data == 1  then
        request_path_flag = 1
        count_no_of_targets = count_no_of_targets + 1
        compute_path_flag = true
        
    end 
    
end


-- Function to get pose (position and orientation) of a handle in reference to another handle
function getpose(handle,ref_handle)
    position = sim.getObjectPosition(handle,ref_handle)
    orientation = sim.getObjectQuaternion(handle,ref_handle)
    pose = {position[1],position[2],position[3],orientation[1],orientation[2],orientation[3],orientation[4]}
    return pose
end


-- This function can be used to visualize the path you compute. This function takes path points as the argument...
-- GO through the code segment for better understanding
function visualizePath( path )
    if not _lineContainer then
        _lineContainer=sim.addDrawingObject(sim.drawing_lines,1,0,-1,99999,{0.2,0.2,1})
    end
    sim.addDrawingObjectItem(_lineContainer,nil)
    if path then
        local pc=#path/7
        for i=1,pc-1,1 do
            lineDat={path[(i-1)*7+1],path[(i-1)*7+2],path[(i-1)*7+3],path[i*7+1],path[i*7+2],path[i*7+3]}
            sim.addDrawingObjectItem(_lineContainer,lineDat)
        end
    end
end


-- This function is used to send the Path computed in the real_world to whycon_world after transformation
-- Message is to be sent in geometry_msgs/PoseArray as WhyCon node subscribes to that message
function packdata(path)

    local sender = {header = {}, poses = {}}
    
    sender['header']={seq=123, stamp=simROS.getTime(), frame_id="drone"}
    sender['poses'] = {}

    for i=1,((no_of_path_points_required-1)*7)+1,7 do
        a = {x = 0, y = 0, w = 0, z = 0}
        b = {x = 0, y = 0, z = 0}
        pose = {position = b, orientation = a, }

        -------------------Add x, y and z value after converting real_world to whycon_world using the computed scale_factor--------------------------------

        pose.position.x = (path[i])/(scale_factor[1])
        pose.position.y = (path[i+1])/(scale_factor[2])
        pose.position.z = whycon_ground_z_value + path[i+2]/(scale_factor[3])
        sender.poses[math.floor(i/7) + 1] = pose


        --------------------------------------------------------------------------------------------------------------------
    end
    -- Debug if the path computed are correct. Display the computed path and see if the path points moves from drone to the target point
    --print(path)
    return sender
end


--- This function is used to compute and publish the path to path_planninglpy
function compute_and_send_path(task)
    local r
    local path


    -- r,path=simOMPL.compute(,,,) -- Provide the correct arguments here.. Make sure you check the no of path points it actually computes
    r,path=simOMPL.compute(t,10,1,no_of_path_points_required)

    if(r == true) then
        visualizePath(path)
        message = packdata(path)  
        simROS.publish(path_pub,message)
        -- Provide slots for debug to cross check the message or path points you recieve after computing
        print(r, #path)
        --print(path)
    end
    return r
end


function sysCall_actuation()
    
    ---- Add your code to set start and goal state after getting present poses of the drone and the new goal when path_planning.py request you a path
    ---------------------------------------------------------------------------------------------------------------
    -- Computing path only once.
    --if flagg == 0 then
        if compute_path_flag == true and request_path_flag == 1 then
            -- Getting startpose
            start_pose = getpose(drone_handle,-1)
            -- Getting the goalpose
            if ( count_no_of_targets == 1) then
                goal_pose = getpose(goal_handle1,-1)

            elseif ( count_no_of_targets == 2) then
                goal_pose = getpose(goal_handle2,-1)

            elseif ( count_no_of_targets == 3) then
                goal_pose = getpose(initial_waypoint_handle,-1)
            end    

             -- Setting start state
            simOMPL.setStartState(t,start_pose)
            -- Setting goal state but the orientation is set same as that of startpose
            simOMPL.setGoalState(t,{goal_pose[1],goal_pose[2],goal_pose[3],start_pose[4],start_pose[5],start_pose[6],start_pose[7]})
            -- Computing path and publishing path points
            --if flagg == 1 then
            status = compute_and_send_path(t)
                --flagg = 0
            --end    
            if(status == true) then -- path computed
                compute_path_flag = false

            end
        end 
    
    ----------------------------------------------------------------------------------------------------------------
    --Hint : Set orientation of the goal as same as that of the start pose as you don't want the drone to rotate on its yaw

end






function sysCall_sensing()

end



function sysCall_cleanup()
    
    --simROS.shutdownSubscriber(request_path)

end

