-- This script is used for realtime emulation of the environment in V-REP


--  Team Id : HB #6265
--  Author List : Shri Ajay Kumar R, Vignesh R, B Tarun Singh, Ajay Vijayakumar 
--  Filename: embScript.lua
--  Theme: eYRC- Hungry Bird
--  Functions: sysCall_init(), sysCall_cleanup(), aruco_callback(), whycon_callback(), key_callback()
--  Global Variables: scale_factor, gv



function sysCall_init()

    -- This child script contains computing path and publishing it into the path_planning.py for waypoint navigation.
    

    -----------Add other required handles here----------------
    goal_handle1 = sim.getObjectHandle('goal_1')
    --goal_handle2 = sim.getObjectHandle('goal_2')
    initial_waypoint_handle = sim.getObjectHandle('initial_waypoint')

    Position_hoop_1_handle = sim.getObjectHandle('Position_hoop3')
    --Position_hoop_3_handle = sim.getObjectHandle('Position_hoop3')
    Orientation_hoop_1_handle = sim.getObjectHandle('Orientation_hoop3')
    --Orientation_hoop_3_handle = sim.getObjectHandle('Orientation_hoop3')
    --obstacle_1_handle = sim.getObjectHandle('obstacle_1')
    drone_handle = sim.getObjectHandle('eDrone')

    -- Declaring required handles

    --collection_handles= sim.getCollectionHandle('Position_hoop_1_handle')
    
    -- Assigning obstacles handles
    no_of_obstacles = 1
    obstacles_handles = {}
   --[[for i=1,no_of_obstacles do
        table.insert(obstacles_handles,sim.getObjectHandle(Position_hoop_1_handle))--'obstacle_'..tostring(i)))
    end
    ]]
    
    gv = 0  -- a global variable to detect key press



    --Hint : Goal handles and other required handles
    ----------------------------------------------------------


    ------------Add the path planning task initial details------------------
    t=simOMPL.createTask('t')
    ss={simOMPL.createStateSpace('6d',simOMPL.StateSpaceType.pose3d,drone_handle,{-2.5,-1.7,0.0},{2,1.7,1.7},1)}
    simOMPL.setStateSpace(t,ss)
    simOMPL.setAlgorithm(t,simOMPL.Algorithm.RRTConnect)
    simOMPL.setCollisionPairs(t,{sim.getObjectHandle('eDrone_outer'),Position_hoop_1_handle})--collection_handles})

    --no_of_path_points_required = 50

    compute_path_flag = true
    whycon_ground_z_value = 55.60
    request_path_flag=0
    count_no_of_targets = 0
    count_all_targets = 1
    --Hint : Creating task, statespace creation, algorithm setting and setting collision pairs
    -- Carefull about the bounds and the collision pairs you set.
    --------------------------------------------------------------------------


    --------------------Add your publisher and subscriber nodes here ---------------------


    path_pub=simROS.advertise('/vrep/waypoints', 'geometry_msgs/PoseArray')    -- Publish the computed path under the topic /vrep/waypoints

    request_path=simROS.subscribe('/ros/requestpath', 'std_msgs/Int64', 'requestPATH_callback')

    aruco_sub = simROS.subscribe('/aruco_marker_publisher/markers', 'aruco_msgs/MarkerArray', 'aruco_callback')
    whycon_sub = simROS.subscribe('/whycon/poses', 'geometry_msgs/PoseArray', 'whycon_callback')
    key_input = simROS.subscribe('/input_key', 'std_msgs/Int16', 'key_callback')



    -- Hint : You will require to subscribe to a topic published by path_planning.py indicating the need of new path once all waypoints are covered. THIS IS IMPORTANT
    ---------------------------------------------------------------------------------------
    

    scale_factor = {-0.1322322775,-0.1322880116,-0.0564927745} -- Add the scale_factor you computed learned from the tutorial of whycon transformation
    no_of_path_points_required = 50 -- Add no of path points you want from one point to another

--    scale_factor = {-0.1322322775,-0.1322880116,-0.0564927745}
    

end






function aruco_callback(msg)
    -- Get the orientation(quaternion) of the ArUco marker 9 set the orientation of the hoop using Orientation_hoop dummy
    
 
    -- Setting orientation for 'Cashew tree'
     if(gv==3) then
         ori2 = {(msg.markers[1].pose.pose.orientation.x),(msg.markers[1].pose.pose.orientation.y ),(msg.markers[1].pose.pose.orientation.z),(msg.markers[1].pose.pose.orientation.w)}
         d2 = math.sqrt((math.pow(msg.markers[1].pose.pose.orientation.x,2)+math.pow(msg.markers[1].pose.pose.orientation.y,2)+math.pow(msg.markers[1].pose.pose.orientation.z,2)+math.pow(msg.markers[1].pose.pose.orientation.w,2)))
         ori_norml2 = { -(ori2[1])/d2 , (ori2[2])/d2 , -(ori2[3])/d2 , (ori2[4])/d2}     
    
         print(ori_norml2)

         o2 = sim.setObjectQuaternion(Orientation_hoop_1_handle,-1,ori_norml2)
         gv = 0
     end
 end


function whycon_callback(msg)
    -- Get the position of the whycon marker and set the position of the food tree and non-food tree using Position_hoop dummy
    
    -- Setting position for 'Cashew tree'
    if(gv==2) then
        position = {-(msg.poses[1].position.x*scale_factor[1]) , msg.poses[1].position.y*scale_factor[2] , -((36-msg.poses[1].position.z)*(scale_factor[3]-0.004))}
        print(position)
        p1 = sim.setObjectPosition(Position_hoop_1_handle,-1,position)
        gv = 0
    end
    -- Setting position for 'Drone'
    if(gv==4) then
        po = {-(msg.poses[2].position.x*scale_factor[1]) , (msg.poses[2].position.y*scale_factor[2]) ,  -((msg.poses[2].position.z)*(scale_factor[3]+0.006)) }
        print(po)
        p2 = sim.setObjectPosition(drone_handle,-1,po)
        gv = 0
    end

end

function key_callback(msg)
    --key pressed = v
    --position of cashew tree
    if(msg.data == 510) then
        print('v')
        gv = 2
    end
    --key pressed = x
    --orientation of cashew tree
    if(msg.data == 520) then
        print('x')
        gv = 3
    end
    --key pressed = y
    --position of drone
    if(msg.data == 530) then
        print('y')
        gv = 4
    end
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
            if(count_no_of_targets == 0 ) then
                goal_pose = getpose(initial_waypoint_handle,-1)
            elseif ( count_no_of_targets == 1) then
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
   -- Unsubscribe the callback associated with the subscribers
    simROS.shutdownSubscriber(aruco_sub)
    simROS.shutdownSubscriber(whycon_sub)
    simROS.shutdownSubscriber(key_input)
end