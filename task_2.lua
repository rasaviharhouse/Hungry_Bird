-- This script is used for realtime emulation of the environment in V-REP


--  Team Id : HB #6265
--  Author List : Shri Ajay Kumar R, Vignesh R, B Tarun Singh, Ajay Vijayakumar 
--  Filename: embScript.lua
--  Theme: eYRC- Hungry Bird
--  Functions: sysCall_init(), sysCall_cleanup(), aruco_callback(), whycon_callback(), key_callback()
--  Global Variables: scale_factor, gv



function sysCall_init()

    scale_factor = {-0.1322322775,-0.1322880116,-0.0564927745}

    --Required handles
    
    Position_hoop_1_handle = sim.getObjectHandle('Position_hoop1')
    Position_hoop_2_handle = sim.getObjectHandle('Position_hoop2')
    Position_hoop_3_handle = sim.getObjectHandle('Position_hoop3')
    Orientation_hoop_1_handle = sim.getObjectHandle('Orientation_hoop1')
    Orientation_hoop_2_handle = sim.getObjectHandle('Orientation_hoop2')
    Orientation_hoop_3_handle = sim.getObjectHandle('Orientation_hoop3')
    obstacle_1_handle = sim.getObjectHandle('obstacle_1')
    obstacle_2_handle = sim.getObjectHandle('obstacle_2')

    gv = 0	-- a global variable to detect key press
    
    aruco_sub = simROS.subscribe('/aruco_marker_publisher/markers', 'aruco_msgs/MarkerArray', 'aruco_callback')
    whycon_sub = simROS.subscribe('/whycon/poses', 'geometry_msgs/PoseArray', 'whycon_callback')
    key_input = simROS.subscribe('/input_key', 'std_msgs/Int16', 'key_callback')
end


function sysCall_actuation()

end

function sysCall_sensing()
    
end

function sysCall_cleanup()
   -- Unsubscribe the callback associated with the subscribers
    simROS.shutdownSubscriber(aruco_sub)
    simROS.shutdownSubscriber(whycon_sub)
    simROS.shutdownSubscriber(key_input)
end



function aruco_callback(msg)
    -- Get the orientation(quaternion) of the ArUco marker 9 set the orientation of the hoop using Orientation_hoop dummy
    
    -- Setting orientation for 'Sal tree'
    if(gv==2) then
        ori1 = {(msg.markers[2].pose.pose.orientation.x),(msg.markers[2].pose.pose.orientation.y),(msg.markers[2].pose.pose.orientation.z),(msg.markers[2].pose.pose.orientation.w)}
        d1 = math.sqrt(math.pow(msg.markers[2].pose.pose.orientation.x,2)+math.pow(msg.markers[2].pose.pose.orientation.y,2)+math.pow(msg.markers[2].pose.pose.orientation.z,2))
        ori_norml1 = { ori1[2]/d1 , ori1[1]/d1 , ori1[3]/d1 , ori1[4]/d1}
        
        print(ori_norml1)

        o1 = sim.setObjectQuaternion(Orientation_hoop_1_handle,-1,ori_norml1)
        gv = 0
    end
    -- Setting orientation for 'Cashew tree'
     if(gv==4) then
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
    
    -- Setting position for 'Sal tree'
    if(gv==1) then
        position = {-(msg.poses[1].position.x*scale_factor[1]) , msg.poses[1].position.y*scale_factor[2] , -(msg.poses[1].position.z)*(scale_factor[3]-0.004)}
        print(position)
        p1 = sim.setObjectPosition(obstacle_1_handle,-1,position)
        gv = 0
    end
    -- Setting position for 'Cashew tree'
    if(gv==3) then
        po = {-(msg.poses[2].position.x*scale_factor[1]) , (msg.poses[2].position.y*scale_factor[2]) ,  -((36-msg.poses[2].position.z)*(scale_factor[3]+0.006)) }
        print(po)
        p2 = sim.setObjectPosition(Position_hoop_3_handle,-1,po)
        gv = 0
    end
    -- Setting position for 'Non-food tree(obstacle)'
    if(gv==5) then
        posi = {-(msg.poses[3].position.x*(-0.16)) , msg.poses[3].position.y*scale_factor[2] , -(msg.poses[3].position.z*(scale_factor[3]-0.011))}
        print(posi)
        p3 = sim.setObjectPosition(obstacle_1_handle,-1,posi)
        gv = 0
    end
end

function key_callback(msg)
    --key pressed = /
    if(msg.data == 500) then
        print('/')
        gv = 1
    end
    --key pressed = v
    if(msg.data == 510) then
        print('v')
        gv = 2
    end
    --key pressed = x
    if(msg.data == 520) then
        print('x')
        gv = 3
    end
    --key pressed = y
    if(msg.data == 530) then
        print('y')
        gv = 4
    end
    --key pressed = .
    if(msg.data == 540) then
        print('.')
        gv = 5
    end
end