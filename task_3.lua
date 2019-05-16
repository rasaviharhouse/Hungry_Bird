-- This script is used for realtime emulation of the environment in V-REP

function sysCall_init()

    -- Add required handles here


    drone_handle = sim.getObjectHandle('eDrone')
    scale_factor = {-0.1322322775,-0.1322880116,-0.0564927745}

    -- Subscribing to the required topic
     whycon_sub = simROS.subscribe('/whycon/poses', 'geometry_msgs/PoseArray', 'whycon_callback')
     path_pub=simROS.advertise('/vrep/waypoints', 'geometry_msgs/PoseArray')    -- Publish the computed path under the topic /vrep/waypoints

     whycon_ground_z_value = 53.40
end

-- Function to get pose (position and orientation) of a handle in reference to another handle
function getpose(handle,ref_handle)
    position = sim.getObjectPosition(handle,ref_handle)
    orientation = sim.getObjectQuaternion(handle,ref_handle)
    pose = {position[1],position[2],position[3],orientation[1],orientation[2],orientation[3],orientation[4]}
    return pose
end


function sysCall_actuation()

end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end


function whycon_callback(msg)
    -- Get the position of the real-world whycon marker and set the position of the drone in the simulator.
        --[[position = {-(msg.poses[1].position.x)*scale_factor[1] , msg.poses[1].position.y*scale_factor[2] , -(msg.poses[1].position.z)*scale_factor[3]}
        print(position)

        p1 = sim.setObjectPosition(drone_handle,-1,position)
        ]]

        position = {-(msg.poses[1].position.x*scale_factor[1]) , msg.poses[1].position.y*scale_factor[2] , -((33-msg.poses[1].position.z)*(scale_factor[3]-0.004))}
        print(position)
        p1 = sim.setObjectPosition(drone_handle,-1,position)

end
