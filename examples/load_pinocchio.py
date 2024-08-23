from FlexivPy.robot.model.model_robot import FlexivModel
import numpy as np
import pinocchio as pin




# urdf = "/home/quim/code/FlexivPy/FlexivPy/assets/r10s_with_capsules.urdf"

urdf = "/home/quim/code/FlexivPy/FlexivPy/assets/flexiv_rizon10s_kinematics_w_gripper_mass.urdf"
robot = FlexivModel(render=True, urdf=urdf)



# q = np.array([0, -0.75, 0, 1.5, 0, 0.75, 0])


# joint id = 1
# max of .75
# min of 2.5

q = np.array([-0,  -2.5, 0, 1.5, 0, 0.75, 0])


robot.vizer.displayCollisions(True)
robot.vizer.displayVisuals(True)
robot.vizer.displayFrames(True)

robot.display(q)

input("Press Enter to continue...")



robot.robot.updateGeometryPlacements(q=None, visual=False)
robot.robot.updateGeometryPlacements(q=None, visual=True)


# Compute all the collisions
pin.computeCollisions(robot.robot.model, robot.robot.data, robot.robot.collision_model, 
                      robot.robot.collision_data, q, False)


robot.robot.collision_model.addAllCollisionPairs()


robot.robot.rebuildData()

pin.computeCollisions(robot.robot.model, robot.robot.data, robot.robot.collision_model, 
                      robot.robot.collision_data, q, False)




# print('done')
#
# # Compute all the collisions
# pin.computeCollisions(model, data, geom_model, geom_data, q, False)

# Print the status of collision for all collision pairs
for k in range(len(robot.robot.collision_model.collisionPairs)):
    cr = robot.robot.collision_data.collisionResults[k]
    cp = robot.robot.collision_model.collisionPairs[k]
    print(
        "collision pair:",
        cp.first,
        ",",
        cp.second,
        "- collision:",
        "Yes" if cr.isCollision() else "No",
    )



robot.robot.collision_model.addAllCollisionPairs()

# remove collision pairs from the 


pin.removeCollisionPairs(robot.robot.model, robot.robot.collision_model,
'/home/quim/code/FlexivPy/FlexivPy/assets/r10s_with_capsules.srdf')


print(
    "num collision pairs - after removing useless collision pairs:",
    len(robot.robot.collision_model.collisionPairs),
)


robot.robot.rebuildData()

import time
tic = time.time()
out = pin.computeCollisions(robot.robot.model, robot.robot.data, robot.robot.collision_model, 
                      robot.robot.collision_data, q, False)
print('out is ', out)

toc = time.time()

print('time one collision ', toc-tic)


for k in range(len(robot.robot.collision_model.collisionPairs)):
    cr = robot.robot.collision_data.collisionResults[k]
    cp = robot.robot.collision_model.collisionPairs[k]

    print(robot.robot.collision_model.geometryObjects[cp.first].name)
    print(robot.robot.collision_model.geometryObjects[cp.second].name)

    print(
        "collision pair:",
        cp.first,
        ",",
        cp.second,
        "- collision:",
        "Yes" if cr.isCollision() else "No",
    )

