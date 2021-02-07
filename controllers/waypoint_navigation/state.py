from statemachine import State, StateMachine

class MyRobotModel(object):
    def __init__(self, state):
        self.state = state

class RobotMachine(StateMachine):
    '''A state machine for a block collection robot'''

    # Define State
    #region
    SPAWN           = State('Spawn', initial=True)
    ONPATH          = State('OnPath')

    AVOID_WALL      = State('Avoid_Wall')
    AVOID_BLOCK     = State('Avoid_Block')
    AVOID_ROBOT     = State('Avoid_Robot')

    COLLECTION      = State('Collection')
    RETURN_HOME     = State('ReturnHome')
    UNLOAD          = State('Unload')

    PATH_FINISHED   = State('PathFinished')
    TIME_FINISHED   = State('TimeFinished')

    END             = State('End')

    #endregion

    # Define Transitions
    # region
    start           = SPAWN.to(ONPATH)
    
    detected_wall   = ONPATH.to(AVOID_WALL)
    avoided_wall    = AVOID_WALL.to(ONPATH)

    detected_poo    = ONPATH.to(AVOID_BLOCK)
    avoided_poo     = AVOID_BLOCK.to(ONPATH)

    detected_robot  = ONPATH.to(AVOID_ROBOT)
    avoided_robot   = AVOID_ROBOT.to(ONPATH)

    detected_gold   = ONPATH.to(COLLECTION)
    collected       = COLLECTION.to(RETURN_HOME)
    hustle          = RETURN_HOME.to(UNLOAD)
    unloaded        = UNLOAD.to(ONPATH)

    no_path_left    = ONPATH.to(PATH_FINISHED)
    no_time_left    = ONPATH.to(TIME_FINISHED)

    returnfrompath  = PATH_FINISHED.to(RETURN_HOME)
    returnfromtime  = TIME_FINISHED.to(RETURN_HOME)

    termination     = RETURN_HOME.to(END)
    
    # endregion

    # Transition Messages
    # region
    def on_spawning(self):
        print("Setting on path")
    
    def on_detecting_wall(self):
        print("Detected Wall")
    def on_avoiding_wall(self):
        print("Avoided Wall")
    
    def on_detecting_poo(self):
        print("Detected Poo")
    def on_avoiding_poo(self):
        print("Avoided Poo")
    
    def on_detecting_robot(self):
        print("Detected Robot Nearby")
    def on_avoiding_robot(self):
        print("Avoided Robot Collision")

    def on_detecting_gold(self):
        print("Initiate Collection")
    def on_collection_done(self):
        print("Block Successfully Collected!")
    def on_hustling(self):
        print("Initiate Unloading")
    def on_unloading_done(self):
        print("Returning to path")

    def on_path_finishing(self):
        print("Path Finished: Returning Home")
    def on_time_finishing(self):
        print("No Time Left: Returning Home")
    
    def on_terminating(self):
        print("Terminating Procedure")

    # endregion

# Example Outputs
# robotMachine = RobotMachine()
# print(robotMachine.current_state)
# # print(robotMachine.is_SPAWN)

# robotMachine.start()
# print(robotMachine.current_state)
