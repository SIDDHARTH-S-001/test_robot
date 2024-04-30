from time import sleep
from py_trees.behaviour import Behaviour # Execution Node (action / condition)
from py_trees.common import Status # success / failure / running
from py_trees.composites import Sequence
from py_trees import logging as log_tree # logging anything we want to see on the terminal
from py_trees.decorators import Inverter

class Action(Behaviour):
    def __init__(self, name):
        super(Action, self).__init__(name)

    def setup(self):
        self.logger.debug(f"Action::setup {self.name}")

    def initialise(self):
        self.logger.debug(f"Action::initialise {self.name}")

    def update(self):
        self.logger.debug(f"Action::update {self.name}")
        sleep(1)
        return Status.FAILURE
    
    def terminate(self, new_status):
        self.logger.debug(f"Action::terminate {self.name} to {new_status}")

class Condition(Behaviour):
    def __init__(self, name):
        super(Condition, self).__init__(name)

    def setup(self):
        self.logger.debug(f"Condition::setup {self.name}")

    def initialise(self):
        self.logger.debug(f"Condition::initialise {self.name}")

    def update(self):
        self.logger.debug(f"Condition::update {self.name}")
        sleep(1)
        return Status.SUCCESS
    
    def terminate(self, new_status):
        self.logger.debug(f"Condition::terminate {self.name} to {new_status}")

def make_bt():
    root = Sequence(name="sequence", memory=True)

    check_battery = Condition("check_battery")
    open_gripper = Inverter(child=Action("open_gripper"), name="open_gripper_inverter")
    approach_object = Inverter(child=Action("approach_gripper"), name="approach_object_inverter")
    close_gripper = Inverter(child=Action("close_gripper"), name="close_gripper_inverter")

    root.add_children(
        [
            check_battery, 
            open_gripper, 
            approach_object, 
            close_gripper
        ]
    )

    return root

if __name__ == "__main__":
    log_tree.level = log_tree.Level.DEBUG
    tree = make_bt()
    tree.tick_once()

