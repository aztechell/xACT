# === xAct_action.py ===
from pybricks.tools import StopWatch


def _action_name(action):
    return action.__class__.__name__


def run_actions(robot, actions, odometry=None):
    while actions:
        if odometry is not None:
            odometry.update()

        for action in actions[:]:
            if action is None:
                actions.remove(action)
                continue

            if not hasattr(action, "_safe_update"):
                print("run_actions: invalid action:", action)
                actions.remove(action)
                continue

            if action._safe_update():
                actions.remove(action)


class Action:
    def __init__(self, robot):
        self.robot = robot
        self._started = False     # for wait()
        self._timer = None
        self._initialized = False # for deferred initialization (on_start)
        self._start_failed = False

    def on_start(self):
        """
        Called once before the first update().
        Override this in subclasses when the action needs fresh robot state
        at the moment it starts.
        """
        return None

    def update(self):
        """
        Main action logic. Return True when complete, False while running.
        """
        return True

    def wait(self, duration_ms):
        """
        Non-blocking pause helper for update().
        Returns True after duration_ms has passed since the first call.
        """
        if not self._started:
            self._timer = StopWatch()
            self._started = True
        return self._timer.time() >= duration_ms

    def _safe_update(self):
        """
        Internal update wrapper. Calls on_start() once, then update().
        """
        if self._start_failed:
            return True
        if not self._initialized:
            try:
                self.on_start()
            except Exception as e:
                print(f"{_action_name(self)}.on_start() error: {e}")
                self._start_failed = True
                self._initialized = True
                return True
            self._initialized = True
        return self.update()


class SequentialAction(Action):
    def __init__(self, robot, actions):
        super().__init__(robot)
        self.actions = actions
        self.index = 0

    def on_start(self):
        return None

    def update(self):
        if self.index >= len(self.actions):
            return True

        current = self.actions[self.index]
        if current is None:
            self.index += 1
            return False
        if not hasattr(current, "_safe_update"):
            print("SequentialAction: invalid sub-action:", current)
            self.actions[self.index] = None
            self.index += 1
            return False
        try:
            if current._safe_update():
                self.actions[self.index] = None
                self.index += 1
        except Exception as e:
            print(f"SequentialAction: {_action_name(current)} failed: {e}")
            self.actions[self.index] = None
            self.index += 1
        return False


class ParallelAction(Action):
    def __init__(self, robot, actions):
        super().__init__(robot)
        self.actions = list(actions)

    def on_start(self):
        return None

    def update(self):
        still_running = []
        for act in self.actions:
            if act is None:
                continue
            if not hasattr(act, "_safe_update"):
                print("ParallelAction: invalid sub-action:", act)
                continue
            try:
                if not act._safe_update():
                    still_running.append(act)
            except Exception as e:
                print(f"ParallelAction: {_action_name(act)} failed: {e}")
        self.actions = still_running
        return len(self.actions) == 0


class ConditionalAction(Action):
    def __init__(self, robot, condition, true_action, false_action=None):
        super().__init__(robot)
        self.condition = condition
        self.true_action = true_action
        self.false_action = false_action
        self.selected = None

    def _resolve_action(self, action):
        if action is None:
            return None
        if isinstance(action, (list, tuple)):
            return SequentialAction(self.robot, list(action))
        if callable(action) and not hasattr(action, "_safe_update"):
            return self._resolve_action(action())
        return action

    def on_start(self):
        condition_result = self.condition() if callable(self.condition) else self.condition
        action = self.true_action if condition_result else self.false_action
        self.selected = self._resolve_action(action)

    def update(self):
        if self.selected is None:
            return True
        if not hasattr(self.selected, "_safe_update"):
            print("ConditionalAction: invalid selected action:", self.selected)
            return True
        return self.selected._safe_update()
