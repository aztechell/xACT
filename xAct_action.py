# === xAct_action.py ===
from pybricks.tools import StopWatch

class Action:
    def __init__(self, robot):
        self.robot = robot
        self._started = False     # для wait()
        self._timer = None
        self._initialized = False # для отложенной инициализации (on_start)
        self._start_failed = False

    # -------------------------
    # Переопределяемые хуки
    # -------------------------
    def on_start(self):
        """
        Вызывается один раз перед первым update().
        Переопределяй в подклассах, если нужно получить
        актуальное состояние робота в момент запуска действия.
        """
        return None

    def update(self):
        """
        Основная логика действия — переопределяется в подклассах.
        Должна возвращать True когда действие завершено, False — если ещё выполняется.
        """
        return True

    # -------------------------
    # Вспомогательные методы
    # -------------------------
    def wait(self, duration_ms):
        """
        Удобная функция для пауз внутри update().
        Возвращает True, когда прошло >= duration_ms миллисекунд с момента первого вызова.
        """
        if not self._started:
            self._timer = StopWatch()
            self._started = True
        return self._timer.time() >= duration_ms

    def _safe_update(self):
        """
        Внутренний вызов: если действие ещё не инициализировано — вызываем on_start(),
        затем вызываем update(). Возвращает результат update().
        """
        if self._start_failed:
            return True
        if not self._initialized:
            try:
                self.on_start()
            except Exception as e:
                # if on_start fails, log and finish the action
                print("Action.on_start() error:", e)
                self._start_failed = True
                self._initialized = True
                return True
            self._initialized = True
        return self.update()


# -------------------------
# Composite actions
# -------------------------
class SequentialAction(Action):
    def __init__(self, robot, actions):
        super().__init__(robot)
        # actions: список объектов Action (можно передать и "лениво" созданные)
        self.actions = actions
        self.index = 0

    def on_start(self):
        # ничего особенного при старте последовательности
        return None

    def update(self):
        # ???? ??? ???-???????? ????????? ? ????????? ??? ??????????????????
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
            # ???????? _safe_update, ? ?? update ????????
            if current._safe_update():
                # ??????? ???????? ??????????? ? ????????? ? ??????????
                self.actions[self.index] = None
                self.index += 1
        except Exception as e:
            # ???????? ??????, ?????????? ?????????? ????????
            print("SequentialAction: sub-action failed:", e)
            self.actions[self.index] = None
            self.index += 1
        return False


class ParallelAction(Action):
    def __init__(self, robot, actions):
        super().__init__(robot)
        # копируем список, чтобы внешние списки не влияли
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
                # ??? ??????? ?????? ???????? ? ?????? ???????? ? ??????? ??? ?? ??????
                print("ParallelAction: sub-action failed:", e)
        self.actions = still_running
        return len(self.actions) == 0
