from controller import Supervisor
import math, time, json, os

LOG_PATH = os.path.join(os.path.dirname(__file__), "hover_log.jsonl")

class Logger(Supervisor):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())
        self.fire = self.getFromDef("FIRE")
        self.drone = self._find_drone()
        self.logged = False

    def _find_drone(self):
        # Try common names, else scan children for a Robot with "Mavic" in name
        candidates = ["MAVIC", "Mavic 2 PRO", "Mavic2Pro"]
        for c in candidates:
            node = self.getFromDef(c)
            if node:
                return node
        root = self.getRoot().getField("children")
        for i in range(root.getCount()):
            n = root.getMFNode(i)
            if n.getTypeName() == "Robot" and "Mavic" in (n.getField("name").getSFString() or ""):
                return n
        return None

    def run(self):
        while self.step(self.timestep) != -1:
            if not self.drone or not self.fire:
                continue
            p = self.drone.getPosition()
            f = self.fire.getField("translation").getSFVec3f()
            dist = math.dist(p[:2], f[:2])
            if dist < 3.0 and not self.logged:
                record = {
                    "ts": time.time(),
                    "drone_gps": p,
                    "fire_gps": f,
                    "note": "hover near fire"
                }
                with open(LOG_PATH, "a") as f_out:
                    f_out.write(json.dumps(record) + "
")
                print(f"[LOG] Hover recorded: {record}")
                self.logged = True

if __name__ == "__main__":
    Logger().run()
