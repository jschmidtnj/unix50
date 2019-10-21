import utils
sched = utils.Scheduler()
sched.use_simulated(True)

for i in range(100):
    sched.stop_action(i)