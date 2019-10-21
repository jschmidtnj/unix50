import utils
sched = utils.Scheduler()
sched.use_simulated(True)


sched.add_robot('codebox0', utils.bottom_N_close2wall)

