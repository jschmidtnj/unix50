import utils
sched = utils.Scheduler()
sched.use_simulated(True)


sched.add_robot('codebox0', utils.bottom_N_close2wall)
sched.add_robot('codebox1', utils.bottom_I_close2wall)

