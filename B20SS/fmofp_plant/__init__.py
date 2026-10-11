"""The plant model: the simulated aircraft and world, as terminal RT 27 on the bus.

In the Python FMOFP the "flight control computer" was a random walk of
altitude, speed and attitude. That is simulation, not flight software, so it
lives here, as test equipment the flight partitions know nothing about: they
only see what the sensors would measure, published as PLANT_SENSORS every
frame (ICD v1.1).
"""
