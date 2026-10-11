"""The simulated weather: the storm cells and stratiform area of weather_state_manager.py.

In the Python these lived in a singleton that the radar's reflectivity simulator
aged and moved whenever it rendered, and the simulator itself invented a cell or
an area when none existed. The port draws the line between world and sensor:
the plant owns the cells and the area, ages and moves them exactly as that code
did (its random draws included, from the plant's own generator), and publishes
them as PLANT_WEATHER once a second. The radar renders what its antenna would
see from them and processes that volume.
"""

import math

from fmofp_bus.icd.messages import PlantWeather

# weather_state_manager.WeatherStateManager.wind: direction degrees, speed knots.
WIND = {"surface": {"direction": 270.0, "speed": 15.0}, "altitude": {"direction": 290.0, "speed": 25.0}}
MAX_CELLS = 8  # PLANT_WEATHER carries eight; the Python made 3 to 8 and replaced at most what expired


def _drift(position, delta_time, wind):
    """StormCell.update / StratiformArea.update: the wind moves a (azimuth, range) position."""
    wind_speed_km_s = wind["speed"] * 1.852 / 3600
    wind_direction_rad = math.radians(wind["direction"])
    delta_range = wind_speed_km_s * delta_time * math.cos(wind_direction_rad)
    delta_az = math.degrees(wind_speed_km_s * delta_time * math.sin(wind_direction_rad) / (position[1] + 0.001))
    return ((position[0] + delta_az) % 360, position[1] + delta_range)


class StormCell:
    """weather_state_manager.StormCell: position (azimuth deg, range km), radius km, peak dBZ, a lifecycle."""

    def __init__(self, rng, position, size, intensity, entry_profile):
        self.position = position
        self.size = size
        self.intensity = intensity
        self.entry_profile = entry_profile  # PLANT_WEATHER cell_N_entry_profile: which vertical profile
        self.age = 0.0
        self.lifetime = rng.uniform(1800, 7200)
        self.growth_phase = rng.uniform(0.15, 0.3)
        self.mature_phase = rng.uniform(0.4, 0.6)

    def update(self, rng, delta_time, wind):
        """True while the cell lives."""
        self.age += delta_time
        life_fraction = self.age / self.lifetime
        if life_fraction < self.growth_phase:
            self.intensity = 35 + 30 * (life_fraction / self.growth_phase)
        elif life_fraction < (self.growth_phase + self.mature_phase):
            self.intensity = 65 + rng.uniform(-5, 5)
        else:
            decay_progress = ((life_fraction - (self.growth_phase + self.mature_phase))
                              / (1 - (self.growth_phase + self.mature_phase)))
            self.intensity = max(0, 65 - 65 * decay_progress)
        self.position = _drift(self.position, delta_time, wind)
        return self.age < self.lifetime and self.intensity > 5


class StratiformArea:
    """weather_state_manager.StratiformArea: position, azimuth width deg, range depth km, dBZ."""

    def __init__(self, rng, position, az_width, range_depth, intensity):
        self.position = position
        self.az_width = az_width
        self.range_depth = range_depth
        self.intensity = intensity
        self.age = 0.0
        self.lifetime = rng.uniform(3600, 10800)
        self.intensity_variation = 0.0

    def update(self, delta_time, wind):
        self.age += delta_time
        life_fraction = self.age / self.lifetime
        self.intensity_variation = 3.0 * math.sin(self.age / 600)
        if life_fraction > 0.8:
            base_intensity = self.intensity * (1 - ((life_fraction - 0.8) / 0.2) * 0.8)
        else:
            base_intensity = self.intensity
        self.intensity = base_intensity + self.intensity_variation
        self.position = _drift(self.position, delta_time, wind)
        return self.age < self.lifetime and self.intensity > 5


class WeatherWorld:
    """The cells and the area, with ReflectivitySimulator's creation and replacement rules."""

    def __init__(self, rng, populate=True):
        self.rng = rng
        self.cells = []
        self.areas = []
        self.seq = 0
        if populate:
            self.initialize()

    def _initial_cell(self):
        u = self.rng.uniform
        return StormCell(self.rng, (u(0, 360), u(20, 180)), u(5, 20), u(35, 65), False)

    def _initial_area(self):
        u = self.rng.uniform
        return StratiformArea(self.rng, (u(0, 360), u(10, 100)), u(60, 180), u(30, 100), u(15, 30))

    def initialize(self):
        """ReflectivitySimulator._initialize_weather_state: 3-8 cells and one area."""
        for _ in range(self.rng.randint(3, 8)):
            self.cells.append(self._initial_cell())
        self.rng.random()  # the 70 % branch and its fallback both create one area
        self.areas.append(self._initial_area())

    def update(self, delta_time):
        """ReflectivitySimulator._update_weather_systems, then its renderers' rule that the sky is never empty."""
        u = self.rng.uniform
        active = [cell for cell in self.cells if cell.update(self.rng, delta_time, WIND["altitude"])]
        for _ in range(len(self.cells) - len(active)):
            if self.rng.random() < 0.7:
                entry_azimuth = (WIND["altitude"]["direction"] + 180) % 360 + u(-30, 30)
                active.append(StormCell(self.rng, (entry_azimuth, 180.0), u(5, 20), u(35, 45), True))
        self.cells = active
        active_areas = [area for area in self.areas if area.update(delta_time, WIND["surface"])]
        if not active_areas and self.rng.random() < 0.3:
            area_az = (WIND["surface"]["direction"] + 180) % 360 + u(-45, 45)
            active_areas.append(StratiformArea(self.rng, (area_az, 180.0), u(60, 180), u(30, 100), u(15, 30)))
        self.areas = active_areas
        # _render_storm_cells / _render_stratiform_areas created one when there was none to render.
        if not self.cells:
            self.cells.append(self._initial_cell())
        if not self.areas:
            self.areas.append(self._initial_area())

    def message(self):
        """PLANT_WEATHER for the current state."""
        m = PlantWeather()
        self.seq = (self.seq + 1) % 256
        m.weather_seq = self.seq
        cells = self.cells[:MAX_CELLS]
        m.cell_count = len(cells)
        for n, cell in enumerate(cells, start=1):
            setattr(m, f"cell_{n}_azimuth", cell.position[0] % 360)
            setattr(m, f"cell_{n}_range", max(0.0, cell.position[1]))
            setattr(m, f"cell_{n}_size", cell.size)
            setattr(m, f"cell_{n}_entry_profile", cell.entry_profile)
            setattr(m, f"cell_{n}_intensity", max(0.0, cell.intensity))
        if self.areas:
            area = self.areas[0]
            m.area_count = 1
            m.area_azimuth = area.position[0] % 360
            m.area_range = max(0.0, area.position[1])
            m.area_width = area.az_width
            m.area_depth = area.range_depth
            m.area_intensity = max(0.0, area.intensity)
        return m
