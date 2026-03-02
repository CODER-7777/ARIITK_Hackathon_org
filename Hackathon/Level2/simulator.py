"""
Operation Touchdown Simulator - Improved
Drone landing simulation with moving platform, camera feed, and configurable controller.
"""

import pygame
import math
import os

# --- CONSTANTS ---
PIXELS_PER_METER = 100
FPS = 30
SIM_TIME_SEC = 20  # Time until the drone touches the ground
LANDING_SUCCESS_M = 0.35  # Distance from platform center to count as successful landing

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (150, 150, 150)
DARK_GRAY = (80, 80, 80)
BG_COLOR = (45, 90, 50)  # Grass-like background
PLATFORM_COLOR = (250, 250, 250)
DRONE_COLOR = (255, 80, 80)
TRAIL_COLOR = (255, 200, 100)
SUCCESS_COLOR = (80, 255, 120)
FAIL_COLOR = (255, 80, 80)


class DroneSim:
    def __init__(self, use_demo_controller=True):
        """
        Args:
            use_demo_controller: If True, use built-in Python controller (no C code needed).
                                 If False, read commands from commands.txt (for C integration).
        """
        pygame.init()
        self.width, self.height = 800, 800
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("ART IITK - Operation Touchdown Simulator | ESC to quit")

        self.use_demo_controller = use_demo_controller

        # Platform (1m x 1m)
        self.plat_size = 1.0 * PIXELS_PER_METER
        self.plat_x = self.width / 2
        self.plat_y = self.height / 2
        self.plat_speed = 0.5 * PIXELS_PER_METER  # 0.5 m/s
        self.plat_direction = 1
        self.movement_limit = 1.0 * PIXELS_PER_METER
        self.start_x = self.plat_x

        # Drone
        self.drone_x = self.width / 2 + 150
        self.drone_y = self.height / 2 - 100
        self.drone_altitude = 10.0
        self.descent_rate = self.drone_altitude / SIM_TIME_SEC

        # Camera: ground area seen scales with altitude (perspective)
        self.cam_resolution = 100
        self.cam_ground_size_base = 2.5  # meters at altitude 1.0
        self._last_cam_surface = None

        # Trajectory trail
        self.trail = []
        self.trail_max_len = 120

        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 24)
        self.big_font = pygame.font.SysFont(None, 48)

    def _ground_size_at_altitude(self):
        """Ground area (meters) visible in camera scales with altitude."""
        return self.cam_ground_size_base * self.drone_altitude

    def update_platform(self, dt):
        self.plat_x += self.plat_speed * self.plat_direction * dt
        if abs(self.plat_x - self.start_x) > self.movement_limit:
            self.plat_direction *= -1

    def generate_camera_feed(self):
        """Generate down-facing camera image. FOV scales with altitude."""
        ground_m = self._ground_size_at_altitude()
        ground_px = ground_m * PIXELS_PER_METER
        cam_size = int(min(800, max(100, ground_px)))

        cam_surface = pygame.Surface((cam_size, cam_size))
        cam_surface.fill(BG_COLOR)

        # Platform position in world; offset for drone center
        rel_x = self.plat_x - (self.drone_x - cam_size / 2)
        rel_y = self.plat_y - (self.drone_y - cam_size / 2)

        rect = pygame.Rect(
            rel_x - self.plat_size / 2, rel_y - self.plat_size / 2,
            self.plat_size, self.plat_size
        )
        pygame.draw.rect(cam_surface, PLATFORM_COLOR, rect)
        inner = pygame.Rect(
            rel_x - self.plat_size / 4, rel_y - self.plat_size / 4,
            self.plat_size / 2, self.plat_size / 2
        )
        pygame.draw.rect(cam_surface, BLACK, inner)

        # Downsample to 100x100 for C code compatibility
        scaled = pygame.transform.scale(cam_surface, (self.cam_resolution, self.cam_resolution))
        self._last_cam_surface = scaled

        with open("camera_pixels.txt", "w") as f:
            for y in range(self.cam_resolution):
                for x in range(self.cam_resolution):
                    r, g, b, a = scaled.get_at((x, y))
                    gray = int(0.299 * r + 0.587 * g + 0.114 * b)
                    f.write(f"{gray} ")
                f.write("\n")

    def demo_controller(self):
        """Simple proportional controller: steer toward platform center."""
        dx = self.plat_x - self.drone_x
        dy = self.plat_y - self.drone_y
        dist = math.hypot(dx, dy)
        max_vel = 1.2  # m/s
        if dist < 5:
            return 0.0, 0.0
        gain = 0.15 * (1 + 2.0 / max(0.5, self.drone_altitude))
        vx = (dx / dist) * min(max_vel, gain * dist / PIXELS_PER_METER)
        vy = (dy / dist) * min(max_vel, gain * dist / PIXELS_PER_METER)
        return vx, vy

    def read_commands(self):
        vx, vy = 0.0, 0.0
        try:
            if os.path.exists("commands.txt"):
                with open("commands.txt", "r") as f:
                    data = f.read().strip().split()
                    if len(data) >= 2:
                        vx, vy = float(data[0]), float(data[1])
        except Exception:
            pass
        return vx, vy

    def get_commands(self):
        if self.use_demo_controller:
            return self.demo_controller()
        return self.read_commands()

    def run(self):
        running = True
        while running and self.drone_altitude > 0:
            dt = self.clock.tick(FPS) / 1000.0

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False

            # Physics
            self.update_platform(dt)
            self.drone_altitude -= self.descent_rate * dt

            self.generate_camera_feed()
            vx, vy = self.get_commands()
            self.drone_x += vx * PIXELS_PER_METER * dt
            self.drone_y += vy * PIXELS_PER_METER * dt

            # Trail
            self.trail.append((self.drone_x, self.drone_y))
            if len(self.trail) > self.trail_max_len:
                self.trail.pop(0)

            # Render
            self._render()

            pygame.display.flip()

        # Result
        final_error = math.hypot(
            self.drone_x - self.plat_x,
            self.drone_y - self.plat_y
        ) / PIXELS_PER_METER
        success = final_error <= LANDING_SUCCESS_M
        print(f"TOUCHDOWN! Final distance: {final_error:.2f} m — {'SUCCESS' if success else 'FAILED'}")

        self._render_result(success, final_error)
        pygame.display.flip()

        # Wait for key or short delay
        pygame.time.wait(2500)
        for _ in range(90):
            for event in pygame.event.get():
                if event.type in (pygame.QUIT, pygame.KEYDOWN):
                    pygame.quit()
                    return
            pygame.time.wait(50)

        pygame.quit()

    def _draw_grid(self):
        step = PIXELS_PER_METER // 2
        for x in range(0, self.width + 1, step):
            c = DARK_GRAY if x % PIXELS_PER_METER else GRAY
            pygame.draw.line(self.screen, c, (x, 0), (x, self.height), 1)
        for y in range(0, self.height + 1, step):
            c = DARK_GRAY if y % PIXELS_PER_METER else GRAY
            pygame.draw.line(self.screen, c, (0, y), (self.width, y), 1)

    def _render(self):
        self.screen.fill(BG_COLOR)
        self._draw_grid()

        # Trail
        if len(self.trail) >= 2:
            pts = [(int(x), int(y)) for x, y in self.trail]
            pygame.draw.lines(self.screen, TRAIL_COLOR, False, pts, 2)

        # Platform
        plat_rect = pygame.Rect(
            self.plat_x - self.plat_size / 2, self.plat_y - self.plat_size / 2,
            self.plat_size, self.plat_size
        )
        pygame.draw.rect(self.screen, PLATFORM_COLOR, plat_rect, 0)
        pygame.draw.rect(self.screen, WHITE, plat_rect, 2)
        inner = pygame.Rect(
            self.plat_x - self.plat_size / 4, self.plat_y - self.plat_size / 4,
            self.plat_size / 2, self.plat_size / 2
        )
        pygame.draw.rect(self.screen, BLACK, inner)

        # Success zone (faint)
        success_r = int(LANDING_SUCCESS_M * PIXELS_PER_METER)
        s = pygame.Surface((success_r * 2, success_r * 2))
        s.set_alpha(40)
        s.fill(SUCCESS_COLOR)
        self.screen.blit(s, (self.plat_x - success_r, self.plat_y - success_r))
        pygame.draw.circle(self.screen, SUCCESS_COLOR, (int(self.plat_x), int(self.plat_y)), success_r, 1)

        # Drone
        pygame.draw.line(
            self.screen, DRONE_COLOR,
            (self.drone_x - 25, self.drone_y), (self.drone_x + 25, self.drone_y), 3
        )
        pygame.draw.line(
            self.screen, DRONE_COLOR,
            (self.drone_x, self.drone_y - 25), (self.drone_x, self.drone_y + 25), 3
        )
        ground_m = self._ground_size_at_altitude()
        view_r = int(ground_m * PIXELS_PER_METER / 2)
        pygame.draw.circle(self.screen, DRONE_COLOR, (int(self.drone_x), int(self.drone_y)), view_r, 1)

        # Camera preview
        if self._last_cam_surface:
            prev_rect = pygame.Rect(self.width - 115, 10, 105, 105)
            pygame.draw.rect(self.screen, BLACK, prev_rect)
            pygame.draw.rect(self.screen, WHITE, prev_rect, 1)
            scaled = pygame.transform.scale(self._last_cam_surface, (100, 100))
            self.screen.blit(scaled, (self.width - 112, 13))

        # UI
        alt_text = self.font.render(f"Altitude: {self.drone_altitude:.2f} m", True, WHITE)
        err = math.hypot(self.drone_x - self.plat_x, self.drone_y - self.plat_y) / PIXELS_PER_METER
        err_text = self.font.render(f"Error: {err:.2f} m", True, WHITE)
        ctrl_text = self.font.render("Demo" if self.use_demo_controller else "C controller", True, GRAY)
        self.screen.blit(alt_text, (10, 10))
        self.screen.blit(err_text, (10, 32))
        self.screen.blit(ctrl_text, (10, 54))

    def _render_result(self, success, final_error):
        self._render()
        overlay = pygame.Surface((self.width, self.height))
        overlay.set_alpha(180)
        overlay.fill(BLACK)
        self.screen.blit(overlay, (0, 0))

        color = SUCCESS_COLOR if success else FAIL_COLOR
        msg = "LANDING SUCCESS!" if success else "LANDING FAILED"
        text = self.big_font.render(msg, True, color)
        rect = text.get_rect(center=(self.width / 2, self.height / 2 - 30))
        self.screen.blit(text, rect)

        sub = self.font.render(f"Distance from center: {final_error:.2f} m", True, WHITE)
        sub_rect = sub.get_rect(center=(self.width / 2, self.height / 2 + 20))
        self.screen.blit(sub, sub_rect)


if __name__ == "__main__":
    import sys
    use_demo = "--c" not in sys.argv
    sim = DroneSim(use_demo_controller=use_demo)
    print("Running with demo controller. Use --c to read from commands.txt instead.")
    sim.run()
