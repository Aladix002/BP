"""Entry point wrapper so mode_switch is callable as `ros2 run wave_rover_bringup mode_switch`."""
import runpy
import os
import sys


def main():
    script = os.path.join(
        os.path.dirname(__file__), '..', '..', '..', '..', 'share',
        'wave_rover_bringup', 'scripts', 'mode_switch',
    )
    # Fall back to the source tree location during development
    if not os.path.exists(script):
        script = os.path.join(
            os.path.dirname(__file__), '..', 'scripts', 'mode_switch')
    sys.argv[0] = 'mode_switch'
    runpy.run_path(script, run_name='__main__')
