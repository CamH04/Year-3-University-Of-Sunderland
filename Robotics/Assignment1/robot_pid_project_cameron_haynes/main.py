import sys
from robot_pid.simulator import Simulator


def main():
    try:
        sim = Simulator()
        sim.run()
    #graceful exit on keyboard interrupt
    except KeyboardInterrupt:
        print("\n\nSimulation interrupted by user.")
        sys.exit(0)
    except Exception as e:
        print(f"\nError occurred during simulation: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
